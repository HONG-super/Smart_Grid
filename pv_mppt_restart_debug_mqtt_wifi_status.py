from machine import Pin, I2C, ADC, PWM
import utime
import network
import machine

try:
    import ubinascii
except:
    ubinascii = None

try:
    from umqtt.simple import MQTTClient
except ImportError:
    MQTTClient = None


# ============================================================
# PV MPPT with PV Return / Restart State Machine
#
# Port A = DC bus
# Port B = PV side
# ============================================================


# =========================
# WiFi / MQTT setup
# =========================

WIFI_SSID = "Hong"
WIFI_PASSWORD = "abcdefgh"

MQTT_ENABLED = True
MQTT_BROKER_HOST = "172.20.10.4"
MQTT_BROKER_PORT = 1884

if ubinascii is not None:
    MQTT_CLIENT_ID = b"team01_pv_" + ubinascii.hexlify(machine.unique_id())
else:
    MQTT_CLIENT_ID = b"team01_pv"

MQTT_TOPIC_PV_TELEMETRY = b"smartgrid/team01/pv/telemetry"

MQTT_PUBLISH_PERIOD_MS = 1000
WIFI_RETRY_PERIOD_MS = 5000
MQTT_RETRY_PERIOD_MS = 3000

wlan = network.WLAN(network.STA_IF)
wlan.active(True)

mqtt_client = None
last_wifi_retry_ms = None
last_mqtt_retry_ms = None


# =========================
# Hardware setup
# =========================

va_pin = ADC(Pin(28))
vb_pin = ADC(Pin(26))

ina_i2c = I2C(0, scl=Pin(1), sda=Pin(0), freq=400000)

pwm = PWM(Pin(9))
pwm.freq(100000)

SHUNT_OHMS = 0.10

# PV is on port B, DC bus is on port A
PV_ON_PORT_A = False

# Current sign correction
IPV_SIGN = -1


# =========================
# Control settings
# =========================

DUTY_MIN = 1000
DUTY_MAX = 64536

DUTY_STANDBY = 21000
DUTY_STARTUP = 21000

duty = DUTY_STANDBY

# Inner PI voltage loop
KP = 1000.0
KI = 20.0
PI_PERIOD_MS = 5

INT_MAX = 30000
INT_MIN = -30000
mppt_err_int = 0.0

# Software low-pass filter
FILTER_ALPHA = 0.15
vpv_filtered = 0.0

FILTER_ALPHA_P = 0.15
ppv_filtered = 0.0

# Outer P&O loop
VREF_STEP = 0.02
VREF_MIN = 2.0
VREF_MAX = 7.8
MPPT_PERIOD_MS = 300

v_ref = 6.5


# =========================
# Safety limits
# =========================

VPV_MIN_SAFE = 2.0
VPV_MAX_SAFE = 10.5
IPV_MAX_SAFE = 1.20
VBUS_MAX_SAFE = 10.8


# =========================
# PV state machine
# =========================

STATE_WAIT_FOR_PV = 0
STATE_PV_STARTUP = 1
STATE_MPPT_RUNNING = 2
STATE_BUS_HIGH_PAUSE = 3
STATE_FAULT = 4

state = STATE_WAIT_FOR_PV

PV_PRESENT_VOLTAGE = 3.0

PV_MIN_CURRENT_TO_RUN = 0.03
PV_MIN_POWER_TO_RUN = 0.10

PV_STARTUP_HOLD_MS = 1000
startup_start_ms = 0

PV_LOST_CONFIRM_MS = 500
pv_lost_start_ms = 0
pv_lost_timer_active = False


# =========================
# Test duration
# =========================

TEST_DURATION_MS = 60000


# =========================
# Logging
# =========================

SAVE_RESULTS = False
RESULTS_FILE = "pv_mppt_restart_debug.csv"

LOG_PERIOD_MS = 10
LOG_FLUSH_PERIOD_MS = 1000
PRINT_PERIOD_MS = 1000


# ============================================================
# INA219
# ============================================================

class ina219:
    REG_CONFIG = 0x00
    REG_SHUNTVOLTAGE = 0x01
    REG_CALIBRATION = 0x05

    def __init__(self, shunt, address):
        self.address = address
        self.shunt = shunt

    def vshunt(self):
        val = int.from_bytes(
            ina_i2c.readfrom_mem(
                self.address,
                self.REG_SHUNTVOLTAGE,
                2
            ),
            "big"
        )

        if val > 0x7FFF:
            val -= 0x10000

        return val * 1e-5

    def configure(self):
        ina_i2c.writeto_mem(
            self.address,
            self.REG_CONFIG,
            b"\x19\x9F"
        )

        ina_i2c.writeto_mem(
            self.address,
            self.REG_CALIBRATION,
            b"\x00\x00"
        )


# ============================================================
# Helper functions
# ============================================================

def saturate(x, upper, lower):
    if x > upper:
        return upper

    if x < lower:
        return lower

    return x


def state_name(s):
    if s == STATE_WAIT_FOR_PV:
        return "WAIT"

    if s == STATE_PV_STARTUP:
        return "STARTUP"

    if s == STATE_MPPT_RUNNING:
        return "MPPT"

    if s == STATE_BUS_HIGH_PAUSE:
        return "BUS_HIGH"

    if s == STATE_FAULT:
        return "FAULT"

    return "UNKNOWN"


def read_power(ina):
    va = (
        1.017 *
        (12490 / 2490) *
        3.3 *
        (va_pin.read_u16() / 65536)
    )

    vb = (
        1.015 *
        (12490 / 2490) *
        3.3 *
        (vb_pin.read_u16() / 65536)
    )

    iL = ina.vshunt() / SHUNT_OHMS

    if PV_ON_PORT_A:
        vpv = va
        vbus = vb
    else:
        vpv = vb
        vbus = va

    ipv = IPV_SIGN * iL

    if ipv > 0:
        ppv = vpv * ipv
    else:
        ppv = 0.0

    return va, vb, vpv, ipv, ppv, vbus, iL


def reset_mppt_filters(vpv, ppv):
    global vpv_filtered
    global ppv_filtered
    global p_prev
    global v_prev
    global first_mppt
    global mppt_err_int

    vpv_filtered = vpv
    ppv_filtered = ppv

    p_prev = ppv_filtered
    v_prev = vpv_filtered

    first_mppt = True
    mppt_err_int = 0.0



#WiFi&MQTT functions

def connect_wifi():
    global last_wifi_retry_ms
    global mqtt_client
    if not MQTT_ENABLED:
        return False
    
    if wlan.isconnected():
        return True
    
    now_ms = utime.ticks_ms()
    
    if last_wifi_retry_ms is not None:
        if utime.ticks_diff(
            now_ms,
            last_wifi_retry_ms
        ) < WIFI_RETRY_PERIOD_MS:
            return False

    last_wifi_retry_ms = now_ms
    mqtt_client = None

    try:
        wlan.disconnect()
    except:
        pass

    try:
        print("Connecting to WiFi:", WIFI_SSID)
        wlan.connect(WIFI_SSID, WIFI_PASSWORD)

    except Exception as e:
        print("WiFi connection failed:", e)

    return False


def connect_mqtt():
    global mqtt_client
    global last_mqtt_retry_ms

    if not MQTT_ENABLED:
        return False

    if MQTTClient is None:
        return False

    if not wlan.isconnected():
        return False

    if mqtt_client is not None:
        return True

    now_ms = utime.ticks_ms()

    if last_mqtt_retry_ms is not None:
        if utime.ticks_diff(
            now_ms,
            last_mqtt_retry_ms
        ) < MQTT_RETRY_PERIOD_MS:
            return False

    last_mqtt_retry_ms = now_ms

    try:
        print("Connecting to MQTT:", MQTT_BROKER_HOST)

        mqtt_client = MQTTClient(
            MQTT_CLIENT_ID,
            MQTT_BROKER_HOST,
            port=MQTT_BROKER_PORT,
            keepalive=30
        )

        mqtt_client.connect()

        print("MQTT connected")
        return True

    except Exception as e:
        print("MQTT connection failed:", e)
        mqtt_client = None
        return False


def mqtt_publish_power(power_w):
    global mqtt_client

    if not MQTT_ENABLED:
        return

    if not wlan.isconnected():
        mqtt_client = None
        connect_wifi()
        return

    if not connect_mqtt():
        return

    payload = '{{"power":{:.3f}}}'.format(power_w)

    try:
        mqtt_client.publish(
            MQTT_TOPIC_PV_TELEMETRY,
            payload.encode()
        )

    except Exception as e:
        print("MQTT publish failed:", e)
        mqtt_client = None


# ============================================================
# Start
# ============================================================

try:
    led = Pin("LED", Pin.OUT)

    for _ in range(3):
        led.on()
        utime.sleep(0.15)
        led.off()
        utime.sleep(0.15)

except:
    pass


ina = ina219(SHUNT_OHMS, 64)
ina.configure()

pwm.duty_u16(duty)


if SAVE_RESULTS:
    results = open(RESULTS_FILE, "w")

    results.write(
        "time_ms,Va,Vb,Vpv_raw,Vpv_filt,Ipv,Ppv_raw,Ppv_filt,"
        "Vbus,Vref,duty,state,unsafe,"
        "unsafe_vpv_low,unsafe_vpv_high,unsafe_ipv_high,"
        "unsafe_vbus_high,pv_present,pv_usable,iL\n"
    )

    results.flush()


start_ms = utime.ticks_ms()

last_mppt_ms = start_ms
last_pi_ms = start_ms
last_log_ms = start_ms
last_flush_ms = start_ms
last_print_ms = start_ms
last_mqtt_ms = start_ms

p_prev = 0.0
v_prev = 0.0
first_mppt = True

va, vb, vpv, ipv, ppv, vbus, iL = read_power(ina)

vpv_filtered = vpv
ppv_filtered = ppv

unsafe = False
unsafe_vpv_low = False
unsafe_vpv_high = False
unsafe_ipv_high = False
unsafe_vbus_high = False

pv_present = False
pv_usable = False


print("=======================================")
print("PV MPPT WITH RESTART STATE MACHINE")
print("PV side = B side = Vb")
print("Bus side = A side = Va")
print("CSV file =", RESULTS_FILE)
print("VREF_MIN =", VREF_MIN)
print("VREF_MAX =", VREF_MAX)
print("VPV_MAX_SAFE =", VPV_MAX_SAFE)
print("VBUS_MAX_SAFE =", VBUS_MAX_SAFE)
print("DUTY_STANDBY =", DUTY_STANDBY)
print("DUTY_STARTUP =", DUTY_STARTUP)
print("DUTY_MAX =", DUTY_MAX)
print(
    "MQTT broker = {}:{}".format(
        MQTT_BROKER_HOST,
        MQTT_BROKER_PORT
    )
)
print("MQTT client id =", MQTT_CLIENT_ID)
print("MQTT topic =", MQTT_TOPIC_PV_TELEMETRY)
print("MQTT payload = {power: Ppv}")
print("=======================================")

connect_wifi()


# ============================================================
# Main loop
# ============================================================

while True:
    now_ms = utime.ticks_ms()
    elapsed_ms = utime.ticks_diff(now_ms, start_ms)

    if elapsed_ms >= TEST_DURATION_MS:
        pwm.duty_u16(DUTY_STANDBY)

        if SAVE_RESULTS:
            results.flush()
            results.close()

        print("Test finished. PWM set to standby.")
        break

    # --------------------------------------------------------
    # Fast loop: measurements + state machine + PI
    # --------------------------------------------------------

    if utime.ticks_diff(now_ms, last_pi_ms) >= PI_PERIOD_MS:
        last_pi_ms = now_ms

        va, vb, vpv, ipv, ppv, vbus, iL = read_power(ina)

        vpv_filtered = (
            FILTER_ALPHA * vpv +
            (1.0 - FILTER_ALPHA) * vpv_filtered
        )

        ppv_filtered = (
            FILTER_ALPHA_P * ppv +
            (1.0 - FILTER_ALPHA_P) * ppv_filtered
        )

        unsafe_vpv_low = vpv < VPV_MIN_SAFE
        unsafe_vpv_high = vpv > VPV_MAX_SAFE
        unsafe_ipv_high = ipv > IPV_MAX_SAFE
        unsafe_vbus_high = vbus > VBUS_MAX_SAFE

        unsafe = (
            unsafe_vpv_low or
            unsafe_vpv_high or
            unsafe_ipv_high or
            unsafe_vbus_high
        )

        pv_present = vpv > PV_PRESENT_VOLTAGE

        pv_usable = (
            ipv > PV_MIN_CURRENT_TO_RUN and
            ppv > PV_MIN_POWER_TO_RUN
        )

        # ====================================================
        # State machine
        # ====================================================

        if unsafe_vbus_high:
            state = STATE_BUS_HIGH_PAUSE

        if unsafe_ipv_high:
            state = STATE_FAULT

        if state == STATE_WAIT_FOR_PV:
            duty = DUTY_STANDBY
            pwm.duty_u16(duty)

            mppt_err_int = 0.0
            reset_mppt_filters(vpv, ppv)

            if pv_present and not unsafe_vbus_high:
                state = STATE_PV_STARTUP
                startup_start_ms = now_ms

                duty = DUTY_STARTUP
                pwm.duty_u16(duty)

                reset_mppt_filters(vpv, ppv)

        elif state == STATE_PV_STARTUP:
            duty = DUTY_STARTUP
            pwm.duty_u16(duty)

            mppt_err_int = 0.0

            if vpv < VREF_MIN:
                v_ref = VREF_MIN

            elif vpv > VREF_MAX:
                v_ref = VREF_MAX

            else:
                v_ref = vpv

            reset_mppt_filters(vpv, ppv)

            if utime.ticks_diff(
                now_ms,
                startup_start_ms
            ) >= PV_STARTUP_HOLD_MS:

                if (
                    pv_usable and
                    not unsafe_vbus_high and
                    not unsafe_ipv_high
                ):
                    state = STATE_MPPT_RUNNING
                    first_mppt = True
                    reset_mppt_filters(vpv, ppv)

                elif not pv_present:
                    state = STATE_WAIT_FOR_PV

                else:
                    startup_start_ms = now_ms

        elif state == STATE_MPPT_RUNNING:
            if not pv_present:
                if not pv_lost_timer_active:
                    pv_lost_timer_active = True
                    pv_lost_start_ms = now_ms

                elif utime.ticks_diff(
                    now_ms,
                    pv_lost_start_ms
                ) >= PV_LOST_CONFIRM_MS:

                    pv_lost_timer_active = False
                    state = STATE_WAIT_FOR_PV

            else:
                pv_lost_timer_active = False

            if unsafe_vbus_high:
                state = STATE_BUS_HIGH_PAUSE

            elif unsafe_ipv_high:
                state = STATE_FAULT

            if state == STATE_MPPT_RUNNING:
                v_err = vpv_filtered - v_ref

                mppt_err_int = saturate(
                    mppt_err_int + v_err,
                    INT_MAX,
                    INT_MIN
                )

                pi_out = (
                    KP * v_err +
                    KI * mppt_err_int
                )

                duty = int(
                    saturate(
                        duty + pi_out,
                        DUTY_MAX,
                        DUTY_MIN
                    )
                )

                pwm.duty_u16(duty)

        elif state == STATE_BUS_HIGH_PAUSE:
            duty = DUTY_STANDBY
            pwm.duty_u16(duty)

            mppt_err_int = 0.0
            reset_mppt_filters(vpv, ppv)

            if (
                vbus < VBUS_MAX_SAFE - 0.3 and
                pv_present
            ):
                state = STATE_PV_STARTUP
                startup_start_ms = now_ms

            if not pv_present:
                state = STATE_WAIT_FOR_PV

        elif state == STATE_FAULT:
            duty = DUTY_STANDBY
            pwm.duty_u16(duty)

            mppt_err_int = 0.0
            reset_mppt_filters(vpv, ppv)

            if (
                not unsafe_ipv_high and
                not unsafe_vbus_high
            ):
                if pv_present:
                    state = STATE_PV_STARTUP
                    startup_start_ms = now_ms

                else:
                    state = STATE_WAIT_FOR_PV

    # --------------------------------------------------------
    # Slow MPPT outer loop
    # --------------------------------------------------------

    if utime.ticks_diff(
        now_ms,
        last_mppt_ms
    ) >= MPPT_PERIOD_MS:

        last_mppt_ms = now_ms

        if state == STATE_MPPT_RUNNING:
            if first_mppt:
                p_prev = ppv_filtered
                v_prev = vpv_filtered
                first_mppt = False

            else:
                delta_p = ppv_filtered - p_prev
                delta_v = vpv_filtered - v_prev

                if abs(delta_v) > 0.005:
                    if delta_p > 0.0:
                        if delta_v > 0.0:
                            v_ref += VREF_STEP
                        else:
                            v_ref -= VREF_STEP

                    else:
                        if delta_v > 0.0:
                            v_ref -= VREF_STEP
                        else:
                            v_ref += VREF_STEP

                    v_ref = saturate(
                        v_ref,
                        VREF_MAX,
                        VREF_MIN
                    )

                p_prev = ppv_filtered
                v_prev = vpv_filtered

    # --------------------------------------------------------
    # Logging
    # --------------------------------------------------------

    if (
        SAVE_RESULTS and
        utime.ticks_diff(
            now_ms,
            last_log_ms
        ) >= LOG_PERIOD_MS
    ):
        last_log_ms = now_ms

        results.write(
            "{},{:.3f},{:.3f},{:.3f},{:.3f},"
            "{:.5f},{:.5f},{:.5f},"
            "{:.3f},{:.3f},{},{},{},"
            "{},{},{},{},{},{},{:.5f}\n".format(
                elapsed_ms,
                va,
                vb,
                vpv,
                vpv_filtered,
                ipv,
                ppv,
                ppv_filtered,
                vbus,
                v_ref,
                duty,
                state_name(state),
                int(unsafe),
                int(unsafe_vpv_low),
                int(unsafe_vpv_high),
                int(unsafe_ipv_high),
                int(unsafe_vbus_high),
                int(pv_present),
                int(pv_usable),
                iL
            )
        )

    if (
        SAVE_RESULTS and
        utime.ticks_diff(
            now_ms,
            last_flush_ms
        ) >= LOG_FLUSH_PERIOD_MS
    ):
        last_flush_ms = now_ms
        results.flush()

    # --------------------------------------------------------
    # MQTT telemetry
    # --------------------------------------------------------

    if utime.ticks_diff(
        now_ms,
        last_mqtt_ms
    ) >= MQTT_PUBLISH_PERIOD_MS:

        last_mqtt_ms = now_ms
        mqtt_publish_power(ppv)

    # --------------------------------------------------------
    # Printing
    # --------------------------------------------------------

    if utime.ticks_diff(
        now_ms,
        last_print_ms
    ) >= PRINT_PERIOD_MS:

        last_print_ms = now_ms

        print(
            "t={}ms | state={} | Vpv={:.2f}V | Vfilt={:.2f}V | "
            "Ipv={:.3f}A | Ppv={:.2f}W | Vbus={:.2f}V | "
            "Vref={:.2f}V | duty={} | unsafe={} "
            "[vl={}, vh={}, ih={}, bh={}] | WiFi={} | MQTT={}".format(
                elapsed_ms,
                state_name(state),
                vpv,
                vpv_filtered,
                ipv,
                ppv,
                vbus,
                v_ref,
                duty,
                int(unsafe),
                int(unsafe_vpv_low),
                int(unsafe_vpv_high),
                int(unsafe_ipv_high),
                int(unsafe_vbus_high),
                "OK" if wlan.isconnected() else "OFF",
                "OK" if mqtt_client is not None else "OFF"
            )
        )
