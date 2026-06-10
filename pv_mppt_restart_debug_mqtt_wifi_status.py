from machine import Pin, I2C, ADC, PWM
import utime
import network

try:
    from umqtt.simple import MQTTClient
except ImportError:
    MQTTClient = None

# ============================================================
# PV MPPT with PV Return / Restart State Machine
#
# Port A = DC bus
# Port B = PV side
#
# Designed for:
# - Grid SMPS holding bus around 10 V
# - Real PV may have zero input, then suddenly return
# - Lamp turn-on may cause PV open-circuit voltage near 9-10 V
# ============================================================


# =========================
# WiFi / MQTT setup
# =========================

# Fill in your WiFi name and password before running on Pico W.
WIFI_SSID = "Hong"
WIFI_PASSWORD = "abcdefgh"

MQTT_ENABLED = True
MQTT_BROKER_HOST = "172.20.10.4"
MQTT_BROKER_PORT = 1884
MQTT_CLIENT_ID = "team01_pv"
MQTT_TOPIC_PV_TELEMETRY = b"smartgrid/team01/pv/telemetry"

# Only publish power as requested.
# This code has no irradiance sensor / irradiance variable, so irradiance is not published.
MQTT_PUBLISH_PERIOD_MS = 1000
MQTT_CONNECT_TIMEOUT_MS = 8000
MQTT_RETRY_PERIOD_MS = 3000

wlan = None
mqtt_client = None
last_mqtt_retry_ms = 0

# Display-only connection status strings for serial monitor.
wifi_status = "NOT_STARTED"
mqtt_status = "NOT_STARTED"
mqtt_last_error = ""

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
DUTY_MAX = 600000

# Normal startup / standby duty.
# Important: do NOT use 1000 as standby when PV returns.
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

# Optional filtered power for more stable P&O
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

# Raised because real PV can reach high open-circuit voltage
# during lamp turn-on before current is drawn.
VPV_MAX_SAFE = 10.5

IPV_MAX_SAFE = 1.20

# Grid SMPS controls bus around 10 V, so 10.0 V was too low.
VBUS_MAX_SAFE = 10.8

# =========================
# PV state machine thresholds
# =========================

STATE_WAIT_FOR_PV = 0
STATE_PV_STARTUP = 1
STATE_MPPT_RUNNING = 2
STATE_BUS_HIGH_PAUSE = 3
STATE_FAULT = 4

state = STATE_WAIT_FOR_PV

# PV is considered absent when voltage is very low
PV_PRESENT_VOLTAGE = 3.0

# PV is considered usable when current/power becomes positive
PV_MIN_CURRENT_TO_RUN = 0.03
PV_MIN_POWER_TO_RUN = 0.10

# Wait time after PV voltage appears before running MPPT
PV_STARTUP_HOLD_MS = 1000
startup_start_ms = 0

# If PV is lost for this long, return to WAIT_FOR_PV
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
            ina_i2c.readfrom_mem(self.address, self.REG_SHUNTVOLTAGE, 2),
            "big"
        )
        if val > 0x7FFF:
            val -= 0x10000
        return val * 1e-5

    def configure(self):
        ina_i2c.writeto_mem(self.address, self.REG_CONFIG, b"\x19\x9F")
        ina_i2c.writeto_mem(self.address, self.REG_CALIBRATION, b"\x00\x00")


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
    va = 1.017 * (12490 / 2490) * 3.3 * (va_pin.read_u16() / 65536)
    vb = 1.015 * (12490 / 2490) * 3.3 * (vb_pin.read_u16() / 65536)

    iL = ina.vshunt() / SHUNT_OHMS

    if PV_ON_PORT_A:
        vpv, vbus = va, vb
    else:
        vpv, vbus = vb, va

    ipv = IPV_SIGN * iL

    # Only positive PV current counts as generated power.
    if ipv > 0:
        ppv = vpv * ipv
    else:
        ppv = 0.0

    return va, vb, vpv, ipv, ppv, vbus, iL


def reset_mppt_filters(vpv, ppv):
    global vpv_filtered, ppv_filtered
    global p_prev, v_prev, first_mppt
    global mppt_err_int

    vpv_filtered = vpv
    ppv_filtered = ppv
    p_prev = ppv_filtered
    v_prev = vpv_filtered
    first_mppt = True
    mppt_err_int = 0.0




# ============================================================
# WiFi / MQTT helper functions
# ============================================================

def wifi_connect():
    global wlan, wifi_status

    if not MQTT_ENABLED:
        wifi_status = "OFF"
        return False

    if wlan is None:
        wlan = network.WLAN(network.STA_IF)
        wlan.active(True)

    if wlan.isconnected():
        if wifi_status != "CONNECTED":
            print("WiFi connected. IP =", wlan.ifconfig()[0])
        wifi_status = "CONNECTED"
        return True

    wifi_status = "CONNECTING"
    print("WiFi connecting to SSID:", WIFI_SSID)
    wlan.connect(WIFI_SSID, WIFI_PASSWORD)

    start_wait_ms = utime.ticks_ms()
    last_status_print_ms = start_wait_ms

    while (not wlan.isconnected()) and (
        utime.ticks_diff(utime.ticks_ms(), start_wait_ms) < MQTT_CONNECT_TIMEOUT_MS
    ):
        now_wait_ms = utime.ticks_ms()
        if utime.ticks_diff(now_wait_ms, last_status_print_ms) >= 1000:
            last_status_print_ms = now_wait_ms
            print("WiFi still connecting...")
        utime.sleep_ms(200)

    if wlan.isconnected():
        wifi_status = "CONNECTED"
        print("WiFi connected. IP =", wlan.ifconfig()[0])
        return True

    wifi_status = "FAILED"
    print("WiFi connect failed. Check SSID/password/hotspot.")
    return False


def mqtt_connect():
    global mqtt_client, last_mqtt_retry_ms
    global mqtt_status, mqtt_last_error

    if not MQTT_ENABLED:
        mqtt_status = "OFF"
        return None

    if MQTTClient is None:
        mqtt_status = "NO_CLIENT"
        mqtt_last_error = "umqtt.simple missing"
        print("MQTTClient not found. Install umqtt.simple on the Pico W.")
        return None

    if mqtt_client is not None:
        mqtt_status = "CONNECTED"
        return mqtt_client

    now_ms = utime.ticks_ms()
    if utime.ticks_diff(now_ms, last_mqtt_retry_ms) < MQTT_RETRY_PERIOD_MS:
        return None

    last_mqtt_retry_ms = now_ms

    try:
        if not wifi_connect():
            mqtt_status = "WAIT_WIFI"
            return None

        mqtt_status = "CONNECTING"
        mqtt_last_error = ""
        print("MQTT connecting to {}:{} ...".format(
            MQTT_BROKER_HOST,
            MQTT_BROKER_PORT
        ))

        mqtt_client = MQTTClient(
            MQTT_CLIENT_ID,
            MQTT_BROKER_HOST,
            port=MQTT_BROKER_PORT,
            keepalive=30
        )
        mqtt_client.connect()
        mqtt_status = "CONNECTED"
        print("MQTT connected. Publish topic =", MQTT_TOPIC_PV_TELEMETRY)
        return mqtt_client

    except Exception as e:
        mqtt_status = "FAILED"
        mqtt_last_error = str(e)
        print("MQTT connect failed:", e)
        try:
            if mqtt_client is not None:
                mqtt_client.disconnect()
        except:
            pass
        mqtt_client = None
        return None


def mqtt_publish_power(power_w):
    global mqtt_client, mqtt_status, mqtt_last_error

    if not MQTT_ENABLED:
        mqtt_status = "OFF"
        return

    client = mqtt_connect()
    if client is None:
        return

    try:
        # Payload intentionally contains only power.
        # Topic already identifies this as PV telemetry.
        payload = '{{"power":{:.3f}}}'.format(power_w)
        client.publish(MQTT_TOPIC_PV_TELEMETRY, payload)
        mqtt_status = "CONNECTED"
    except Exception as e:
        mqtt_status = "PUBLISH_FAILED"
        mqtt_last_error = str(e)
        print("MQTT publish failed:", e)
        try:
            if mqtt_client is not None:
                mqtt_client.disconnect()
        except:
            pass
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

# Start from standby duty, not 1000
pwm.duty_u16(duty)

if SAVE_RESULTS:
    results = open(RESULTS_FILE, "w")
    results.write(
        "time_ms,Va,Vb,Vpv_raw,Vpv_filt,Ipv,Ppv_raw,Ppv_filt,"
        "Vbus,Vref,duty,state,unsafe,"
        "unsafe_vpv_low,unsafe_vpv_high,unsafe_ipv_high,unsafe_vbus_high,"
        "pv_present,pv_usable,iL\n"
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
print("MQTT broker = {}:{}".format(MQTT_BROKER_HOST, MQTT_BROKER_PORT))
print("MQTT topic =", MQTT_TOPIC_PV_TELEMETRY)
print("MQTT payload = {power: Ppv}")
print("Serial display: WiFi/MQTT connection status enabled")
print("=======================================")

# Try once at startup. If it fails, the main loop will retry without stopping MPPT.
mqtt_connect()

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

        # Update filters
        vpv_filtered = FILTER_ALPHA * vpv + (1.0 - FILTER_ALPHA) * vpv_filtered
        ppv_filtered = FILTER_ALPHA_P * ppv + (1.0 - FILTER_ALPHA_P) * ppv_filtered

        # Split unsafe reasons
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
        pv_usable = (ipv > PV_MIN_CURRENT_TO_RUN) and (ppv > PV_MIN_POWER_TO_RUN)

        # ====================================================
        # State machine
        # ====================================================

        # ---------- bus too high pause ----------
        if unsafe_vbus_high:
            state = STATE_BUS_HIGH_PAUSE

        # ---------- hard current / extreme voltage fault ----------
        # Here Vpv high alone is not treated as fatal because it may be Voc.
        if unsafe_ipv_high:
            state = STATE_FAULT

        # ---------- state actions ----------
        if state == STATE_WAIT_FOR_PV:
            duty = DUTY_STANDBY
            pwm.duty_u16(duty)
            mppt_err_int = 0.0
            reset_mppt_filters(vpv, ppv)

            # When lamp turns on, Vpv may jump high before current is positive.
            # That is enough to enter STARTUP.
            if pv_present and (not unsafe_vbus_high):
                state = STATE_PV_STARTUP
                startup_start_ms = now_ms
                duty = DUTY_STARTUP
                pwm.duty_u16(duty)
                reset_mppt_filters(vpv, ppv)

        elif state == STATE_PV_STARTUP:
            # Hold a known startup duty. Do not run P&O yet.
            duty = DUTY_STARTUP
            pwm.duty_u16(duty)
            mppt_err_int = 0.0

            # Keep Vref close to actual Vpv but within MPPT range.
            if vpv < VREF_MIN:
                v_ref = VREF_MIN
            elif vpv > VREF_MAX:
                v_ref = VREF_MAX
            else:
                v_ref = vpv

            reset_mppt_filters(vpv, ppv)

            # After hold time, enter MPPT if PV starts producing positive power.
            if utime.ticks_diff(now_ms, startup_start_ms) >= PV_STARTUP_HOLD_MS:
                if pv_usable and (not unsafe_vbus_high) and (not unsafe_ipv_high):
                    state = STATE_MPPT_RUNNING
                    first_mppt = True
                    reset_mppt_filters(vpv, ppv)
                elif not pv_present:
                    state = STATE_WAIT_FOR_PV
                else:
                    # Stay in startup. This handles lamp flicker / weak light.
                    startup_start_ms = now_ms

        elif state == STATE_MPPT_RUNNING:
            # If PV disappears, confirm for a short time before leaving MPPT.
            if not pv_present:
                if not pv_lost_timer_active:
                    pv_lost_timer_active = True
                    pv_lost_start_ms = now_ms
                elif utime.ticks_diff(now_ms, pv_lost_start_ms) >= PV_LOST_CONFIRM_MS:
                    pv_lost_timer_active = False
                    state = STATE_WAIT_FOR_PV
            else:
                pv_lost_timer_active = False

            # If bus/current dangerous, leave MPPT
            if unsafe_vbus_high:
                state = STATE_BUS_HIGH_PAUSE
            elif unsafe_ipv_high:
                state = STATE_FAULT

            # Run inner PI only if still in MPPT
            if state == STATE_MPPT_RUNNING:
                # Voltage PI: increasing duty is assumed to pull Vpv DOWN
                v_err = vpv_filtered - v_ref

                mppt_err_int = saturate(
                    mppt_err_int + v_err,
                    INT_MAX,
                    INT_MIN
                )

                pi_out = (KP * v_err) + (KI * mppt_err_int)

                duty = int(saturate(duty + pi_out, DUTY_MAX, DUTY_MIN))
                pwm.duty_u16(duty)

        elif state == STATE_BUS_HIGH_PAUSE:
            # Bus is too high. Do not export more PV power.
            duty = DUTY_STANDBY
            pwm.duty_u16(duty)
            mppt_err_int = 0.0
            reset_mppt_filters(vpv, ppv)

            # Resume when bus falls and PV is present
            if (vbus < VBUS_MAX_SAFE - 0.3) and pv_present:
                state = STATE_PV_STARTUP
                startup_start_ms = now_ms

            if not pv_present:
                state = STATE_WAIT_FOR_PV

        elif state == STATE_FAULT:
            # Current too high. Stop action and wait for safe conditions.
            duty = DUTY_STANDBY
            pwm.duty_u16(duty)
            mppt_err_int = 0.0
            reset_mppt_filters(vpv, ppv)

            if (not unsafe_ipv_high) and (not unsafe_vbus_high):
                if pv_present:
                    state = STATE_PV_STARTUP
                    startup_start_ms = now_ms
                else:
                    state = STATE_WAIT_FOR_PV

    # --------------------------------------------------------
    # Slow MPPT outer loop
    # --------------------------------------------------------

    if utime.ticks_diff(now_ms, last_mppt_ms) >= MPPT_PERIOD_MS:
        last_mppt_ms = now_ms

        if state == STATE_MPPT_RUNNING:
            if first_mppt:
                p_prev = ppv_filtered
                v_prev = vpv_filtered
                first_mppt = False
            else:
                delta_p = ppv_filtered - p_prev
                delta_v = vpv_filtered - v_prev

                # Ignore extremely small voltage changes
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

                    v_ref = saturate(v_ref, VREF_MAX, VREF_MIN)

                p_prev = ppv_filtered
                v_prev = vpv_filtered

    # --------------------------------------------------------
    # Logging
    # --------------------------------------------------------

    if SAVE_RESULTS and utime.ticks_diff(now_ms, last_log_ms) >= LOG_PERIOD_MS:
        last_log_ms = now_ms

        results.write(
            "{},{:.3f},{:.3f},{:.3f},{:.3f},{:.5f},{:.5f},{:.5f},"
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

    if SAVE_RESULTS and utime.ticks_diff(now_ms, last_flush_ms) >= LOG_FLUSH_PERIOD_MS:
        last_flush_ms = now_ms
        results.flush()

    # --------------------------------------------------------
    # MQTT telemetry
    # --------------------------------------------------------

    if utime.ticks_diff(now_ms, last_mqtt_ms) >= MQTT_PUBLISH_PERIOD_MS:
        last_mqtt_ms = now_ms
        mqtt_publish_power(ppv)

    # --------------------------------------------------------
    # Printing
    # --------------------------------------------------------

    if utime.ticks_diff(now_ms, last_print_ms) >= PRINT_PERIOD_MS:
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
                wifi_status,
                mqtt_status
            )
        )
