from machine import Pin, I2C, ADC, PWM
import utime

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
DUTY_MAX = 60000

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

SAVE_RESULTS = True
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
print("=======================================")

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
    # Printing
    # --------------------------------------------------------

    if utime.ticks_diff(now_ms, last_print_ms) >= PRINT_PERIOD_MS:
        last_print_ms = now_ms

        print(
            "t={}ms | state={} | Vpv={:.2f}V | Vfilt={:.2f}V | "
            "Ipv={:.3f}A | Ppv={:.2f}W | Vbus={:.2f}V | "
            "Vref={:.2f}V | duty={} | unsafe={} "
            "[vl={}, vh={}, ih={}, bh={}]".format(
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
                int(unsafe_vbus_high)
            )
        )