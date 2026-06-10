# Supercapacitor Current Controller
# Hardware: 0.5 F cap, Vbus = 10.0 V
# WARNING: this is a test version with opened STORE PWM ceiling. Use PSU current limit.
# Commands: S[J]=store  E[J]=extract  U=maintain  H=stop  P=print  C=40s CSV log
# Version: v36 (softer STORE braking + EXTRACT braking uses E_plot + 5-step EXTRACT braking)
# Main changes:
# 1) Added Constant-Current / Constant-Voltage (CC-CV) dynamic tapering to the main loop.
# 2) Tapering ONLY kicks in when the ESR spike threatens the terminal voltage limits.
# 3) Hard 5-second timeout (software fuse) added to STORE and EXTRACT completion check.
# 4) STORE exits immediately if Vcap reaches/exceeds VCAP_WORK_MAX; STORE taper can go to 0A.
# 5) STORE completion returns to MAINTAIN instead of PWM=0/STOPPED, so next STORE can warm start.
# 6) Polynomial ADC non-linearity correction added to read_vcap(). Replace placeholder
#    coefficients (ADC_CORR_A, ADC_CORR_B, ADC_CORR_C) with values from bench calibration.

from machine import Pin, I2C, ADC, PWM, Timer
import time
import sys

try:
    import uselect as select
except:
    import select


# ============================================================
# Hardware
# ============================================================

va_pin = ADC(Pin(28))

ina_i2c = I2C(
    0,
    scl=Pin(1),
    sda=Pin(0),
    freq=2400000
)

pwm = PWM(Pin(9))
pwm.freq(100000)

MIN_PWM = 1000
MAX_PWM = 64536

duty = 0
duty_cmd = 0.0
last_pwm_applied = 0
pwm.duty_u16(0)

led = Pin("LED", Pin.OUT)
led.on()


# ============================================================
# Parameters
# ============================================================

V_BUS      = 10
C          = 0.5
SHUNT_OHMS = 0.10
CAP_ESR_OHMS = 2.0           # Two 4 ohm capacitors in parallel -> 2 ohm total ESR
dt         = 1 / 1000.0

VCAP_MIN = 10.0

# Normal charging stops here. This leaves voltage headroom for switching / discharge overshoot.
VCAP_WORK_MAX = 15.7

# Absolute safety limit. If this is reached, the controller trips.
VCAP_HARD_MAX = 16.6

# Keep VCAP_MAX for old calculations / printouts. It now means hard max.
VCAP_MAX = VCAP_HARD_MAX

# STORE is allowed slightly below the normal operating minimum during testing,
# but a reading around 4-5 V is treated as a wiring/sensing fault for this setup.
VCAP_STORE_MIN = 8.5

# If current flows while PWM is OFF / STOPPED, software is not controlling
# the energy path. Your latest log showed IL around 0.7-1.9 A while
# duty=0 and pwm_applied=0, so STORE must be blocked in that condition.
STOPPED_CURRENT_WARN = 0.15
STOPPED_CURRENT_BLOCK_STORE = 0.20
STOPPED_CURRENT_DANGER = 0.80
STOPPED_WARNING_INTERVAL_MS = 1000
STATUS_PRINT_INTERVAL_MS = 1000

E_MIN = 0.5 * C * VCAP_MIN ** 2
E_WORK_MAX = 0.5 * C * VCAP_WORK_MAX ** 2
E_MAX = 0.5 * C * VCAP_HARD_MAX ** 2

E_STEP = 15.0                 # Default energy when S/E has no number, J
STORE_TIME_MS = 5000         # Store target time: 5 seconds


# ============================================================
# ADC Non-Linearity Correction Coefficients
# ============================================================
# read_vcap() applies a cubic correction that activates above 10.0 V:
#
#   z = raw_v - 10.0
#   error_correction = ADC_CUBIC * z^3 + ADC_QUAD * z^2
#   corrected_v = raw_v + error_correction   (if raw_v > 10.0, else raw_v)
#
# Fitted from bench calibration data (zero-current, multimeter on terminals):
#   Pico 12.00 V -> True 12.08 V  (+0.08 V)
#   Pico 14.20 V -> True 14.62 V  (+0.42 V)
#   Pico 14.85 V -> True 15.57 V  (+0.72 V)
# The error accelerates at the top end; a flat multiplier would ruin the 10 V range.
# Hinging at 10 V keeps low-voltage readings untouched.
#
ADC_CUBIC = 0.0037     # Cubic term  (fitted from calibration data)
ADC_QUAD  = 0.0125     # Quadratic term (fitted from calibration data)


# ============================================================
# CSV Logging Settings
# ============================================================

# Send command C to save a 40-second CSV file directly on the Pico.
# The file is overwritten each time you start a new log.
CSV_FILENAME = "log.csv"
CSV_LOG_TIME_MS = 80000      # 40 seconds
CSV_LOG_INTERVAL_MS = 50     # one row every 50 ms = about 800 rows

# Extra plotting energy. Control still uses E from ESR-corrected Vcap.
# E_terminal is smoother because it does not include IL * ESR correction.
# E_plot is a low-pass filtered copy of E, only for cleaner graphs.
E_PLOT_ALPHA = 0.03


# ============================================================
# Current Settings
# ============================================================

# For the default +15 J in 5 s, required power is 3 W. Around 10.5-16 V this
# means roughly 0.19-0.29 A ideal capacitor current, but practical SMPS loss
# and control delay mean we use a higher current target. Watch board temperature.
I_STORE    =  0.60          # Faster but safer store current target, A
I_EXTRACT  = -0.60          # Safer extract current target, A
I_MAINTAIN =  0.005           # Smaller maintain current target, A
I_LIMIT    =  1.20          # Hard overcurrent trip limit, A


# ============================================================
# STORE Controller Tuning
# ============================================================

# SAFE STORE STARTUP
# Important v12 change:
# With V_BUS = 9 V and Vcap already around 10.25 V, a cold STORE start
# from PWM=1000 entered a low-duty reverse-current region and producedSTORE_PWM_HARD_MAX_HIGH 
# store reverse current around -0.861 A.
# Therefore cold-start PWM is now selected from Vcap - V_BUS.
STORE_SAFE_START_PWM = 1000
STORE_COLD_MID_PWM = 3000
STORE_COLD_HIGH_PWM = 5600
STORE_COLD_MID_DELTA = 0.30
STORE_COLD_HIGH_DELTA = 0.80

SAFE_HOLD_PWM_POS = 5600
SAFE_HOLD_PWM_NEG = 8500      # Vcap >= V_BUS + 0.80 V -> high start

# OPENED STORE PWM CEILING TEST VERSION
# The previous 12650 cap can still limit STORE around 10.5-12 V.
# This version keeps the low-voltage ceiling conservative, but opens
# the high-voltage STORE ceiling to 15000 once Vcap >= 10.4 V.
# Current protection is NOT disabled.
STORE_PWM_HARD_MAX_BASE = 12520
STORE_PWM_HARD_MAX_HIGH = 30000
STORE_HIGH_VCAP = 10.4

# Incremental proportional controller for STORE mode.
# This version ramps upward faster to reach the 15  J target sooner.
# Downward motion is still faster for safety.
STORE_PWM_GAIN = 200.0
STORE_PWM_MAX_STEP_UP = 25.0       # Base ramp-up below 12.5 V
STORE_PWM_MAX_STEP_UP_MID = 35.0   # Faster ramp-up from 12.5 V
STORE_PWM_MAX_STEP_UP_HIGH = 45.0  # Fastest ramp-up from 13.8 V
STORE_PWM_STEP_MID_VCAP = 12.5
STORE_PWM_STEP_HIGH_VCAP = 13.8
STORE_PWM_MAX_STEP_DOWN = 320.0

# MAINTAIN is deliberately protected.
# Latest log: after STORE finished at about Vcap=12.4 V, MAINTAIN
# reduced duty to about 5200 and suddenly fell into a large negative
# current region. Therefore MAINTAIN is not allowed to go below 5600.
MAINTAIN_PWM_MIN = 5600
MAINTAIN_PWM_GAIN = 45.0
MAINTAIN_PWM_MAX_STEP_UP = 4.0
MAINTAIN_PWM_MAX_STEP_DOWN = 35.0
MAINTAIN_ENTRY_PWM_MARGIN = 1000

# If MAINTAIN still detects negative current, jump back to a known safer
# holding PWM instead of waiting for the hard overcurrent trip.
MAINTAIN_REVERSE_CURRENT_LIMIT = -0.20
MAINTAIN_RECOVERY_PWM = 14500
MAINTAIN_RECOVERY_PWM_MAX = 24000

# If a large current limit event is detected, do not immediately PWM=0.
# In a bidirectional synchronous SMPS, PWM=0 may still leave a switch path active.
# SAFE_HOLD keeps PWM active near a safer tested region and lets the current loop
# bring current back toward a small positive value.
SAFE_HOLD_PWM = 14500
I_SAFE_HOLD = 0.04

# EXTRACT finishing control.
# The latest log showed that after EXTRACT finished, handing over at too low
# a PWM could keep a large negative current flowing and over-discharge the cap.
# Therefore EXTRACT slows down near the energy target and jumps to a safer
# hold PWM before entering MAINTAIN.
EXTRACT_EXIT_HOLD_PWM = 14500       # Upper target only; do not jump to it directly after EXTRACT
EXTRACT_EXIT_PWM_STEP = 300         # First handover step from final EXTRACT PWM
MAINTAIN_SOFT_RAMP_MS = 1400        # Time to ramp MAINTAIN minimum upward after EXTRACT
EXTRACT_BRAKE_1_J = 4.00
EXTRACT_BRAKE_2_J = 2.50
EXTRACT_BRAKE_3_J = 1.50
EXTRACT_BRAKE_4_J = 0.80
EXTRACT_BRAKE_5_J = 0.35

I_EXTRACT_BRAKE_1 = -0.45
I_EXTRACT_BRAKE_2 = -0.30
I_EXTRACT_BRAKE_3 = -0.18
I_EXTRACT_BRAKE_4 = -0.09
I_EXTRACT_BRAKE_5 = -0.035
EXTRACT_DONE_TOL_J = 0.25

# EXTRACT voltage floor hold.
# When EXTRACT reaches VCAP_MIN, this is no longer treated as a fault.
# The controller enters V_HOLD mode and tries to keep the capacitor around
# VCAP_MIN instead of tripping.
V_HOLD_REF = VCAP_MIN
V_HOLD_BAND = 0.04
I_VHOLD_CHARGE = 0.08
I_VHOLD_FLOAT = 0.02
I_VHOLD_DISCHARGE = -0.02

# Low-pass filtering for measured current in STORE / MAINTAIN / EXTRACT modes.
IL_FILTER_ALPHA = 0.20
IL_filtered = 0.0

# EXTRACT target current is reduced slightly. These also restrict how fast PWM
# is allowed to move, because the ending transition can create an overcurrent spike.
EXTRACT_PWM_GAIN = 150.0
EXTRACT_PWM_MAX_STEP = 22.0
EXTRACT_MIN_PWM = 0

# If STORE current turns negative, the converter has crossed into an unsafe
# reverse-current region. Since this version opens the high PWM ceiling,
# keep the reverse-current threshold tight.
STORE_REVERSE_CURRENT_LIMIT = -0.10

# H command uses a short soft ramp-down instead of immediately forcing PWM=0.
SOFT_STOP_STEP = 300
SOFT_STOP_DELAY_MS = 1


# ============================================================
# PID Current Controller
# ============================================================

# These gains output a PWM step every 1 ms control tick.
# KD is left at 0 first because INA219 current noise can make D term unstable.
# So this runs as a PI current controller, but the PID structure is present.

KP_STORE = 200.0
KI_STORE = 25.0
KD_STORE = 0.0

KP_MAINTAIN = 45.0
KI_MAINTAIN = 6.0
KD_MAINTAIN = 0.0

KP_EXTRACT = 150.0
KI_EXTRACT = 18.0
KD_EXTRACT = 0.0

int_err = 0.0
prev_err = 0.0

INT_MIN = -0.8
INT_MAX = 0.8


# ============================================================
# State Variables
# ============================================================

I_target = 0.0

E_initial = 0.0
E_delta = 0.0
E_target_action = 0.0

action_in_progress = False
action_start_ms = 0
store_deadline_reported = False
extract_return_pwm = 0
maintain_soft_active = False
maintain_soft_start_pwm = 0
maintain_soft_target_pwm = 0
maintain_soft_start_ms = 0

mode = "STOPPED"
command = ""
command_energy_j = E_STEP

hard_stopped = True

trip = False
trip_reason = ""

r_elapsed = 0
count = 0
last_stopped_warning_ms = 0
last_status_print_ms = 0

E_plot = 0.0


# ============================================================
# Snapshot for P Command
# ============================================================

last_va = 0.0
last_va_terminal = 0.0
last_IL = 0.0
last_E = 0.0
last_E_terminal = 0.0
last_E_plot = 0.0
last_SoC = 0.0


# ============================================================
# CSV Logger State
# ============================================================

csv_logging = False
csv_file = None
csv_start_ms = 0
csv_last_ms = 0
csv_rows = 0


# ============================================================
# Serial Input
# ============================================================

poll = select.poll()
poll.register(sys.stdin, select.POLLIN)


# ============================================================
# INA219
# ============================================================

class INA219:
    REG_SHUNT = 0x01

    def __init__(self, shunt, addr):
        self.shunt = shunt
        self.addr = addr

    def vshunt(self):
        raw = ina_i2c.readfrom_mem(self.addr, self.REG_SHUNT, 2)

        value = int.from_bytes(raw, "big")

        if value >= 2 ** 15:
            value -= 2 ** 16

        return float(value) * 1e-5

    def configure(self):
        ina_i2c.writeto_mem(self.addr, 0x00, b"\x19\x9F")
        ina_i2c.writeto_mem(self.addr, 0x05, b"\x00\x00")


# ============================================================
# Helper Functions
# ============================================================

def tick(t):
    global timer_elapsed
    timer_elapsed = 1


def clamp(value, lower, upper):
    return max(min(value, upper), lower)


def read_vcap():
    raw_v = 1.017 * (12490 / 2490) * 3.3 * (va_pin.read_u16() / 65536)

    # ADC non-linearity correction (fitted from bench calibration data).
    # The error accelerates above 10 V, so the correction hinges there:
    #   error = ADC_CUBIC * z^3 + ADC_QUAD * z^2,  where z = raw_v - 10.0
    # Below 10 V the raw reading is returned unchanged.
    if raw_v > 10.0:
        z = raw_v - 10.0
        return raw_v + ADC_CUBIC * z * z * z + ADC_QUAD * z * z

    return raw_v


def correct_vcap_for_esr(v_terminal, cap_current):
    # IL is positive while charging and negative while extracting.
    return v_terminal - cap_current * CAP_ESR_OHMS


def reset_pid():
    global int_err, prev_err

    int_err = 0.0
    prev_err = 0.0


def pid_current_step(target_current, measured_current, kp_use, ki_use, kd_use, step_limit):
    global int_err, prev_err

    err = target_current - measured_current

    int_err += err * dt
    int_err = clamp(int_err, INT_MIN, INT_MAX)

    d_err = (err - prev_err) / dt
    prev_err = err

    output = kp_use * err + ki_use * int_err + kd_use * d_err
    output = clamp(output, -step_limit, step_limit)

    return output


def calculate_store_start_pwm(vcap):
    """
    Safe cold-start PWM for STORE mode.
    After STORE now stops with PWM=0, the next S command is a cold start.
    Your log showed Vcap around 12.1-12.3 V starting at PWM=5600 caused
    STORE reverse current around -0.9 A to -1.0 A.
    So high Vcap cold starts must begin at a higher PWM region.
    """

    if vcap >= 14.2:
        return 24500

    if vcap >= 13.5:
        return 22000

    if vcap >= 12.5:
        return 19500

    if vcap >= 11.5:
        return 17500

    if vcap >= 10.8:
        return 13500

    if vcap >= 10.3:
        return 8500

    return STORE_SAFE_START_PWM


def get_store_pwm_hard_max(vcap):
    if vcap >= STORE_HIGH_VCAP:
        return STORE_PWM_HARD_MAX_HIGH

    return STORE_PWM_HARD_MAX_BASE


def get_store_pwm_step_up(vcap):
    if vcap >= STORE_PWM_STEP_HIGH_VCAP:
        return STORE_PWM_MAX_STEP_UP_HIGH

    if vcap >= STORE_PWM_STEP_MID_VCAP:
        return STORE_PWM_MAX_STEP_UP_MID

    return STORE_PWM_MAX_STEP_UP


def calculate_store_warm_start_pwm(vcap):
    if (not hard_stopped) and last_pwm_applied > 0:
        return int(clamp(
            last_pwm_applied,
            MIN_PWM,
            get_store_pwm_hard_max(vcap)
        ))

    return int(clamp(
        calculate_store_start_pwm(vcap),
        MIN_PWM,
        get_store_pwm_hard_max(vcap)
    ))


def calculate_extract_start_pwm(vcap):
    if not hard_stopped and last_pwm_applied > 0:
        return int(clamp(
            last_pwm_applied,
            MIN_PWM,
            MAX_PWM
        ))

    return int(clamp(
        calculate_store_start_pwm(vcap),
        MIN_PWM,
        MAX_PWM
    ))


def pwm_off():
    global duty, duty_cmd, last_pwm_applied, hard_stopped

    duty = 0
    duty_cmd = 0.0
    last_pwm_applied = 0

    pwm.duty_u16(0)

    hard_stopped = True


def pwm_soft_off():
    global duty, duty_cmd, last_pwm_applied, hard_stopped

    temp = int(last_pwm_applied)

    while temp > 0:
        temp -= SOFT_STOP_STEP
        if temp < 0:
            temp = 0

        pwm.duty_u16(temp)
        time.sleep_ms(SOFT_STOP_DELAY_MS)

    duty = 0
    duty_cmd = 0.0
    last_pwm_applied = 0
    hard_stopped = True


def pwm_start(initial_pwm):
    global duty, duty_cmd, last_pwm_applied, hard_stopped

    duty = int(clamp(initial_pwm, MIN_PWM, MAX_PWM))
    duty_cmd = float(duty)

    last_pwm_applied = duty
    pwm.duty_u16(last_pwm_applied)

    hard_stopped = False

    reset_pid()


def enter_safe_hold(reason):
    global trip, trip_reason, command
    global action_in_progress, mode, I_target
    global duty, duty_cmd, last_pwm_applied, hard_stopped, IL_filtered

    recovery_pwm = calculate_safe_recovery_pwm(last_va)

    if not trip:
        print("TRIP:", reason)
        print(
            "Overcurrent safe hold: PWM set to {} instead of PWM=0. "
            "Send H only after current is near zero or after checking wiring.".format(
                recovery_pwm
            )
        )

    trip = True
    trip_reason = reason
    command = ""

    action_in_progress = False
    I_target = I_SAFE_HOLD
    mode = "SAFE_HOLD"

    duty = int(recovery_pwm)
    duty_cmd = float(duty)
    last_pwm_applied = duty
    pwm.duty_u16(last_pwm_applied)
    hard_stopped = False

    IL_filtered = last_IL
    reset_pid()


def do_trip(reason, reverse_hold=False):
    global trip, trip_reason, command
    global action_in_progress, mode, I_target

    if reverse_hold:
        enter_safe_hold(reason)
        return

    if not trip:
        print("TRIP:", reason)

    trip = True
    trip_reason = reason
    command = ""

    action_in_progress = False
    I_target = 0.0
    mode = "TRIPPED"

    pwm_off()


def do_stop():
    global trip, trip_reason, command
    global action_in_progress, mode, I_target
    global E_delta, E_target_action
    global store_deadline_reported

    trip = False
    trip_reason = ""

    pwm_soft_off()

    mode = "STOPPED"
    I_target = 0.0

    action_in_progress = False
    E_delta = 0.0
    E_target_action = 0.0
    store_deadline_reported = False

    reset_pid()

    command = ""

    print("Stopped. Send S/E/U to resume.")


def stop_store_pwm_off():
    global command, mode, I_target, action_in_progress
    global E_delta, E_target_action, store_deadline_reported

    pwm_soft_off()

    mode = "STOPPED"
    I_target = 0.0
    action_in_progress = False
    E_delta = 0.0
    E_target_action = 0.0
    store_deadline_reported = False

    reset_pid()
    command = ""


def do_maintain(va):
    global command, mode, I_target, action_in_progress
    global E_delta, E_target_action, IL_filtered
    global duty, duty_cmd, last_pwm_applied

    if hard_stopped:
        pwm_start(MAINTAIN_PWM_MIN)
    else:
        # When STORE/EXTRACT hands over, do not keep a very high PWM.
        # Keep the first MAINTAIN PWM close to the calculated safe minimum,
        # so the current can settle instead of continuing to charge hard.
        maintain_entry_max = calculate_maintain_min_pwm(va) + MAINTAIN_ENTRY_PWM_MARGIN

        if last_pwm_applied > maintain_entry_max:
            duty = int(clamp(maintain_entry_max, MIN_PWM, MAX_PWM))
            duty_cmd = float(duty)
            last_pwm_applied = duty
            pwm.duty_u16(last_pwm_applied)

    IL_filtered = last_IL

    command = ""
    mode = "MAINTAIN"

    if va >= VCAP_MAX:
        I_target = 0.0
    else:
        I_target = I_MAINTAIN

    action_in_progress = False
    E_delta = 0.0
    E_target_action = 0.0

    reset_pid()


def calculate_vhold_current(va):
    if va < V_HOLD_REF - V_HOLD_BAND:
        return I_VHOLD_CHARGE

    if va > V_HOLD_REF + V_HOLD_BAND:
        return I_VHOLD_DISCHARGE

    return I_VHOLD_FLOAT


def calculate_safe_recovery_pwm(va):
    if va >= 14.0:
        return 23000

    if va >= 12.5:
        value = 14500 + (va - 12.5) * 4000
        return int(clamp(value, 14500, 23000))

    if va >= 10.5:
        value = 9000 + (va - 10.5) * 2750
        return int(clamp(value, 9000, 14500))

    return 8500


def calculate_maintain_min_pwm(va):
    safe_pwm = calculate_safe_recovery_pwm(va) - 2500
    return int(clamp(safe_pwm, MAINTAIN_PWM_MIN, 23000))


def calculate_active_maintain_min_pwm(va):
    global maintain_soft_active

    normal_min = calculate_maintain_min_pwm(va)

    if not maintain_soft_active:
        return normal_min

    elapsed_ms = time.ticks_diff(time.ticks_ms(), maintain_soft_start_ms)

    if elapsed_ms >= MAINTAIN_SOFT_RAMP_MS:
        maintain_soft_active = False
        return normal_min

    if maintain_soft_target_pwm <= maintain_soft_start_pwm:
        return normal_min

    ramp_pwm = maintain_soft_start_pwm + (
        (maintain_soft_target_pwm - maintain_soft_start_pwm)
        * elapsed_ms
        / MAINTAIN_SOFT_RAMP_MS
    )

    if ramp_pwm > normal_min:
        ramp_pwm = normal_min

    return int(clamp(ramp_pwm, MAINTAIN_PWM_MIN, normal_min))


def calculate_extract_target_current(E_now, current_va):
    """
    EXTRACT target current with two braking limits:
    1) voltage-floor taper, to avoid pulling below VCAP_MIN;
    2) time-based braking, to avoid waiting for ESR-corrected energy to catch up.

    The old energy-only braking could start too late because during EXTRACT,
    negative IL makes ESR-corrected E jump upward first. Time braking makes
    the current step down even when E/E_plot is temporarily misleading.
    """
    terminal_floor_limit = VCAP_MIN + 0.2
    dynamic_I_limit = -(current_va - terminal_floor_limit) / CAP_ESR_OHMS

    base_taper_target = -clamp(
        -dynamic_I_limit,
        0.035,
        abs(I_EXTRACT)
    )

    # ----- Energy-based braking, still useful near the final target -----
    target_E = E_initial - E_target_action
    remaining_J = E_now - target_E

    energy_limit = I_EXTRACT

    if remaining_J <= EXTRACT_BRAKE_5_J:
        energy_limit = I_EXTRACT_BRAKE_5
    elif remaining_J <= EXTRACT_BRAKE_4_J:
        energy_limit = I_EXTRACT_BRAKE_4
    elif remaining_J <= EXTRACT_BRAKE_3_J:
        energy_limit = I_EXTRACT_BRAKE_3
    elif remaining_J <= EXTRACT_BRAKE_2_J:
        energy_limit = I_EXTRACT_BRAKE_2
    elif remaining_J <= EXTRACT_BRAKE_1_J:
        energy_limit = I_EXTRACT_BRAKE_1

    # ----- Time-based braking -----
    # v39: delay the braking slightly compared with v38.
    # v38 reduced to -0.035 A too early, so E10 often timed out before
    # reaching the full 10 J. This keeps the ending smooth but gives
    # EXTRACT more time at useful current.
    elapsed_ms = time.ticks_diff(time.ticks_ms(), action_start_ms)

    scale = E_target_action / 10.0
    scale = clamp(scale, 0.65, 1.30)

    # For E10, this is approximately:
    # 0.0-1.1s  -> -0.60 A
    # 1.1-1.7s  -> -0.45 A
    # 1.7-2.3s  -> -0.30 A
    # 2.3-3.0s  -> -0.18 A
    # 3.0-3.7s  -> -0.09 A
    # after 3.7s -> -0.035 A
    t1 = int(1100 * scale)
    t2 = int(1700 * scale)
    t3 = int(2300 * scale)
    t4 = int(3000 * scale)
    t5 = int(3700 * scale)

    time_limit = I_EXTRACT

    if elapsed_ms >= t5:
        time_limit = I_EXTRACT_BRAKE_5
    elif elapsed_ms >= t4:
        time_limit = I_EXTRACT_BRAKE_4
    elif elapsed_ms >= t3:
        time_limit = I_EXTRACT_BRAKE_3
    elif elapsed_ms >= t2:
        time_limit = I_EXTRACT_BRAKE_2
    elif elapsed_ms >= t1:
        time_limit = I_EXTRACT_BRAKE_1

    # For negative currents, max() chooses the safer / smaller magnitude value.
    return max(base_taper_target, energy_limit, time_limit)

def enter_maintain_after_extract(va):
    global duty, duty_cmd, last_pwm_applied, extract_return_pwm
    global maintain_soft_active, maintain_soft_start_pwm
    global maintain_soft_target_pwm, maintain_soft_start_ms

    final_extract_pwm = int(last_pwm_applied)
    normal_min = calculate_maintain_min_pwm(va)

    target_pwm = final_extract_pwm + EXTRACT_EXIT_PWM_STEP

    if target_pwm > normal_min:
        target_pwm = normal_min

    if target_pwm < MIN_PWM:
        target_pwm = MIN_PWM

    duty = int(clamp(target_pwm, MIN_PWM, MAX_PWM))
    duty_cmd = float(duty)
    last_pwm_applied = duty
    pwm.duty_u16(last_pwm_applied)

    maintain_soft_active = True
    maintain_soft_start_pwm = duty
    maintain_soft_target_pwm = normal_min
    maintain_soft_start_ms = time.ticks_ms()

    print(
        "EXTRACT handover: final_pwm={} first_hold_pwm={} "
        "normal_min={} soft_ramp_ms={}".format(
            final_extract_pwm,
            duty,
            normal_min,
            MAINTAIN_SOFT_RAMP_MS
        )
    )

    do_maintain(va)


def do_voltage_hold_floor(va):
    global command, mode, I_target, action_in_progress
    global E_delta, E_target_action, IL_filtered

    if hard_stopped:
        pwm_start(MAINTAIN_PWM_MIN)

    IL_filtered = last_IL

    command = ""
    mode = "V_HOLD"
    I_target = calculate_vhold_current(va)

    action_in_progress = False
    E_delta = 0.0
    E_target_action = 0.0

    reset_pid()


def recover_from_maintain_reverse_current(IL):
    global duty, duty_cmd, last_pwm_applied, IL_filtered

    # Pure relative bump: do not use calculate_safe_recovery_pwm() here, and
    # do not apply a hardcoded PWM floor.
    # During a reverse-current transient, IL is large and negative, so the
    # ESR correction inflates va well above the true capacitor voltage.
    # Passing that spiked va into calculate_safe_recovery_pwm() returns ~23000
    # at an actual physical voltage of ~12-13 V, causing a violent positive
    # overshoot (the MAINTAIN death spiral).
    # The old MAINTAIN_RECOVERY_PWM = 14500 floor had the same problem at low
    # Vcap: after an EXTRACT handover at ~4840 PWM, bumping to 14500 acted
    # like a full STORE command and caused a 1.3 A overcurrent spike.
    # The fix is still relative, but cap the recovery PWM so repeated
    # reverse-current detections cannot push MAINTAIN into a high STORE-like PWM.
    recovery_pwm = last_pwm_applied + 800

    if recovery_pwm > MAINTAIN_RECOVERY_PWM_MAX:
        recovery_pwm = MAINTAIN_RECOVERY_PWM_MAX

    print(
        "MAINTAIN reverse current {:.3f}A -> soft bumping PWM to {}".format(
            IL,
            recovery_pwm
        )
    )

    duty = int(clamp(recovery_pwm, MIN_PWM, MAX_PWM))
    duty_cmd = float(duty)
    last_pwm_applied = duty
    pwm.duty_u16(last_pwm_applied)

    IL_filtered = IL


def warn_if_current_while_stopped(now_ms, va, IL):
    global last_stopped_warning_ms

    if not hard_stopped:
        return

    if abs(IL) < STOPPED_CURRENT_WARN:
        return

    if time.ticks_diff(now_ms, last_stopped_warning_ms) < STOPPED_WARNING_INTERVAL_MS:
        return

    last_stopped_warning_ms = now_ms

    if abs(IL) >= STOPPED_CURRENT_DANGER:
        level = "DANGER"
    else:
        level = "WARNING"

    print(
        "{}: uncontrolled current while STOPPED: IL={:.3f}A, "
        "Vcap={:.3f}V, pwm=0. Check wiring / external PSU path. "
        "STORE will be blocked until |IL| < {:.3f}A.".format(
            level,
            IL,
            va,
            STOPPED_CURRENT_BLOCK_STORE
        )
    )


def start_csv_log():
    global csv_logging, csv_file, csv_start_ms, csv_last_ms, csv_rows

    if csv_file:
        try:
            csv_file.flush()
            csv_file.close()
        except:
            pass

    try:
        csv_file = open(CSV_FILENAME, "w")

        csv_file.write(
            "time_ms,elapsed_ms,mode,trip,hard_stopped,"
            "action_in_progress,Vterm,Vcap,IL,IL_filtered,"
            "E,E_terminal,E_plot,E_delta,SoC,I_target,duty,pwm_applied\n"
        )
        csv_file.flush()

        csv_logging = True
        csv_start_ms = time.ticks_ms()
        csv_last_ms = time.ticks_add(csv_start_ms, -CSV_LOG_INTERVAL_MS)
        csv_rows = 0

        print(
            "CSV logging started: {} for {:.1f}s".format(
                CSV_FILENAME,
                CSV_LOG_TIME_MS / 1000.0
            )
        )

    except Exception as e:
        csv_logging = False
        csv_file = None
        print("CSV open error:", e)


def stop_csv_log():
    global csv_logging, csv_file

    if csv_file:
        try:
            csv_file.flush()
            csv_file.close()
        except Exception as e:
            print("CSV close error:", e)

    csv_file = None
    csv_logging = False

    print(
        "CSV logging finished: rows={} file={}".format(
            csv_rows,
            CSV_FILENAME
        )
    )


def update_csv_log(now_ms, va_terminal, va, IL, E, E_terminal, E_plot_value, SoC):
    global csv_last_ms, csv_rows

    if not csv_logging:
        return

    elapsed_ms = time.ticks_diff(now_ms, csv_start_ms)

    if elapsed_ms >= CSV_LOG_TIME_MS:
        stop_csv_log()
        return

    if time.ticks_diff(now_ms, csv_last_ms) < CSV_LOG_INTERVAL_MS:
        return

    csv_last_ms = now_ms

    try:
        csv_file.write(
            "{},{},{},{},{},{},{:.4f},{:.4f},{:.4f},{:.4f},"
            "{:.4f},{:.4f},{:.4f},{:.4f},{:.2f},{:.4f},{},{}\n".format(
                now_ms,
                elapsed_ms,
                mode,
                int(trip),
                int(hard_stopped),
                int(action_in_progress),
                va_terminal,
                va,
                IL,
                IL_filtered,
                E,
                E_terminal,
                E_plot_value,
                E_delta,
                SoC,
                I_target,
                duty,
                last_pwm_applied
            )
        )

        csv_rows += 1

        if csv_rows % int(1000 / CSV_LOG_INTERVAL_MS) == 0:
            csv_file.flush()

    except Exception as e:
        print("CSV write error:", e)
        stop_csv_log()


def parse_energy_amount(text):
    text = text.strip()

    if not text:
        return E_STEP

    if text[0] in "=:":
        text = text[1:].strip()

    if text.endswith("J"):
        text = text[:-1].strip()

    try:
        value = float(text)
    except:
        return None

    if value <= 0.0:
        return None

    return value


def read_cmd():
    global command, command_energy_j

    try:
        if poll.poll(0):
            line = sys.stdin.readline().strip().upper()

            if not line:
                return

            cmd = line[0]

            if cmd in "SE":
                amount = parse_energy_amount(line[1:])

                if amount is None:
                    print("Invalid energy command. Use S10, S10J, E5, or E5.5J.")
                    return

                command_energy_j = amount
                command = cmd

            elif cmd in "UHPC":
                command = cmd

    except:
        pass


def print_status():
    if action_in_progress:
        elapsed_s = time.ticks_diff(time.ticks_ms(), action_start_ms) / 1000.0
    else:
        elapsed_s = 0.0

    energy_space = clamp(E_MAX - last_E, 0.0, E_MAX - E_MIN)
    energy_available = clamp(last_E - E_MIN, 0.0, E_MAX - E_MIN)

    print(
        "mode={} trip={} Vterm={:.3f}V Vcap={:.3f}V IL={:.3f}A "
        "E={:.3f}J Eterm={:.3f}J Eplot={:.3f}J Espace={:.3f}J Eout={:.3f}J "
        "dE={:.3f}J SoC={:.1f}% target={:.3f}A "
        "duty={} pwm_applied={} t={:.3f}s".format(
            mode,
            trip,
            last_va_terminal,
            last_va,
            last_IL,
            last_E,
            last_E_terminal,
            last_E_plot,
            energy_space,
            energy_available,
            E_delta,
            last_SoC,
            I_target,
            duty,
            last_pwm_applied,
            elapsed_s
        )
    )

    if trip:
        print("trip_reason:", trip_reason)


# ============================================================
# Initialisation
# ============================================================

timer_elapsed = 0

ina = INA219(SHUNT_OHMS, 64)
ina.configure()

loop_r = Timer(
    mode=Timer.PERIODIC,
    freq=1000,
    callback=tick
)

print("Ready. v40 STORE soft brake + delayed EXTRACT braking + capped MAINTAIN recovery. Commands: S[J] E[J] U H P C")
print("Examples: S8 charges 8J, E3.5 extracts 3.5J. S/E alone use default.")
print("PWM initially OFF.")
print("ESR correction enabled: Vcap = Vterm - IL * {:.3f} ohm".format(CAP_ESR_OHMS))
print(
    "ADC correction: cubic={} quad={} hinge=10.0V (fitted from bench calibration)".format(
        ADC_CUBIC, ADC_QUAD
    )
)
print(
    "I_STORE={:.3f}A I_EXTRACT={:.3f}A "
    "I_MAINTAIN={:.3f}A I_LIMIT={:.3f}A".format(
        I_STORE,
        I_EXTRACT,
        I_MAINTAIN,
        I_LIMIT
    )
)
print(
    "Default STORE/EXTRACT target: {:.3f}J; usable energy range: {:.3f}-{:.3f}J "
    "({:.3f}-{:.3f}V work, hard max {:.3f}V)".format(
        E_STEP,
        E_MIN,
        E_WORK_MAX,
        VCAP_MIN,
        VCAP_WORK_MAX,
        VCAP_HARD_MAX
    )
)
print(
    "OPEN LIMIT TEST: safe cold start uses Vcap table: 10.3V->8500, 10.8V->13500, 11.5V->17500, 12.5V->19500, 13.5V->22000, 14.2V->24500; hard_max_base={} hard_max_high={} high_from={:.2f}V; ramp_up_high={} count/ms; overcurrent->SAFE_HOLD; MAINTAIN base min PWM={}; stopped-current block={:.3f}A; EXTRACT floor={:.2f}V -> V_HOLD".format(
        STORE_PWM_HARD_MAX_BASE,
        STORE_PWM_HARD_MAX_HIGH,
        STORE_HIGH_VCAP,
        STORE_PWM_MAX_STEP_UP_HIGH,
        MAINTAIN_PWM_MIN,
        STOPPED_CURRENT_BLOCK_STORE,
        VCAP_MIN
    )
)


# ============================================================
# Main Loop
# ============================================================

while True:

    if not timer_elapsed:
        continue

    timer_elapsed = 0


    # --------------------------------------------------------
    # Measure Voltage and Current
    # --------------------------------------------------------

    try:
        va_terminal = read_vcap()
        IL = -ina.vshunt() / SHUNT_OHMS
        va = correct_vcap_for_esr(va_terminal, IL)

    except Exception as e:
        do_trip("sensor error: " + str(e))
        continue


    E = 0.5 * C * va ** 2
    E_terminal = 0.5 * C * va_terminal ** 2

    if E_plot <= 0.0:
        E_plot = E
    else:
        E_plot = (1.0 - E_PLOT_ALPHA) * E_plot + E_PLOT_ALPHA * E

    SoC = clamp(E / E_MAX * 100.0, 0.0, 100.0)

    last_va = va
    last_va_terminal = va_terminal
    last_IL = IL
    last_E = E
    last_E_terminal = E_terminal
    last_E_plot = E_plot
    last_SoC = SoC

    now_ms = time.ticks_ms()
    update_csv_log(now_ms, va_terminal, va, IL, E, E_terminal, E_plot, SoC)

    warn_if_current_while_stopped(now_ms, va, IL)


    # --------------------------------------------------------
    # Safety Trips: execute immediately after measurement
    # --------------------------------------------------------

    if (
        not trip
        and not hard_stopped
        and mode == "STORE"
        and IL <= STORE_REVERSE_CURRENT_LIMIT
    ):
        do_trip("store reverse current {:.3f}A".format(IL), reverse_hold=True)
        continue

    if (
        not trip
        and not hard_stopped
        and (mode == "MAINTAIN" or mode == "V_HOLD")
        and IL <= MAINTAIN_REVERSE_CURRENT_LIMIT
    ):
        recover_from_maintain_reverse_current(IL)
        continue

    if not trip and not hard_stopped and abs(IL) >= I_LIMIT:
        do_trip("overcurrent {:.3f}A".format(IL), reverse_hold=True)
        continue

    if not trip and not hard_stopped and va >= VCAP_HARD_MAX:
        do_trip("hard overvoltage {:.3f}V".format(va))
        continue

    if (
        not trip
        and not hard_stopped
        and mode == "STORE"
        and va >= VCAP_WORK_MAX
    ):
        if action_in_progress:
            E_delta = E - E_initial
            elapsed_s = time.ticks_diff(
                time.ticks_ms(),
                action_start_ms
            ) / 1000.0
        else:
            elapsed_s = 0.0

        print(
            "STORE stopped: work max reached. deltaE={:.3f}J "
            "time={:.3f}s E={:.3f}J Vcap={:.3f}V IL={:.3f}A -> MAINTAIN".format(
                E_delta,
                elapsed_s,
                E,
                va,
                IL
            )
        )
        do_maintain(va)
        continue

    if not trip and mode == "EXTRACT" and va <= VCAP_MIN:
        if action_in_progress:
            E_delta = E - E_initial
            elapsed_s = time.ticks_diff(
                time.ticks_ms(),
                action_start_ms
            ) / 1000.0

            print(
                "EXTRACT reached {:.2f}V floor. deltaE={:.3f}J "
                "time={:.3f}s E={:.3f}J Vcap={:.3f}V -> V_HOLD".format(
                    VCAP_MIN,
                    E_delta,
                    elapsed_s,
                    E,
                    va
                )
            )
        else:
            print(
                "EXTRACT voltage floor reached: Vcap={:.3f}V -> V_HOLD".format(
                    va
                )
            )

        do_voltage_hold_floor(va)
        continue


    # --------------------------------------------------------
    # Read Serial Command
    # --------------------------------------------------------

    read_cmd()


    # --------------------------------------------------------
    # Immediate Commands
    # --------------------------------------------------------

    if command == "H":
        do_stop()

    elif command == "P":
        print_status()
        command = ""

    elif command == "C":
        start_csv_log()
        command = ""

    elif command == "U":

        if trip:
            print("Trip active - send H first.")
            command = ""
        else:
            do_maintain(va)


    # --------------------------------------------------------
    # Start STORE
    # --------------------------------------------------------

    if command == "S":

        command = ""

        if trip:
            if hard_stopped and abs(IL) <= STOPPED_CURRENT_BLOCK_STORE:
                print("Trip flag cleared automatically for STORE start.")
                trip = False
                trip_reason = ""
            else:
                print(
                    "STORE blocked: trip active or current is not safe. "
                    "IL={:.3f}A, limit={:.3f}A. Send H only after checking "
                    "the current is near zero.".format(
                        IL,
                        STOPPED_CURRENT_BLOCK_STORE
                    )
                )

        if trip:
            pass

        elif action_in_progress:
            print("Action in progress.")

        elif va < VCAP_STORE_MIN:
            print(
                "STORE blocked: Vcap={:.3f}V below {:.3f}V. "
                "Check capacitor voltage, ADC divider, and wiring.".format(
                    va,
                    VCAP_STORE_MIN
                )
            )

        elif hard_stopped and abs(IL) > STOPPED_CURRENT_BLOCK_STORE:
            print(
                "STORE blocked: current is already flowing while PWM is OFF. "
                "IL={:.3f}A, limit={:.3f}A. This is a hardware/power-path "
                "issue, not a current-loop issue. Lower the external PSU "
                "current limit or disconnect the uncontrolled path first.".format(
                    IL,
                    STOPPED_CURRENT_BLOCK_STORE
                )
            )

        elif E >= E_WORK_MAX - 0.05:
            print(
                "Already at work max energy ({:.3f}J, Vcap={:.3f}V).".format(
                    E,
                    va
                )
            )

        else:
            start_pwm = calculate_store_warm_start_pwm(va)

            if hard_stopped:
                pwm_start(start_pwm)
                store_start_type = "cold"
            else:
                duty = int(clamp(start_pwm, MIN_PWM, get_store_pwm_hard_max(va)))
                duty_cmd = float(duty)
                last_pwm_applied = duty
                pwm.duty_u16(last_pwm_applied)
                hard_stopped = False
                reset_pid()
                store_start_type = "warm"

            E_initial = E
            E_delta = 0.0
            E_target_action = min(command_energy_j, E_WORK_MAX - E)

            I_target = I_STORE
            action_in_progress = True
            mode = "STORE"

            action_start_ms = time.ticks_ms()
            store_deadline_reported = False

            IL_filtered = IL

            reset_pid()

            print(
                "STORE: requested +{:.3f}J, target +{:.3f}J within {:.3f}s "
                "from {:.3f}J start_pwm={} type={} Vcap={:.3f}V Vbus={:.3f}V dV={:.3f}V".format(
                    command_energy_j,
                    E_target_action,
                    STORE_TIME_MS / 1000.0,
                    E,
                    start_pwm,
                    store_start_type,
                    va,
                    V_BUS,
                    va - V_BUS
                )
            )


    # --------------------------------------------------------
    # Start EXTRACT
    # --------------------------------------------------------

    if command == "E" and not trip:

        command = ""

        if action_in_progress:
            print("Action in progress.")

        elif E <= E_MIN + 0.05:
            print(
                "Already at {:.2f}V lower floor ({:.3f}J). "
                "Entering V_HOLD instead of EXTRACT.".format(VCAP_MIN, E)
            )
            if not hard_stopped:
                do_voltage_hold_floor(va)

        elif hard_stopped:
            print(
                "EXTRACT blocked while PWM is OFF. "
                "Send U first, wait until MAINTAIN current is near "
                "{:.3f}A, then send E.".format(I_MAINTAIN)
            )

        else:
            start_pwm = calculate_extract_start_pwm(va)
            extract_return_pwm = start_pwm

            pwm_start(start_pwm)

            E_initial = E
            E_delta = 0.0
            E_target_action = min(command_energy_j, E - E_MIN)

            I_target = I_EXTRACT
            action_in_progress = True
            mode = "EXTRACT"

            action_start_ms = time.ticks_ms()

            IL_filtered = IL
            reset_pid()

            print(
                "EXTRACT: requested -{:.3f}J, target -{:.3f}J from {:.3f}J "
                "start_pwm={} (from MAINTAIN)".format(
                    command_energy_j,
                    E_target_action,
                    E,
                    start_pwm
                )
            )


    # --------------------------------------------------------
    # Check STORE / EXTRACT Completion
    # --------------------------------------------------------

    if not trip and action_in_progress:

        E_delta = E - E_initial
        elapsed_ms = time.ticks_diff(time.ticks_ms(), action_start_ms)
        elapsed_s = elapsed_ms / 1000.0

        # --- STORE COMPLETION & TIMEOUT ---
        if mode == "STORE":

            # 0. Work voltage ceiling reached
            if va >= VCAP_WORK_MAX:
                print(
                    "STORE stopped: work max reached. deltaE={:.3f}J "
                    "time={:.3f}s E={:.3f}J Vcap={:.3f}V IL={:.3f}A -> MAINTAIN".format(
                        E_delta,
                        elapsed_s,
                        E,
                        va,
                        IL
                    )
                )
                do_maintain(va)

            # 1. Target Reached
            elif E_delta >= E_target_action:
                print(
                    "STORE done. deltaE={:.3f}J time={:.3f}s "
                    "E={:.3f}J Vcap={:.3f}V -> MAINTAIN".format(
                        E_delta, elapsed_s, E, va
                    )
                )
                do_maintain(va)

            # 2. Hard 5-Second Timeout (Software Fuse)
            elif elapsed_ms >= STORE_TIME_MS:
                print(
                    "STORE TIMEOUT: 5.000s limit reached. "
                    "deltaE={:.3f}J Vcap={:.3f}V IL={:.3f}A -> MAINTAIN".format(
                        E_delta, va, IL
                    )
                )
                do_maintain(va)

        # --- EXTRACT COMPLETION & TIMEOUT ---
        elif mode == "EXTRACT":

            # 1. Target Reached
            if E_delta <= -(E_target_action - EXTRACT_DONE_TOL_J):
                print(
                    "EXTRACT done. deltaE={:.3f}J time={:.3f}s "
                    "E={:.3f}J Vcap={:.3f}V".format(
                        E_delta, elapsed_s, E, va
                    )
                )
                enter_maintain_after_extract(va)

            # 2. Hard 5-Second Timeout (Software Fuse)
            elif elapsed_ms >= STORE_TIME_MS:
                print(
                    "EXTRACT TIMEOUT: Aborting trade! 5.000s limit reached. "
                    "deltaE={:.3f}J Vcap={:.3f}V IL={:.3f}A".format(
                        E_delta, va, IL
                    )
                )
                enter_maintain_after_extract(va)
                print("Hardware protected -> Dropping to MAINTAIN.")


    # --------------------------------------------------------
    # Current Control
    # --------------------------------------------------------

    control_allowed = (not hard_stopped) and ((not trip) or mode == "SAFE_HOLD")

    if control_allowed:

        if mode == "STORE":

            # --- MATHEMATICAL CC-CV TAPERING ---
            # Taper around the work max, not the hard max.
            # This leaves headroom between VCAP_WORK_MAX and VCAP_HARD_MAX
            # for switching / discharge overshoot.
            terminal_ceiling = VCAP_WORK_MAX - 0.2
            dynamic_I = (terminal_ceiling - va) / CAP_ESR_OHMS

            if va >= VCAP_WORK_MAX:
                I_target = 0.0
            else:
                I_target = clamp(dynamic_I, 0.0, I_STORE)

            # STORE braking near the energy target.
            # The first part of STORE still uses I_STORE = 0.60 A,
            # but the last part reduces current before entering MAINTAIN.
            # This reduces the voltage / energy jump caused by ESR and current handover.
            energy_remaining = E_target_action - E_delta

            if action_in_progress:
                if energy_remaining < 4.0:
                    I_target = min(I_target, 0.35)

                if energy_remaining < 2.5:
                    I_target = min(I_target, 0.22)

                if energy_remaining < 1.5:
                    I_target = min(I_target, 0.12)

                if energy_remaining < 0.8:
                    I_target = min(I_target, 0.05)

                if energy_remaining < 0.35:
                    I_target = min(I_target, 0.02)
            # -----------------------------------

            IL_filtered = (
                (1.0 - IL_FILTER_ALPHA) * IL_filtered
                + IL_FILTER_ALPHA * IL
            )

            if I_target <= 0.0:
                reset_pid()
                pwm_step = -STORE_PWM_MAX_STEP_DOWN
            else:
                raw_pid_step = pid_current_step(
                    I_target,
                    IL_filtered,
                    KP_STORE,
                    KI_STORE,
                    KD_STORE,
                    STORE_PWM_MAX_STEP_DOWN
                )

                if raw_pid_step >= 0:
                    pwm_step = clamp(
                        raw_pid_step,
                        0.0,
                        get_store_pwm_step_up(va)
                    )
                else:
                    pwm_step = clamp(
                        raw_pid_step,
                        -STORE_PWM_MAX_STEP_DOWN,
                        0.0
                    )

            duty_cmd = clamp(
                duty_cmd + pwm_step,
                MIN_PWM,
                get_store_pwm_hard_max(va)
            )

            duty = int(duty_cmd)


        elif mode == "MAINTAIN" or mode == "SAFE_HOLD" or mode == "V_HOLD":

            if mode == "V_HOLD":
                I_target = calculate_vhold_current(va)

            IL_filtered = (
                (1.0 - IL_FILTER_ALPHA) * IL_filtered
                + IL_FILTER_ALPHA * IL
            )

            raw_pid_step = pid_current_step(
                I_target,
                IL_filtered,
                KP_MAINTAIN,
                KI_MAINTAIN,
                KD_MAINTAIN,
                MAINTAIN_PWM_MAX_STEP_DOWN
            )

            if raw_pid_step >= 0:
                pwm_step = clamp(
                    raw_pid_step,
                    0.0,
                    MAINTAIN_PWM_MAX_STEP_UP
                )
            else:
                pwm_step = clamp(
                    raw_pid_step,
                    -MAINTAIN_PWM_MAX_STEP_DOWN,
                    0.0
                )

            duty_cmd = clamp(
                duty_cmd + pwm_step,
                calculate_active_maintain_min_pwm(va),
                get_store_pwm_hard_max(va)
            )

            duty = int(duty_cmd)


        elif mode == "EXTRACT":

            IL_filtered = (
                (1.0 - IL_FILTER_ALPHA) * IL_filtered
                + IL_FILTER_ALPHA * IL
            )

            # Use E_plot plus time-based braking for EXTRACT.
            # E/E_plot can be misleading during the first discharge transient,
            # so calculate_extract_target_current() also steps current down by elapsed time.
            I_target = calculate_extract_target_current(E_plot, va)

            pwm_step = pid_current_step(
                I_target,
                IL_filtered,
                KP_EXTRACT,
                KI_EXTRACT,
                KD_EXTRACT,
                EXTRACT_PWM_MAX_STEP
            )

            duty_cmd = clamp(
                duty_cmd + pwm_step,
                EXTRACT_MIN_PWM,
                MAX_PWM
            )

            duty = int(duty_cmd)


        last_pwm_applied = duty
        pwm.duty_u16(last_pwm_applied)


    # --------------------------------------------------------
    # Periodic Status Output
    # --------------------------------------------------------

    if time.ticks_diff(now_ms, last_status_print_ms) >= STATUS_PRINT_INTERVAL_MS:
        last_status_print_ms = now_ms
        print_status()

    count += 1

