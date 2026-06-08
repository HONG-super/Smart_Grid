# Supercapacitor Current Controller
# Hardware: 0.5 F cap, Vbus = 10.0 V
# WARNING: this is a test version with opened STORE PWM ceiling. Use PSU current limit.
# Commands: S[J]=store  E[J]=extract  U=maintain  H=stop  P=print  C=40s CSV log
# Version: v27 B-port capacitor wiring, bidirectional PWM convention
# Main changes:
# 1) Raise high-voltage STORE ceiling to 15000 because 14000 still limited current near 12 V.
# 2) Use voltage-based SAFE_HOLD / recovery so EXTRACT handover does not keep discharging.
# 3) Allow S10 / E5.5 style selectable STORE/EXTRACT energy targets.
# 4) Allow direct S start from STOPPED/TRIPPED when current is already safe.
# 5) Reduce EXTRACT end overcurrent by braking earlier and switching back more smoothly.
# 6) v20: stronger final EXTRACT braking after the 1.207 A end spike test.
# 7) v21: EXTRACT -> MAINTAIN no longer jumps straight to high hold PWM.
# 8) v22: faster STORE PWM ramp above 12.5 V / 13.8 V to improve high-voltage charging speed.
# 9) v23: status auto-print reduced to 1 Hz to reduce serial load.
# 10) v24: correct measured terminal voltage for 2 ohm effective capacitor ESR.
# 11) v25: capacitor moved to B port, DC bus moved to A port.
# 12) v26: keep direct PWM command mapping; controller thresholds are tuned in direct PWM units.
# 13) v27: use provided bidirectional code convention: duty_actual = 65536 - pwm_out.

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

va_pin = ADC(Pin(28))    # A port: DC bus
vb_pin = ADC(Pin(26))    # B port: capacitor

ina_i2c = I2C(
    0,
    scl=Pin(1),
    sda=Pin(0),
    freq=2400000
)

pwm = PWM(Pin(9))
pwm.freq(100000)

MIN_PWM = 0
MAX_PWM = 64536

duty = 0
duty_cmd = 0.0
last_pwm_applied = 0
last_pwm_actual = 0
pwm.duty_u16(0)

led = Pin("LED", Pin.OUT)
led.on()


# ============================================================
# Parameters
# ============================================================

V_BUS      = 10
C          = 1.5
SHUNT_OHMS = 0.10
CAP_ESR_OHMS = 0.3           # Two 4 ohm capacitors in parallel -> 2 ohm total ESR
dt         = 1 / 1000.0

VCAP_MIN = 10.5
VCAP_MAX = 17.2

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
E_MAX = 0.5 * C * VCAP_MAX ** 2

E_STEP = 15.0                 # Default energy when S/E has no number, J
STORE_TIME_MS = 5000         # Store target time: 5 seconds


# ============================================================
# CSV Logging Settings
# ============================================================

# Send command C to save a 40-second CSV file directly on the Pico.
# The file is overwritten each time you start a new log.
CSV_FILENAME = "log.csv"
CSV_LOG_TIME_MS = 80000      # 40 seconds
CSV_LOG_INTERVAL_MS = 50     # one row every 50 ms = about 800 rows


# ============================================================
# Current Settings
# ============================================================

# For the default +15 J in 5 s, required power is 3 W. Around 10.5-16 V this
# means roughly 0.19-0.29 A ideal capacitor current, but practical SMPS loss
# and control delay mean we use a higher current target. Watch board temperature.
I_STORE    =  0.70          # Faster but safer store current target, A
I_EXTRACT  = -0.70          # Safer extract current target, A
I_PRECHARGE = 0.08          # Low-current start when capacitor is below VCAP_STORE_MIN, A
I_MAINTAIN =  0.03           # Maintain current target, A
I_LIMIT    =  1.20          # Hard overcurrent trip limit, A

V_BUS_PRECHARGE_MIN = 8.0
VCAP_PRECHARGE_MIN = 0.8
PRECHARGE_TARGET_V = VCAP_MIN
PRECHARGE_PWM_HARD_MAX = 12520
PRECHARGE_START_PWM = 0
PRECHARGE_PWM_MAX_STEP_UP = 2.0
PRECHARGE_PWM_MAX_STEP_DOWN = 80.0


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
# This version ramps upward faster to reach the 15 J target sooner.
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

# If MAINTAIN still detects negative current, jump back to a known safer
# holding PWM instead of waiting for the hard overcurrent trip.
MAINTAIN_REVERSE_CURRENT_LIMIT = -0.20
MAINTAIN_RECOVERY_PWM = 14500

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
EXTRACT_BRAKE_1_J = 1.30
EXTRACT_BRAKE_2_J = 0.65
EXTRACT_BRAKE_3_J = 0.25
I_EXTRACT_BRAKE_1 = -0.30
I_EXTRACT_BRAKE_2 = -0.16
I_EXTRACT_BRAKE_3 = -0.06
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
# Original PI variables retained for compatibility
# ============================================================

kp = 10
ki = 0.05
kd = 0

int_err = 0.0
prev_err = 0.0

INT_MIN = -5000
INT_MAX = 5000


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
precharge_followup_energy_j = 0.0
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


# ============================================================
# Snapshot for P Command
# ============================================================

last_va = 0.0
last_va_terminal = 0.0
last_vbus = 0.0
last_IL = 0.0
last_E = 0.0
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
    return 1.015 * (12490 / 2490) * 3.3 * (vb_pin.read_u16() / 65536)


def read_vbus():
    return 1.017 * (12490 / 2490) * 3.3 * (va_pin.read_u16() / 65536)


def correct_vcap_for_esr(v_terminal, cap_current):
    # IL is positive while charging and negative while extracting.
    return v_terminal - cap_current * CAP_ESR_OHMS


def write_active_pwm(command_pwm):
    global last_pwm_actual

    pwm_out = int(clamp(command_pwm, MIN_PWM, MAX_PWM))
    duty_actual = int(clamp(65536 - pwm_out, 0, 65535))
    last_pwm_actual = duty_actual
    pwm.duty_u16(duty_actual)


def reset_pid():
    global int_err, prev_err

    int_err = 0.0
    prev_err = 0.0


def calculate_store_start_pwm(vcap):
    """
    Safe cold-start PWM for STORE mode.

    v12 reason:
    When V_BUS was changed to 9 V, the capacitor was tested at about
    Vcap=10.25 V. Starting STORE from PWM=1000 made the converter enter
    a low-duty reverse-current region and caused a store reverse-current
    trip. SAFE_HOLD later showed that PWM around 8k-9k was needed before
    current became small positive again.

    Rule:
    - Vcap close to bus: start low and ramp up.
    - Vcap already above bus: skip the dangerous low-duty region.
    - Warm starts from MAINTAIN are handled separately and use the present
      stable PWM instead of this cold-start function.
    """
    dv = vcap - V_BUS

    if dv >= STORE_COLD_HIGH_DELTA:
        return STORE_COLD_HIGH_PWM

    if dv >= STORE_COLD_MID_DELTA:
        return STORE_COLD_MID_PWM

    return STORE_SAFE_START_PWM


def get_store_pwm_hard_max(vcap):
    """
    Adaptive STORE PWM limit.

    Latest CSV result:
    - STORE reached PWM=12520 near Vcap=11.7-11.8 V.
    - At that point IL dropped far below I_STORE, so voltage rise became slow.
    - Therefore allow a small extra ceiling only when Vcap is already high.

    Safety rule:
    - Below 11.6 V: stay at 12520.
    - At/above STORE_HIGH_VCAP: allow the opened high ceiling.
    - Keep STORE_REVERSE_CURRENT_LIMIT tight so any negative current
      immediately goes to SAFE_HOLD.
    """
    if vcap >= STORE_HIGH_VCAP:
        return STORE_PWM_HARD_MAX_HIGH

    return STORE_PWM_HARD_MAX_BASE


def get_store_pwm_step_up(vcap):
    """
    STORE ramp-up speed.

    Low voltage already reached 15 J in about 5 s, so keep the old base ramp.
    High voltage STORE was slower because PWM had to climb from about 23k to
    25k+ while current stayed slightly below target, so allow faster upward
    PWM movement only at higher Vcap.
    """
    if vcap >= STORE_PWM_STEP_HIGH_VCAP:
        return STORE_PWM_MAX_STEP_UP_HIGH

    if vcap >= STORE_PWM_STEP_MID_VCAP:
        return STORE_PWM_MAX_STEP_UP_MID

    return STORE_PWM_MAX_STEP_UP


def calculate_store_warm_start_pwm(vcap):
    """
    STORE start PWM.

    Cold start from STOPPED/TRIPPED: use 1000.
    Warm start from MAINTAIN: keep the current stable PWM.

    Your latest test showed that after STORE finished, MAINTAIN
    settled around PWM 8800 and IL about 0.020 A. Pressing S again
    should therefore continue from that stable point. Jumping back
    to PWM=1000 caused a reverse-current transient.
    """
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
    """
    Begin EXTRACT from the current stable MAINTAIN PWM.

    Measured result at Vcap about 12.16 V:
        MAINTAIN duty about 12084 -> IL about +0.020 A
        old EXTRACT duty 64536    -> IL about +0.954 A

    Therefore the Vcap/Vbus formula is not usable for discharge start.
    """
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
    global duty, duty_cmd, last_pwm_applied, last_pwm_actual, hard_stopped

    duty = 0
    duty_cmd = 0.0
    last_pwm_applied = 0
    last_pwm_actual = 0

    pwm.duty_u16(0)

    hard_stopped = True


def pwm_soft_off():
    """
    Softly ramp PWM down for manual H stop.

    This reduces the reverse-current kick that was observed when PWM was
    forced directly to zero after charging.
    """
    global duty, duty_cmd, last_pwm_applied, last_pwm_actual, hard_stopped

    temp = int(last_pwm_actual)

    while temp > 0:
        temp -= SOFT_STOP_STEP
        if temp < 0:
            temp = 0

        pwm.duty_u16(temp)
        time.sleep_ms(SOFT_STOP_DELAY_MS)

    duty = 0
    duty_cmd = 0.0
    last_pwm_applied = 0
    last_pwm_actual = 0
    hard_stopped = True


def pwm_start(initial_pwm):
    global duty, duty_cmd, last_pwm_applied, hard_stopped

    duty = int(clamp(initial_pwm, MIN_PWM, MAX_PWM))
    duty_cmd = float(duty)

    last_pwm_applied = duty
    write_active_pwm(last_pwm_applied)

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
    write_active_pwm(last_pwm_applied)
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
    global precharge_followup_energy_j

    trip = False
    trip_reason = ""

    pwm_soft_off()

    mode = "STOPPED"
    I_target = 0.0

    action_in_progress = False
    E_delta = 0.0
    E_target_action = 0.0
    store_deadline_reported = False
    precharge_followup_energy_j = 0.0

    reset_pid()

    command = ""

    print("Stopped. Send S/E/U to resume.")


def do_maintain(va):
    global command, mode, I_target, action_in_progress
    global E_delta, E_target_action, IL_filtered

    # MAINTAIN keeps PWM active instead of setting it to zero.
    # This prevents the SMPS from entering the reverse-current state
    # seen immediately after STORE finished.
    if hard_stopped:
        # From STOPPED, starting MAINTAIN at 1000 can also cross an
        # uncontrolled region. Start at the protected hold minimum.
        pwm_start(MAINTAIN_PWM_MIN)

    IL_filtered = last_IL

    command = ""
    mode = "MAINTAIN"
    I_target = I_MAINTAIN

    action_in_progress = False
    E_delta = 0.0
    E_target_action = 0.0

    reset_pid()


def calculate_vhold_current(va):
    # Simple voltage-floor hold around VCAP_MIN.
    # Below VCAP_MIN: gently charge.
    # Around VCAP_MIN: small positive float current.
    # Above VCAP_MIN: allow a very small discharge command, but the
    # MAINTAIN_PWM_MIN protection prevents falling into the dangerous
    # low-duty reverse-current region.
    if va < V_HOLD_REF - V_HOLD_BAND:
        return I_VHOLD_CHARGE

    if va > V_HOLD_REF + V_HOLD_BAND:
        return I_VHOLD_DISCHARGE

    return I_VHOLD_FLOAT



def calculate_safe_recovery_pwm(va):
    """
    Choose a safer hold PWM after EXTRACT or overcurrent recovery.

    From the latest log:
    - around 12.5 V, MAINTAIN was stable near 14500 PWM
    - around 14.6 V, MAINTAIN was stable near 23000 PWM

    A fixed 8500 PWM is too low at high capacitor voltage and can keep
    discharging after EXTRACT finishes.
    """
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
    """
    Keep normal MAINTAIN away from the low-duty reverse-current region.
    The minimum is raised when Vcap is high.
    """
    safe_pwm = calculate_safe_recovery_pwm(va) - 2500
    return int(clamp(safe_pwm, MAINTAIN_PWM_MIN, 23000))


def calculate_active_maintain_min_pwm(va):
    """
    Normal MAINTAIN needs a high minimum PWM at high Vcap, but jumping
    directly to that minimum right after EXTRACT caused a positive current
    spike. During the short handover period, ramp the minimum upward instead.
    """
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

def calculate_extract_target_current(E_now):
    """
    Energy-based EXTRACT braking.

    During EXTRACT, the fixed -0.30 A target can overshoot because the
    converter current does not instantly return to zero at the exact 5 J point.
    This function reduces the negative current earlier as the remaining energy
    approaches zero, so the handover to MAINTAIN is smoother.
    """
    remaining_J = (E_initial - E_target_action) - E_now
    # The expression above is negative before the target. Use the direct form
    # below for clarity: target energy after extraction is E_initial - target.
    target_E = E_initial - E_target_action
    remaining_J = E_now - target_E

    if remaining_J <= EXTRACT_BRAKE_3_J:
        return I_EXTRACT_BRAKE_3

    if remaining_J <= EXTRACT_BRAKE_2_J:
        return I_EXTRACT_BRAKE_2

    if remaining_J <= EXTRACT_BRAKE_1_J:
        return I_EXTRACT_BRAKE_1

    return I_EXTRACT


def enter_maintain_after_extract(va):
    """
    Enter MAINTAIN after EXTRACT smoothly.

    The previous version could jump from the final EXTRACT PWM around 13k
    straight to a hold PWM around 15k. That produced a positive current
    spike. This version only adds a small PWM step first, then lets MAINTAIN
    ramp back up slowly.
    """
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
    write_active_pwm(last_pwm_applied)

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

    recovery_pwm = calculate_safe_recovery_pwm(last_va)

    if recovery_pwm < MAINTAIN_RECOVERY_PWM:
        recovery_pwm = MAINTAIN_RECOVERY_PWM

    print(
        "MAINTAIN reverse current {:.3f}A -> recovery PWM {}".format(
            IL,
            recovery_pwm
        )
    )

    duty = int(clamp(recovery_pwm, MIN_PWM, MAX_PWM))
    duty_cmd = float(duty)
    last_pwm_applied = duty
    write_active_pwm(last_pwm_applied)

    IL_filtered = IL


def warn_if_current_while_stopped(now_ms, va, IL):
    """
    Warn when current flows while PWM is OFF.

    This is the hardware condition seen in the latest log: duty=0,
    pwm_applied=0, but IL was still much larger than zero and Vcap kept
    rising. That means the external source/bus is charging the capacitor
    through a path outside the PWM loop.
    """
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

    # If a previous file is still open, close it first.
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
            "action_in_progress,Vbus,Vterm,Vcap,IL,IL_filtered,E,E_delta,"
            "SoC,I_target,pwm_out,duty_actual\n"
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


def update_csv_log(now_ms, vbus, va_terminal, va, IL, E, SoC):
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
            "{},{},{},{},{},{},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},"
            "{:.4f},{:.4f},{:.2f},{:.4f},{},{}\n".format(
                now_ms,
                elapsed_ms,
                mode,
                int(trip),
                int(hard_stopped),
                int(action_in_progress),
                vbus,
                va_terminal,
                va,
                IL,
                IL_filtered,
                E,
                E_delta,
                SoC,
                I_target,
                last_pwm_applied,
                last_pwm_actual
            )
        )

        csv_rows += 1

        # Flush once per second so data is not lost if power is removed.
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
        "mode={} trip={} Vbus={:.3f}V Vterm={:.3f}V Vcap={:.3f}V IL={:.3f}A "
        "E={:.3f}J Espace={:.3f}J Eout={:.3f}J "
        "dE={:.3f}J SoC={:.1f}% target={:.3f}A "
        "pwm_out={} duty_actual={} t={:.3f}s".format(
            mode,
            trip,
            last_vbus,
            last_va_terminal,
            last_va,
            last_IL,
            last_E,
            energy_space,
            energy_available,
            E_delta,
            last_SoC,
            I_target,
            last_pwm_applied,
            last_pwm_actual,
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

print("Ready. Commands: S[J] E[J] U H P C")
print("Examples: S8 charges 8J, E3.5 extracts 3.5J. S/E alone use default.")
print("PWM initially OFF.")
print("Port wiring: A port = DC bus, B port = capacitor.")
print("Active PWM uses bidirectional convention: duty_actual = 65536 - pwm_out.")
print("ESR correction enabled: Vcap = Vterm - IL * {:.3f} ohm".format(CAP_ESR_OHMS))
print(
    "PRECHARGE enabled: S below {:.3f}V charges at {:.3f}A until {:.3f}V; "
    "requires Vbus >= {:.3f}V.".format(
        VCAP_STORE_MIN,
        I_PRECHARGE,
        PRECHARGE_TARGET_V,
        V_BUS_PRECHARGE_MIN
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
    "({:.3f}-{:.3f}V)".format(
        E_STEP,
        E_MIN,
        E_MAX,
        VCAP_MIN,
        VCAP_MAX
    )
)
print(
    "OPEN LIMIT TEST: STORE cold start low={} mid={} high={} using Vcap-Vbus thresholds {:.2f}V/{:.2f}V; hard_max_base={} hard_max_high={} high_from={:.2f}V; ramp_up_high={} count/ms; overcurrent->SAFE_HOLD; MAINTAIN base min PWM={}; stopped-current block={:.3f}A; EXTRACT floor={:.2f}V -> V_HOLD".format(
        STORE_SAFE_START_PWM,
        STORE_COLD_MID_PWM,
        STORE_COLD_HIGH_PWM,
        STORE_COLD_MID_DELTA,
        STORE_COLD_HIGH_DELTA,
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
        vbus = read_vbus()
        va_terminal = read_vcap()
        IL = ina.vshunt() / SHUNT_OHMS
        va = correct_vcap_for_esr(va_terminal, IL)

    except Exception as e:
        do_trip("sensor error: " + str(e))
        continue


    E = 0.5 * C * va ** 2
    SoC = clamp(E / E_MAX * 100.0, 0.0, 100.0)

    last_va = va
    last_va_terminal = va_terminal
    last_vbus = vbus
    last_IL = IL
    last_E = E
    last_SoC = SoC

    now_ms = time.ticks_ms()
    update_csv_log(now_ms, vbus, va_terminal, va, IL, E, SoC)

    # If current is flowing while PWM is already OFF, the converter is
    # not the thing controlling current. Warn, but do not call pwm_off()
    # again because that cannot stop a hardware bypass path.
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
        and mode == "PRECHARGE"
        and IL <= STORE_REVERSE_CURRENT_LIMIT
    ):
        if abs(IL) >= I_LIMIT:
            do_trip("precharge reverse overcurrent {:.3f}A".format(IL), reverse_hold=True)
            continue

        duty = PRECHARGE_START_PWM
        duty_cmd = float(duty)
        last_pwm_applied = duty
        write_active_pwm(last_pwm_applied)
        IL_filtered = IL

        print(
            "PRECHARGE reverse current {:.3f}A: holding at start PWM {}. "
            "If this repeats, current sign or power path direction is wrong.".format(
                IL,
                PRECHARGE_START_PWM
            )
        )
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
        # In a bidirectional synchronous SMPS, PWM=0 is not guaranteed to be
        # an open circuit. Therefore both positive and negative overcurrent
        # now enter SAFE_HOLD instead of forcing PWM directly to 0.
        do_trip("overcurrent {:.3f}A".format(IL), reverse_hold=True)
        continue

    if not trip and not hard_stopped and va >= VCAP_MAX:
        do_trip("overvoltage {:.3f}V".format(va))
        continue

    if not trip and mode == "EXTRACT" and va <= VCAP_MIN:
        # VCAP_MIN is the normal lower operating floor for EXTRACT,
        # not a fault. Stop extracting and hold around VCAP_MIN.
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
        # Start a 40-second CSV recording saved directly on the Pico.
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

        # New v16 behaviour:
        # You do not need to type H first after reset or after a normal STOPPED
        # state. If the old trip flag is active but PWM is already off and
        # measured current is small, S clears the trip flag and starts STORE.
        # If current is still large while PWM is off, STORE is still blocked
        # because that means the hardware path is uncontrolled.
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

        elif va < VCAP_PRECHARGE_MIN:
            print(
                "PRECHARGE blocked: Vcap={:.3f}V below {:.3f}V. "
                "Check capacitor voltage, ADC divider, and wiring before starting.".format(
                    va,
                    VCAP_PRECHARGE_MIN
                )
            )

        elif va < VCAP_STORE_MIN and vbus < V_BUS_PRECHARGE_MIN:
            print(
                "PRECHARGE blocked: Vbus={:.3f}V below {:.3f}V. "
                "Check A port DC bus supply.".format(
                    vbus,
                    V_BUS_PRECHARGE_MIN
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

        elif E >= E_MAX - 0.05:
            print("Already at max energy ({:.3f}J).".format(E))

        elif va < VCAP_STORE_MIN:
            start_pwm = PRECHARGE_START_PWM

            if hard_stopped:
                pwm_start(start_pwm)
                store_start_type = "cold"
            else:
                duty = int(clamp(start_pwm, MIN_PWM, PRECHARGE_PWM_HARD_MAX))
                duty_cmd = float(duty)
                last_pwm_applied = duty
                write_active_pwm(last_pwm_applied)
                hard_stopped = False
                reset_pid()
                store_start_type = "warm"

            E_initial = E
            E_delta = 0.0
            E_target_action = 0.5 * C * PRECHARGE_TARGET_V ** 2 - E

            if E_target_action < 0.0:
                E_target_action = 0.0

            precharge_followup_energy_j = command_energy_j
            I_target = I_PRECHARGE
            action_in_progress = True
            mode = "PRECHARGE"

            action_start_ms = time.ticks_ms()
            store_deadline_reported = True

            IL_filtered = IL
            reset_pid()

            print(
                "PRECHARGE: Vcap={:.3f}V -> {:.3f}V, target +{:.3f}J, "
                "then STORE +{:.3f}J. I_target={:.3f}A start_pwm={} type={} Vbus={:.3f}V".format(
                    va,
                    PRECHARGE_TARGET_V,
                    E_target_action,
                    precharge_followup_energy_j,
                    I_target,
                    start_pwm,
                    store_start_type,
                    vbus
                )
            )

        else:
            start_pwm = calculate_store_warm_start_pwm(va)

            if hard_stopped:
                # Cold start from PWM off.
                pwm_start(start_pwm)
                store_start_type = "cold"
            else:
                # Warm start from an already stable active PWM, usually MAINTAIN.
                # Do NOT jump back to PWM=1000; that caused reverse current.
                duty = int(clamp(start_pwm, MIN_PWM, get_store_pwm_hard_max(va)))
                duty_cmd = float(duty)
                last_pwm_applied = duty
                write_active_pwm(last_pwm_applied)
                hard_stopped = False
                reset_pid()
                store_start_type = "warm"

            E_initial = E
            E_delta = 0.0
            E_target_action = min(command_energy_j, E_MAX - E)

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
            # Use the currently stable MAINTAIN PWM. Do not jump to 64536.
            start_pwm = calculate_extract_start_pwm(va)
            extract_return_pwm = start_pwm

            pwm_start(start_pwm)

            E_initial = E
            E_delta = 0.0
            E_target_action = min(command_energy_j, E - E_MIN)

            I_target = I_EXTRACT
            action_in_progress = True
            mode = "EXTRACT"

            # Reset EXTRACT timer too, otherwise the printed EXTRACT time can
            # include time from a previous action.
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

        if mode == "PRECHARGE" and va >= PRECHARGE_TARGET_V:

            elapsed_s = time.ticks_diff(
                time.ticks_ms(),
                action_start_ms
            ) / 1000.0

            print(
                "PRECHARGE done. Vcap={:.3f}V E={:.3f}J "
                "time={:.3f}s -> STORE +{:.3f}J".format(
                    va,
                    E,
                    elapsed_s,
                    precharge_followup_energy_j
                )
            )

            E_initial = E
            E_delta = 0.0
            E_target_action = min(precharge_followup_energy_j, E_MAX - E)

            I_target = I_STORE
            mode = "STORE"
            action_start_ms = time.ticks_ms()
            store_deadline_reported = False
            IL_filtered = IL
            reset_pid()

            print(
                "STORE: target +{:.3f}J after precharge from {:.3f}J. "
                "I_target={:.3f}A".format(
                    E_target_action,
                    E,
                    I_target
                )
            )

        elif mode == "STORE" and E_delta >= E_target_action:

            elapsed_s = time.ticks_diff(
                time.ticks_ms(),
                action_start_ms
            ) / 1000.0

            print(
                "STORE done. deltaE={:.3f}J time={:.3f}s "
                "E={:.3f}J Vcap={:.3f}V".format(
                    E_delta,
                    elapsed_s,
                    E,
                    va
                )
            )

            if elapsed_s <= STORE_TIME_MS / 1000.0:
                print("STORE target achieved within 5 seconds.")
            else:
                print("STORE target completed, but slower than 5 seconds.")

            # Do not turn PWM off here. With this bidirectional SMPS,
            # PWM=0 after STORE produced reverse current and discharged
            # the supercapacitor. Hand over directly to MAINTAIN instead.
            do_maintain(va)

            print(
                "STORE finished -> MAINTAIN. "
                "Holding with target current {:.3f}A.".format(
                    I_MAINTAIN
                )
            )


        elif (
            mode == "STORE"
            and not store_deadline_reported
            and time.ticks_diff(time.ticks_ms(), action_start_ms)
            >= STORE_TIME_MS
        ):

            print(
                "STORE missed 5.000s target. "
                "deltaE={:.3f}J Vcap={:.3f}V IL={:.3f}A".format(
                    E_delta,
                    va,
                    IL
                )
            )

            store_deadline_reported = True


        elif mode == "EXTRACT" and E_delta <= -(E_target_action - EXTRACT_DONE_TOL_J):

            elapsed_s = time.ticks_diff(
                time.ticks_ms(),
                action_start_ms
            ) / 1000.0

            print(
                "EXTRACT done. deltaE={:.3f}J time={:.3f}s "
                "E={:.3f}J Vcap={:.3f}V".format(
                    E_delta,
                    elapsed_s,
                    E,
                    va
                )
            )

            enter_maintain_after_extract(va)


    # --------------------------------------------------------
    # Current Control
    # --------------------------------------------------------

    control_allowed = (not hard_stopped) and ((not trip) or mode == "SAFE_HOLD")

    if control_allowed:

        if mode == "STORE" or mode == "PRECHARGE":

            # ------------------------------------------------
            # SAFE STORE controller
            #
            # The old code jumped directly to about PWM=25000,
            # which caused 2.5-3.2 A overcurrent. In the 0.2 A test,
            # PWM around 12.6k caused a sudden negative-current trip.
            # This version caps STORE PWM within the active safety limits and ramps
            # upward faster only inside the safe range.
            # ------------------------------------------------

            IL_filtered = (
                (1.0 - IL_FILTER_ALPHA) * IL_filtered
                + IL_FILTER_ALPHA * IL
            )

            err = I_target - IL_filtered
            raw_step = STORE_PWM_GAIN * err

            if raw_step >= 0:
                if mode == "PRECHARGE":
                    max_step_up = PRECHARGE_PWM_MAX_STEP_UP
                else:
                    max_step_up = get_store_pwm_step_up(va)

                pwm_step = clamp(
                    raw_step,
                    0.0,
                    max_step_up
                )
            else:
                if mode == "PRECHARGE":
                    max_step_down = PRECHARGE_PWM_MAX_STEP_DOWN
                else:
                    max_step_down = STORE_PWM_MAX_STEP_DOWN

                pwm_step = clamp(
                    raw_step,
                    -max_step_down,
                    0.0
                )

            if mode == "PRECHARGE":
                store_pwm_max = PRECHARGE_PWM_HARD_MAX
            else:
                store_pwm_max = get_store_pwm_hard_max(va)

            duty_cmd = clamp(
                duty_cmd + pwm_step,
                MIN_PWM,
                store_pwm_max
            )

            duty = int(duty_cmd)


        elif mode == "MAINTAIN" or mode == "SAFE_HOLD" or mode == "V_HOLD":

            if mode == "V_HOLD":
                I_target = calculate_vhold_current(va)

            IL_filtered = (
                (1.0 - IL_FILTER_ALPHA) * IL_filtered
                + IL_FILTER_ALPHA * IL
            )

            err = I_target - IL_filtered
            raw_step = MAINTAIN_PWM_GAIN * err

            if raw_step >= 0:
                pwm_step = clamp(
                    raw_step,
                    0.0,
                    MAINTAIN_PWM_MAX_STEP_UP
                )
            else:
                pwm_step = clamp(
                    raw_step,
                    -MAINTAIN_PWM_MAX_STEP_DOWN,
                    0.0
                )

            # Critical protection: do not allow the hold duty to fall
            # into the low-duty reverse-current region. At higher Vcap,
            # the safe minimum PWM must also be higher.
            duty_cmd = clamp(
                duty_cmd + pwm_step,
                calculate_active_maintain_min_pwm(va),
                get_store_pwm_hard_max(va)
            )

            duty = int(duty_cmd)


        elif mode == "EXTRACT":

            # ------------------------------------------------
            # EXTRACT controller
            #
            # Duty is lowered gradually from the MAINTAIN point.
            # This avoids the positive-current jump and the later
            # negative-current overshoot seen in the previous result.
            # ------------------------------------------------

            IL_filtered = (
                (1.0 - IL_FILTER_ALPHA) * IL_filtered
                + IL_FILTER_ALPHA * IL
            )

            I_target = calculate_extract_target_current(E)

            err = I_target - IL_filtered

            pwm_step = clamp(
                EXTRACT_PWM_GAIN * err,
                -EXTRACT_PWM_MAX_STEP,
                EXTRACT_PWM_MAX_STEP
            )

            duty_cmd = clamp(
                duty_cmd + pwm_step,
                EXTRACT_MIN_PWM,
                MAX_PWM
            )

            duty = int(duty_cmd)


        last_pwm_applied = duty
        write_active_pwm(last_pwm_applied)


    # --------------------------------------------------------
    # Periodic Status Output
    # --------------------------------------------------------

    if time.ticks_diff(now_ms, last_status_print_ms) >= STATUS_PRINT_INTERVAL_MS:
        last_status_print_ms = now_ms
        print_status()

    count += 1

