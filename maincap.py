# Supercapacitor Current Controller
# Hardware: 0.25 F cap, Vbus = 10.0 V
# Commands: S=store  E=extract  U=maintain  H=stop  P=print

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

V_BUS      = 10.0
C          = 0.5
SHUNT_OHMS = 0.10
dt         = 1 / 1000.0

VCAP_MIN = 10.5
VCAP_MAX = 14.0

E_MIN = 0.5 * C * VCAP_MIN ** 2
E_MAX = 0.5 * C * VCAP_MAX ** 2

E_STEP = 5.0                 # Energy required for each S/E command, J
STORE_TIME_MS = 5000         # Store target time: 5 seconds


# ============================================================
# Current Settings
# ============================================================

I_STORE    =  0.35           # Store current target, A
I_EXTRACT  = -0.20           # Extract current target, A
I_MAINTAIN =  0.02           # Maintain current target, A
I_LIMIT    =  1.00           # Hard overcurrent trip limit, A


# ============================================================
# STORE Controller Tuning
# ============================================================

# From your successful charging data:
# around Vcap = 9 V, PWM near 21000 gave IL near 0.22 A.
# This avoids starting from PWM around 56000, which caused 0.647 A.
STORE_START_MIN = 18000
STORE_START_MAX = 32000

# Incremental proportional controller for STORE mode.
# The PWM moves only a limited amount per loop to reduce oscillation.
STORE_PWM_GAIN = 150.0
STORE_PWM_MAX_STEP = 20.0

# Low-pass filtering for measured current in STORE / MAINTAIN / EXTRACT modes.
IL_FILTER_ALPHA = 0.10
IL_filtered = 0.0

# EXTRACT target current is unchanged. These only restrict how fast PWM
# is allowed to move, because the negative-current region is very sensitive.
EXTRACT_PWM_GAIN = 60.0
EXTRACT_PWM_MAX_STEP = 1.0


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

mode = "STOPPED"
command = ""

hard_stopped = True

trip = False
trip_reason = ""

timer_elapsed = 0
count = 0


# ============================================================
# Snapshot for P Command
# ============================================================

last_va = 0.0
last_IL = 0.0
last_E = 0.0
last_SoC = 0.0


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
    return 1.017 * (12490 / 2490) * 3.3 * (va_pin.read_u16() / 65536)


def reset_pid():
    global int_err, prev_err

    int_err = 0.0
    prev_err = 0.0


def calculate_store_start_pwm(vcap):
    """
    Empirical starting PWM for STORE mode.

    Previous result:
        Vcap around 9.0 V, PWM around 21000 -> IL around 0.22 A.

    Increase starting PWM moderately as Vcap rises.
    """
    initial_duty = int(21000 + 5200 * (vcap - 8.90))

    return int(clamp(
        initial_duty,
        STORE_START_MIN,
        STORE_START_MAX
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
    global duty, duty_cmd, last_pwm_applied, hard_stopped

    duty = 0
    duty_cmd = 0.0
    last_pwm_applied = 0

    pwm.duty_u16(0)

    hard_stopped = True


def pwm_start(initial_pwm):
    global duty, duty_cmd, last_pwm_applied, hard_stopped

    duty = int(clamp(initial_pwm, MIN_PWM, MAX_PWM))
    duty_cmd = float(duty)

    last_pwm_applied = duty
    pwm.duty_u16(last_pwm_applied)

    hard_stopped = False

    reset_pid()


def do_trip(reason):
    global trip, trip_reason, command
    global action_in_progress, mode, I_target

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

    pwm_off()

    mode = "STOPPED"
    I_target = 0.0

    action_in_progress = False
    E_delta = 0.0
    E_target_action = 0.0
    store_deadline_reported = False

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
        pwm_start(calculate_store_start_pwm(va))

    IL_filtered = last_IL

    command = ""
    mode = "MAINTAIN"
    I_target = I_MAINTAIN

    action_in_progress = False
    E_delta = 0.0
    E_target_action = 0.0

    reset_pid()


def read_cmd():
    global command

    try:
        if poll.poll(0):
            line = sys.stdin.readline().strip().upper()

            if line and line[0] in "SEUHP":
                command = line[0]

    except:
        pass


def print_status():
    if action_in_progress and mode == "STORE":
        elapsed_s = time.ticks_diff(time.ticks_ms(), action_start_ms) / 1000.0
    else:
        elapsed_s = 0.0

    print(
        "mode={} trip={} Vcap={:.3f}V IL={:.3f}A E={:.3f}J "
        "dE={:.3f}J SoC={:.1f}% target={:.3f}A "
        "duty={} pwm_applied={} t={:.3f}s".format(
            mode,
            trip,
            last_va,
            last_IL,
            last_E,
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

ina = INA219(SHUNT_OHMS, 64)
ina.configure()

loop_timer = Timer(
    mode=Timer.PERIODIC,
    freq=1000,
    callback=tick
)

print("Ready. Commands: S E U H P")
print("PWM initially OFF.")
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
    "STORE target: +{:.3f}J within {:.3f}s".format(
        E_STEP,
        STORE_TIME_MS / 1000.0
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
        va = read_vcap()
        IL = -ina.vshunt() / SHUNT_OHMS

    except Exception as e:
        do_trip("sensor error: " + str(e))
        continue


    E = 0.5 * C * va ** 2
    SoC = clamp(E / E_MAX * 100.0, 0.0, 100.0)

    last_va = va
    last_IL = IL
    last_E = E
    last_SoC = SoC


    # --------------------------------------------------------
    # Safety Trips: execute immediately after measurement
    # --------------------------------------------------------

    if not trip and not hard_stopped and abs(IL) >= I_LIMIT:
        do_trip("overcurrent {:.3f}A".format(IL))
        continue

    if not trip and not hard_stopped and va >= VCAP_MAX:
        do_trip("overvoltage {:.3f}V".format(va))
        continue

    if not trip and mode == "EXTRACT" and va <= VCAP_MIN:
        do_trip("undervoltage {:.3f}V".format(va))
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

    elif command == "U":

        if trip:
            print("Trip active - send H first.")
            command = ""
        else:
            do_maintain(va)


    # --------------------------------------------------------
    # Start STORE
    # --------------------------------------------------------

    if command == "S" and not trip:

        command = ""

        if action_in_progress:
            print("Action in progress.")

        elif E >= E_MAX - 0.05:
            print("Already at max energy ({:.3f}J).".format(E))

        else:
            start_pwm = calculate_store_start_pwm(va)

            pwm_start(start_pwm)

            E_initial = E
            E_delta = 0.0
            E_target_action = min(E_STEP, E_MAX - E)

            I_target = I_STORE
            action_in_progress = True
            mode = "STORE"

            action_start_ms = time.ticks_ms()
            store_deadline_reported = False

            IL_filtered = IL

            reset_pid()

            print(
                "STORE: target +{:.3f}J within {:.3f}s "
                "from {:.3f}J start_pwm={}".format(
                    E_target_action,
                    STORE_TIME_MS / 1000.0,
                    E,
                    start_pwm
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
            print("Already at min energy ({:.3f}J).".format(E))

        elif hard_stopped:
            print(
                "EXTRACT blocked while PWM is OFF. "
                "Send U first, wait until MAINTAIN current is near "
                "{:.3f}A, then send E.".format(I_MAINTAIN)
            )

        else:
            # Use the currently stable MAINTAIN PWM. Do not jump to 64536.
            start_pwm = calculate_extract_start_pwm(va)

            pwm_start(start_pwm)

            E_initial = E
            E_delta = 0.0
            E_target_action = min(E_STEP, E - E_MIN)

            I_target = I_EXTRACT
            action_in_progress = True
            mode = "EXTRACT"

            IL_filtered = IL
            reset_pid()

            print(
                "EXTRACT: target -{:.3f}J from {:.3f}J "
                "start_pwm={} (from MAINTAIN)".format(
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

        if mode == "STORE" and E_delta >= E_target_action:

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


        elif mode == "EXTRACT" and E_delta <= -E_target_action:

            print(
                "EXTRACT done. E={:.3f}J Vcap={:.3f}V".format(
                    E,
                    va
                )
            )

            do_maintain(va)


    # --------------------------------------------------------
    # Current Control
    # --------------------------------------------------------

    if not trip and not hard_stopped:

        if mode == "STORE" or mode == "MAINTAIN":

            # ------------------------------------------------
            # STORE / MAINTAIN controller
            #
            # Use filtered current and bounded incremental
            # proportional control. This avoids the large
            # oscillation caused by repeatedly adding PI output
            # to PWM duty.
            # ------------------------------------------------

            IL_filtered = (
                (1.0 - IL_FILTER_ALPHA) * IL_filtered
                + IL_FILTER_ALPHA * IL
            )

            err = I_target - IL_filtered

            pwm_step = clamp(
                STORE_PWM_GAIN * err,
                -STORE_PWM_MAX_STEP,
                STORE_PWM_MAX_STEP
            )

            duty_cmd = clamp(
                duty_cmd + pwm_step,
                MIN_PWM,
                MAX_PWM
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

            err = I_target - IL_filtered

            pwm_step = clamp(
                EXTRACT_PWM_GAIN * err,
                -EXTRACT_PWM_MAX_STEP,
                EXTRACT_PWM_MAX_STEP
            )

            duty_cmd = clamp(
                duty_cmd + pwm_step,
                MIN_PWM,
                MAX_PWM
            )

            duty = int(duty_cmd)


        last_pwm_applied = duty
        pwm.duty_u16(last_pwm_applied)


    # --------------------------------------------------------
    # Periodic Status Output
    # --------------------------------------------------------

    # 1000 Hz loop, print every 50 loops = approximately 0.05 s
    if count % 50 == 0:
        print_status()

    count += 1
