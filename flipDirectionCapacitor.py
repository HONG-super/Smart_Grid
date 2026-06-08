# Simple Supercapacitor Controller
# A port = DC bus, B port = capacitor
# This file is intentionally small: active low-current charge/discharge first.
#
# Commands:
#   S        charge to V_CAP_MAX
#   S10      charge until +10 J, stopping at voltage/current limits
#   E        discharge DEFAULT_DISCHARGE_J
#   E5       discharge 5 J
#   H        stop, PWM off
#   P        print one status line
#
# PWM convention follows the provided bidirectional code:
#   pwm_out      = controller-side value
#   duty_actual  = 65536 - pwm_out
#
# Important:
#   pwm_off() uses pwm.duty_u16(0), not duty_actual = 0.
#   pwm_out starts near MAX_PWM_OUT and moves slowly.

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
pwm.duty_u16(0)

led = Pin("LED", Pin.OUT)
led.on()

CODE_VERSION = "parallel_2x_0p25F_ESR4_charge0p7_discharge0p7_2026_06_08"


# ============================================================
# Parameters
# ============================================================

V_BUS_MIN = 8.0
V_CAP_MIN = 4.0
V_CAP_MAX = 16.0
V_TERM_HARD_MIN = 0.2
V_TERM_HARD_MAX = 24.0

C_FARADS = 0.500
SHUNT_OHMS = 0.10
CAP_ESR_OHMS = 2.00

MIN_PWM_OUT = 0
MAX_PWM_OUT = 65535

I_PRECHARGE_LIMIT = 0.85
I_CHARGE_TARGET = 0.70
I_CHARGE_REVERSE_STOP = -0.05
I_DISCHARGE_TARGET = -0.70
I_DISCHARGE_LIMIT = -0.85
I_HARD_LIMIT = 1.20
CHARGE_ENERGY_LOSS_STOP_J = 0.20

DEFAULT_CHARGE_J = 0.0
DEFAULT_DISCHARGE_J = 5.0
STATUS_INTERVAL_MS = 1000

CHARGE_START_PWM_OUT = MAX_PWM_OUT
CHARGE_MIN_PWM_OUT = 64536
CHARGE_STEP_DOWN = 1
CHARGE_STEP_UP = 200

DISCHARGE_START_PWM_OUT = 64536
DISCHARGE_MIN_PWM_OUT = 30000
DISCHARGE_STEP_DOWN = 4
DISCHARGE_STEP_UP = 200


# ============================================================
# State
# ============================================================

timer_elapsed = 0
mode = "STOPPED"
trip = False
trip_reason = ""

target_energy_j = 0.0
start_energy_j = 0.0

last_vbus = 0.0
last_vterm = 0.0
last_vcap = 0.0
last_il = 0.0
last_energy = 0.0
last_pwm_out = 0
last_duty_actual = 0
last_status_ms = 0

command = ""
command_energy_j = DEFAULT_CHARGE_J


# ============================================================
# Helpers
# ============================================================

def clamp(value, lower, upper):
    return max(min(value, upper), lower)


def tick(t):
    global timer_elapsed
    timer_elapsed = 1


def read_vbus():
    return 1.017 * (12490 / 2490) * 3.3 * (va_pin.read_u16() / 65536)


def read_vterm():
    return 1.015 * (12490 / 2490) * 3.3 * (vb_pin.read_u16() / 65536)


def correct_vcap_for_esr(vterm, il):
    return vterm - il * CAP_ESR_OHMS


def energy_from_vcap(vcap):
    return 0.5 * C_FARADS * vcap * vcap


def energy_at_voltage(voltage):
    return 0.5 * C_FARADS * voltage * voltage


def usable_energy(vcap):
    return max(0.0, energy_from_vcap(vcap) - energy_at_voltage(V_CAP_MIN))


def remaining_energy_space(vcap):
    return max(0.0, energy_at_voltage(V_CAP_MAX) - energy_from_vcap(vcap))


def pwm_off():
    global last_pwm_out, last_duty_actual
    last_pwm_out = 0
    last_duty_actual = 0
    pwm.duty_u16(0)


def write_pwm_out(pwm_out):
    global last_pwm_out, last_duty_actual
    pwm_out = int(clamp(pwm_out, MIN_PWM_OUT, MAX_PWM_OUT))
    duty_actual = int(clamp(65536 - pwm_out, 0, 65535))
    last_pwm_out = pwm_out
    last_duty_actual = duty_actual
    pwm.duty_u16(duty_actual)


def parse_energy_amount(text, default_value):
    text = text.strip().upper()

    if not text:
        return default_value

    if text[0] in "=:":
        text = text[1:].strip()

    if text.endswith("J"):
        text = text[:-1].strip()

    try:
        value = float(text)
    except:
        return None

    if value < 0.0:
        return None

    return value


def print_status():
    usable_j = usable_energy(last_vcap)
    space_j = remaining_energy_space(last_vcap)

    print(
        "mode={} trip={} Vbus={:.3f}V Vterm={:.3f}V Vcap={:.3f}V "
        "IL={:.3f}A E={:.3f}J usable={:.3f}J space={:.3f}J "
        "dE={:.3f}J targetE={:.3f}J "
        "pwm_out={} duty_actual={}".format(
            mode,
            trip,
            last_vbus,
            last_vterm,
            last_vcap,
            last_il,
            last_energy,
            usable_j,
            space_j,
            last_energy - start_energy_j,
            target_energy_j,
            last_pwm_out,
            last_duty_actual
        )
    )

    if trip:
        print("trip_reason:", trip_reason)


def do_trip(reason):
    global mode, trip, trip_reason
    trip = True
    trip_reason = reason
    mode = "TRIPPED"
    pwm_off()
    print("TRIP:", reason)


def do_stop():
    global mode, trip, trip_reason, target_energy_j, start_energy_j
    pwm_off()
    mode = "STOPPED"
    trip = False
    trip_reason = ""
    target_energy_j = 0.0
    start_energy_j = last_energy
    print("Stopped. PWM off.")


def start_charge(request_j):
    global mode, target_energy_j, start_energy_j

    if trip:
        print("Trip active. Send H first.")
        return

    if last_vterm < V_TERM_HARD_MIN or last_vterm > V_TERM_HARD_MAX:
        print(
            "Start blocked: Vterm={:.3f}V outside hard sensing range {:.3f}-{:.3f}V.".format(
                last_vterm,
                V_TERM_HARD_MIN,
                V_TERM_HARD_MAX
            )
        )
        return

    if last_vcap >= V_CAP_MAX:
        print(
            "Charge blocked: Vcap={:.3f}V already at/above {:.3f}V.".format(
                last_vcap,
                V_CAP_MAX
            )
        )
        return

    if last_vbus < V_BUS_MIN:
        print(
            "Start blocked: Vbus={:.3f}V below {:.3f}V.".format(
                last_vbus,
                V_BUS_MIN
            )
        )
        return

    if abs(last_il) >= I_PRECHARGE_LIMIT:
        print(
            "Start blocked: IL={:.3f}A exceeds precharge limit {:.3f}A.".format(
                last_il,
                I_PRECHARGE_LIMIT
            )
        )
        return

    write_pwm_out(CHARGE_START_PWM_OUT)
    start_energy_j = last_energy
    target_energy_j = request_j
    mode = "CHARGE"

    print(
        "CHARGE started: Vcap={:.3f}V -> {:.3f}V, "
        "target +{:.3f}J, current target {:.3f}A.".format(
            last_vcap,
            V_CAP_MAX,
            target_energy_j,
            I_CHARGE_TARGET
        )
    )


def start_discharge(request_j):
    global mode, target_energy_j, start_energy_j

    if trip:
        print("Trip active. Send H first.")
        return

    if request_j <= 0.0:
        print("Invalid discharge amount. Use E or E5.")
        return

    if last_vcap <= V_CAP_MIN:
        print(
            "Discharge blocked: Vcap={:.3f}V is already at/below {:.3f}V.".format(
                last_vcap,
                V_CAP_MIN
            )
        )
        return

    if last_vbus < V_BUS_MIN:
        print(
            "Discharge blocked: Vbus={:.3f}V below {:.3f}V.".format(
                last_vbus,
                V_BUS_MIN
            )
        )
        return

    start_energy_j = last_energy
    target_energy_j = request_j
    mode = "DISCHARGE"
    write_pwm_out(DISCHARGE_START_PWM_OUT)

    print(
        "DISCHARGE started: target -{:.3f}J, stop at Vcap <= {:.3f}V, "
        "current target {:.3f}A.".format(
            target_energy_j,
            V_CAP_MIN,
            I_DISCHARGE_TARGET
        )
    )


def read_cmd():
    global command, command_energy_j

    try:
        if poll.poll(0):
            line = sys.stdin.readline().strip().upper()

            if not line:
                return

            cmd = line[0]

            if cmd == "S":
                amount = parse_energy_amount(line[1:], DEFAULT_CHARGE_J)

                if amount is None:
                    print("Invalid command. Use S, S10, or S10J.")
                    return

                command_energy_j = amount
                command = "S"

            elif cmd == "E":
                amount = parse_energy_amount(line[1:], DEFAULT_DISCHARGE_J)

                if amount is None:
                    print("Invalid command. Use E, E5, or E5J.")
                    return

                command_energy_j = amount
                command = "E"

            elif cmd in "HP":
                command = cmd

            else:
                print("Unknown command. Use S, E, H, or P.")

    except:
        pass


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
# Initialisation
# ============================================================

poll = select.poll()
poll.register(sys.stdin, select.POLLIN)

ina = INA219(SHUNT_OHMS, 64)
ina.configure()

loop_timer = Timer(
    mode=Timer.PERIODIC,
    freq=1000,
    callback=tick
)

print("Simple capacitor controller ready.")
print("Code version:", CODE_VERSION)
print("A port = DC bus, B port = capacitor.")
print("Commands: S, S10, E, E5, H, P")
print("PWM convention: duty_actual = 65536 - pwm_out.")
print("Capacitor pack: 2 parallel caps, each 0.25F and 4ohm ESR.")
print(
    "Cap limits use ESR-corrected Vcap: {:.3f}V to {:.3f}V, ESR={:.3f}ohm.".format(
        V_CAP_MIN,
        V_CAP_MAX,
        CAP_ESR_OHMS
    )
)
print(
    "Equivalent C={:.3f}F, usable window={:.3f}J.".format(
        C_FARADS,
        energy_at_voltage(V_CAP_MAX) - energy_at_voltage(V_CAP_MIN)
    )
)
print(
    "CHARGE: S or S10, target {:.3f}A, voltage target {:.3f}V, current limit {:.3f}A.".format(
        I_CHARGE_TARGET,
        V_CAP_MAX,
        I_PRECHARGE_LIMIT
    )
)
print(
    "CHARGE protection: stop and PWM off if IL <= {:.3f}A.".format(
        I_CHARGE_REVERSE_STOP
    )
)
print(
    "CHARGE protection: stop if dE <= -{:.3f}J.".format(
        CHARGE_ENERGY_LOSS_STOP_J
    )
)
print(
    "DISCHARGE: E or E5, target {:.3f}A, stop at {:.3f}V, current limit {:.3f}A.".format(
        I_DISCHARGE_TARGET,
        V_CAP_MIN,
        I_DISCHARGE_LIMIT
    )
)


# ============================================================
# Main Loop
# ============================================================

while True:

    if not timer_elapsed:
        continue

    timer_elapsed = 0

    try:
        vbus = read_vbus()
        vterm = read_vterm()
        il = ina.vshunt() / SHUNT_OHMS
        vcap = correct_vcap_for_esr(vterm, il)

    except Exception as e:
        do_trip("sensor error: " + str(e))
        continue

    energy = energy_from_vcap(vcap)

    last_vbus = vbus
    last_vterm = vterm
    last_vcap = vcap
    last_il = il
    last_energy = energy

    read_cmd()

    if command == "H":
        command = ""
        do_stop()

    elif command == "P":
        command = ""
        print_status()

    elif command == "S":
        command = ""
        start_charge(command_energy_j)

    elif command == "E":
        command = ""
        start_discharge(command_energy_j)

    if not trip and mode == "CHARGE":

        if abs(il) >= I_HARD_LIMIT:
            do_trip("hard overcurrent during charge {:.3f}A".format(il))
            continue

        if vterm >= V_TERM_HARD_MAX:
            do_trip("terminal overvoltage during charge {:.3f}V".format(vterm))
            continue

        if target_energy_j <= 0.0 and vcap >= V_CAP_MAX:
            print(
                "CHARGE done: Vcap={:.3f}V E={:.3f}J. PWM off.".format(
                    vcap,
                    energy
                )
            )
            mode = "STOPPED"
            pwm_off()
            continue

        if energy - start_energy_j <= -CHARGE_ENERGY_LOSS_STOP_J:
            print(
                "CHARGE stopped: capacitor energy is falling, dE={:.3f}J. "
                "PWM off.".format(
                    energy - start_energy_j
                )
            )
            mode = "STOPPED"
            pwm_off()
            continue

        if target_energy_j > 0.0 and energy - start_energy_j >= target_energy_j:
            print(
                "CHARGE target done: dE={:.3f}J Vcap={:.3f}V. PWM off.".format(
                    energy - start_energy_j,
                    vcap
                )
            )
            mode = "STOPPED"
            pwm_off()
            continue

        if vcap >= V_CAP_MAX:
            print(
                "CHARGE stopped: Vcap={:.3f}V reached limit {:.3f}V. "
                "dE={:.3f}J target={:.3f}J. PWM off.".format(
                    vcap,
                    V_CAP_MAX,
                    energy - start_energy_j,
                    target_energy_j
                )
            )
            mode = "STOPPED"
            pwm_off()
            continue

        if il >= I_PRECHARGE_LIMIT:
            next_pwm = last_pwm_out + CHARGE_STEP_UP
            write_pwm_out(next_pwm)
            print(
                "Charge current too high: IL={:.3f}A, backing off pwm_out={}.".format(
                    il,
                    last_pwm_out
                )
            )
            continue

        if il <= I_CHARGE_REVERSE_STOP:
            mode = "STOPPED"
            pwm_off()
            print(
                "CHARGE stopped: current went negative IL={:.3f}A. "
                "This PWM direction discharges the capacitor, so PWM is off.".format(
                    il
                )
            )
            continue

        if il < I_CHARGE_TARGET:
            next_pwm = last_pwm_out - CHARGE_STEP_DOWN
            write_pwm_out(max(next_pwm, CHARGE_MIN_PWM_OUT))

        else:
            next_pwm = last_pwm_out + CHARGE_STEP_UP
            write_pwm_out(min(next_pwm, CHARGE_START_PWM_OUT))

    if not trip and mode == "DISCHARGE":

        if abs(il) >= I_HARD_LIMIT:
            do_trip("hard overcurrent during discharge {:.3f}A".format(il))
            continue

        if vterm <= V_TERM_HARD_MIN:
            do_trip("terminal undervoltage during discharge {:.3f}V".format(vterm))
            continue

        discharged_j = start_energy_j - energy

        if discharged_j >= target_energy_j:
            print(
                "DISCHARGE done: dE=-{:.3f}J Vcap={:.3f}V. PWM off.".format(
                    discharged_j,
                    vcap
                )
            )
            mode = "STOPPED"
            pwm_off()
            continue

        if vcap <= V_CAP_MIN:
            print(
                "DISCHARGE stopped: Vcap={:.3f}V reached limit {:.3f}V. "
                "dE=-{:.3f}J target=-{:.3f}J. PWM off.".format(
                    vcap,
                    V_CAP_MIN,
                    discharged_j,
                    target_energy_j
                )
            )
            mode = "STOPPED"
            pwm_off()
            continue

        if il <= I_DISCHARGE_LIMIT:
            next_pwm = last_pwm_out + DISCHARGE_STEP_UP
            write_pwm_out(next_pwm)
            print(
                "Discharge current too high: IL={:.3f}A, backing off pwm_out={}.".format(
                    il,
                    last_pwm_out
                )
            )
            continue

        if il > I_DISCHARGE_TARGET:
            next_pwm = last_pwm_out - DISCHARGE_STEP_DOWN
            write_pwm_out(max(next_pwm, DISCHARGE_MIN_PWM_OUT))

        else:
            next_pwm = last_pwm_out + DISCHARGE_STEP_UP
            write_pwm_out(min(next_pwm, DISCHARGE_START_PWM_OUT))

    now_ms = time.ticks_ms()
    if time.ticks_diff(now_ms, last_status_ms) >= STATUS_INTERVAL_MS:
        last_status_ms = now_ms
        print_status()

