# Simple Supercapacitor Controller
# A port = DC bus, B port = capacitor
# This file is intentionally small: safe low-voltage precharge first.
#
# Commands:
#   S        precharge to PRECHARGE_TARGET_V
#   S10      precharge, then keep passive charging until +10 J if possible
#   H        stop, PWM off
#   P        print one status line
#
# PWM convention follows the provided bidirectional code:
#   pwm_out      = controller-side value
#   duty_actual  = 65536 - pwm_out
#
# Important:
#   pwm_off() uses pwm.duty_u16(0), not duty_actual = 0.
#   This is the condition that your log showed naturally precharging the cap.

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


# ============================================================
# Parameters
# ============================================================

V_BUS_MIN = 8.0
V_CAP_ABSOLUTE_MIN = 0.8
PRECHARGE_TARGET_V = 8.0
PASSIVE_STOP_MARGIN_V = 0.25

C_FARADS = 1.5
SHUNT_OHMS = 0.10
CAP_ESR_OHMS = 0.30

MIN_PWM_OUT = 0
MAX_PWM_OUT = 64536

I_PRECHARGE_LIMIT = 0.45
I_REVERSE_LIMIT = -0.08
I_HARD_LIMIT = 1.20

DEFAULT_CHARGE_J = 0.0
STATUS_INTERVAL_MS = 1000


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


def parse_energy_amount(text):
    text = text.strip().upper()

    if not text:
        return DEFAULT_CHARGE_J

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
    print(
        "mode={} trip={} Vbus={:.3f}V Vterm={:.3f}V Vcap={:.3f}V "
        "IL={:.3f}A E={:.3f}J dE={:.3f}J targetE={:.3f}J "
        "pwm_out={} duty_actual={}".format(
            mode,
            trip,
            last_vbus,
            last_vterm,
            last_vcap,
            last_il,
            last_energy,
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


def start_precharge(request_j):
    global mode, target_energy_j, start_energy_j

    if trip:
        print("Trip active. Send H first.")
        return

    if last_vcap < V_CAP_ABSOLUTE_MIN:
        print(
            "Start blocked: Vcap={:.3f}V below {:.3f}V. Check sensing/wiring.".format(
                last_vcap,
                V_CAP_ABSOLUTE_MIN
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

    pwm_off()
    start_energy_j = last_energy
    target_energy_j = request_j
    mode = "PRECHARGE"

    print(
        "PRECHARGE started: PWM off, Vcap={:.3f}V -> {:.3f}V, "
        "followup target +{:.3f}J if passive charging can reach it.".format(
            last_vcap,
            PRECHARGE_TARGET_V,
            target_energy_j
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
                amount = parse_energy_amount(line[1:])

                if amount is None:
                    print("Invalid command. Use S, S10, or S10J.")
                    return

                command_energy_j = amount
                command = "S"

            elif cmd in "HP":
                command = cmd

            else:
                print("Unknown command. Use S, H, or P.")

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
print("A port = DC bus, B port = capacitor.")
print("Commands: S, S10, H, P")
print("PWM convention: duty_actual = 65536 - pwm_out, but PRECHARGE uses PWM off.")
print(
    "PRECHARGE: PWM off until Vcap >= {:.3f}V; current limit {:.3f}A.".format(
        PRECHARGE_TARGET_V,
        I_PRECHARGE_LIMIT
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
        start_precharge(command_energy_j)

    if not trip and mode == "PRECHARGE":

        # PRECHARGE deliberately keeps PWM off. Your observed hardware already
        # charges the capacitor gently in this state.
        pwm_off()

        if abs(il) >= I_HARD_LIMIT:
            do_trip("hard overcurrent during precharge {:.3f}A".format(il))
            continue

        if abs(il) >= I_PRECHARGE_LIMIT:
            do_trip("precharge current {:.3f}A".format(il))
            continue

        if il <= I_REVERSE_LIMIT:
            do_trip("reverse current during precharge {:.3f}A".format(il))
            continue

        if vcap >= PRECHARGE_TARGET_V:
            if target_energy_j <= 0.0:
                print(
                    "PRECHARGE done: Vcap={:.3f}V E={:.3f}J. PWM off.".format(
                        vcap,
                        energy
                    )
                )
                mode = "STOPPED"
                pwm_off()

            elif energy - start_energy_j >= target_energy_j:
                print(
                    "Passive charge target done: dE={:.3f}J Vcap={:.3f}V. PWM off.".format(
                        energy - start_energy_j,
                        vcap
                    )
                )
                mode = "STOPPED"
                pwm_off()

            elif vcap >= vbus - PASSIVE_STOP_MARGIN_V:
                print(
                    "Passive charge cannot safely continue: Vcap={:.3f}V near Vbus={:.3f}V. "
                    "dE={:.3f}J target={:.3f}J. PWM off.".format(
                        vcap,
                        vbus,
                        energy - start_energy_j,
                        target_energy_j
                    )
                )
                mode = "STOPPED"
                pwm_off()

    now_ms = time.ticks_ms()
    if time.ticks_diff(now_ms, last_status_ms) >= STATUS_INTERVAL_MS:
        last_status_ms = now_ms
        print_status()

