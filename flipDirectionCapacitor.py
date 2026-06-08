from machine import Pin, I2C, ADC, PWM, Timer
import sys
import select

# =========================
# Capacitor Constants
# =========================
CAPACITOR_MAX_VOLTAGE = 18
CAPACITANCE = 0.25
ESR = 4
CAPACITOR_MAX_CURRENT = 0.35

MAX_ENERGY = 0.5 * CAPACITANCE * CAPACITOR_MAX_VOLTAGE**2
ENERGY_THRESHOLD = 5

DISCHARGE_CURRENT = -0.2
CHARGE_CURRENT = 0.2

# =========================
# Pins
# =========================
va_pin = ADC(Pin(28))
vb_pin = ADC(Pin(26))

OL_CL_pin = Pin(12, Pin.IN, Pin.PULL_UP)
BU_BO_pin = Pin(2, Pin.OUT)

ina_i2c = I2C(0, scl=Pin(1), sda=Pin(0), freq=400000)

pwm = PWM(Pin(9))
pwm.freq(100000)

min_pwm = 0
max_pwm = 64536
pwm.duty_u16(0)

# =========================
# Controller Parameters
# =========================
SHUNT_OHMS = 0.10

kp = 100
ki = 300

i_err_int = 0
i_ref = 0

timer_elapsed = 0
count = 0
first_run = 1

mode = "HOLD"
target_energy = 0


def saturate(signal, upper, lower):
    if signal > upper:
        return upper
    if signal < lower:
        return lower
    return signal


def tick(t):
    global timer_elapsed
    timer_elapsed = 1


# =========================
# INA219
# =========================
class ina219:
    REG_CONFIG = 0x00
    REG_SHUNTVOLTAGE = 0x01
    REG_BUSVOLTAGE = 0x02
    REG_POWER = 0x03
    REG_CURRENT = 0x04
    REG_CALIBRATION = 0x05

    def __init__(self, sr, address, maxi):
        self.address = address
        self.shunt = sr

    def vshunt(self):
        reg_bytes = ina_i2c.readfrom_mem(self.address, self.REG_SHUNTVOLTAGE, 2)
        reg_value = int.from_bytes(reg_bytes, 'big')

        if reg_value > 2**15:
            sign = -1
            reg_value = (~reg_value + 1) & 0xFFFF
        else:
            sign = 1

        return float(reg_value) * 1e-5 * sign

    def vbus(self):
        reg_bytes = ina_i2c.readfrom_mem(self.address, self.REG_BUSVOLTAGE, 2)
        reg_value = int.from_bytes(reg_bytes, 'big') >> 3
        return float(reg_value) * 0.004

    def configure(self):
        ina_i2c.writeto_mem(self.address, self.REG_CONFIG, b'\x19\x9F')
        ina_i2c.writeto_mem(self.address, self.REG_CALIBRATION, b'\x00\x00')


# =========================
# USB Command Reader
# =========================
poll = select.poll()
poll.register(sys.stdin, select.POLLIN)


def read_command(current_energy):
    global mode, target_energy, i_err_int

    if poll.poll(0):
        line = sys.stdin.readline().strip().upper()
        parts = line.split()

        if len(parts) == 0:
            return

        cmd = parts[0]

        if cmd == "C":
            if len(parts) < 2:
                print("Use: C 10")
                return

            delta_E = float(parts[1])
            target_energy = current_energy + delta_E
            target_energy = min(target_energy, MAX_ENERGY - ENERGY_THRESHOLD)

            mode = "CHARGE"
            i_err_int = 0

            print("CHARGE {:.2f} J, target = {:.2f} J".format(delta_E, target_energy))

        elif cmd == "D":
            if len(parts) < 2:
                print("Use: D 10")
                return

            delta_E = float(parts[1])
            target_energy = current_energy - delta_E
            target_energy = max(target_energy, ENERGY_THRESHOLD)

            mode = "DISCHARGE"
            i_err_int = 0

            print("DISCHARGE {:.2f} J, target = {:.2f} J".format(delta_E, target_energy))

        elif cmd == "H":
            mode = "HOLD"
            target_energy = current_energy
            i_err_int = 0
            pwm.duty_u16(0)

            print("HOLD")

        else:
            print("Unknown command. Use C 10, D 10, or H.")


def manual_algo(current_energy):
    global mode

    if mode == "CHARGE":
        if current_energy >= target_energy:
            mode = "HOLD"
            pwm.duty_u16(0)
            print("CHARGE DONE. E = {:.2f} J".format(current_energy))
            return 0, 0
        return 0, CHARGE_CURRENT

    elif mode == "DISCHARGE":
        if current_energy <= target_energy:
            mode = "HOLD"
            pwm.duty_u16(0)
            print("DISCHARGE DONE. E = {:.2f} J".format(current_energy))
            return 1, 0
        return 1, DISCHARGE_CURRENT

    else:
        return 0, 0


# =========================
# Main Loop
# =========================
ina = ina219(SHUNT_OHMS, 64, 5)
ina.configure()

loop_timer = Timer(mode=Timer.PERIODIC, freq=1000, callback=tick)

print("Manual capacitor control ready.")
print("Commands:")
print("C 10  -> charge/store 10 J")
print("D 10  -> discharge/extract 10 J")
print("H     -> hold")

while True:
    if timer_elapsed == 1:

        va = 1.017 * (12490 / 2490) * 3.3 * (va_pin.read_u16() / 65536)
        vb = 1.015 * (12490 / 2490) * 3.3 * (vb_pin.read_u16() / 65536)

        capacitor_energy = 0.5 * CAPACITANCE * (vb ** 2)

        read_command(capacitor_energy)

        if capacitor_energy <= ENERGY_THRESHOLD:
            BU = 0
            i_ref = CHARGE_CURRENT

        elif capacitor_energy >= MAX_ENERGY - ENERGY_THRESHOLD:
            BU = 1
            i_ref = DISCHARGE_CURRENT

        else:
            BU, i_ref = manual_algo(capacitor_energy)

        BU_BO_pin.value(BU)

        i_ref = saturate(i_ref, CAPACITOR_MAX_CURRENT, -CAPACITOR_MAX_CURRENT)

        Vshunt = ina.vshunt()
        iL = Vshunt / SHUNT_OHMS

        if abs(iL) > CAPACITOR_MAX_CURRENT:
            pwm.duty_u16(0)
            mode = "HOLD"
            i_err_int = 0
            print("Overcurrent detected! iL = {:.3f} A".format(iL))
            timer_elapsed = 0
            continue

        if mode == "HOLD":
            pwm.duty_u16(0)
        else:
            i_err = i_ref - iL
            i_err_int = i_err_int + i_err
            i_err_int = saturate(i_err_int, 10000, -10000)

            i_pi_out = (kp * i_err) + (ki * i_err_int)
            pwm_out = saturate(i_pi_out, max_pwm, min_pwm)

            duty = int(65536 - pwm_out)
            duty = int(saturate(duty, 65535, 0))

            pwm.duty_u16(duty)

        count += 1
        timer_elapsed = 0

        if count > 100:
            print(
                "Mode={} Va={:.2f} Vb={:.2f} E={:.2f} J target={:.2f} iL={:.3f} i_ref={:.3f}".format(
                    mode, va, vb, capacitor_energy, target_energy, iL, i_ref
                )
            )
            count = 0
