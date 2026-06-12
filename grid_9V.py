# The main difference is we solve the problem in a more complex way at first
# In this method we only use PI control and get rid of the CL/OL
# we also change the way of calculating the duty cycle in previous version we can only let the PWM change between a small range around the PWM_base while in this case we use a inverse PWM 
# and duty = 65536 - PI_output and the intergal increases very fast resulting the settling time is small
# Less is more

from machine import Pin, I2C, ADC, PWM, Timer
import utime

# Save test results to CSV
SAVE_RESULTS = True
RESULTS_FILE = "external_grid_10V_nopv_2.csv"
LOG_DURATION_MS = 60000        # record for 30 seconds
LOG_PERIOD_MS = 10             # write one row every 10 ms
LOG_FLUSH_PERIOD_MS = 1000     # flush every 1 second

# ============================================================
# Hardware setup
# ============================================================

va_pin = ADC(Pin(28))
vb_pin = ADC(Pin(26))

ina_i2c = I2C(0, scl=Pin(1), sda=Pin(0), freq=2400000)

pwm = PWM(Pin(9))
pwm.freq(100000)

min_pwm = 1000
max_pwm = 64536
pwm_out = min_pwm

# ============================================================
# Controller settings
# ============================================================

kp = 150
ki = 300

# Changed from 7.05 V to 10.00 V
v_ref = 10.00

v_err = 0.0
v_err_int = 0.0
v_pi_out = 0.0

# Integral limit
V_ERR_INT_LIMIT = 10000

# Shunt resistance
SHUNT_OHMS = 0.10

# Timer variables
timer_elapsed = 0
count = 0
first_run = 1

# Logging variables
results = None;
log_start_ms = 0
last_log_ms = 0
last_flush_ms = 0
log_done = False

# ============================================================
# Helper functions
# ============================================================

def saturate(signal, upper, lower):
    if signal > upper:
        signal = upper
    if signal < lower:
        signal = lower
    return signal



def tick(t):
    global timer_elapsed
    timer_elapsed = 1


# ============================================================
# INA219 current sensor
# ============================================================

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
        reg_value = int.from_bytes(reg_bytes, "big")

        # Signed 16-bit conversion
        if reg_value >= 0x8000:
            reg_value = reg_value - 0x10000

        # INA219 shunt voltage LSB = 10 uV
        return float(reg_value) * 1e-5

    def vbus(self):
        reg_bytes = ina_i2c.readfrom_mem(self.address, self.REG_BUSVOLTAGE, 2)
        reg_value = int.from_bytes(reg_bytes, "big") >> 3
        return float(reg_value) * 0.004

    def configure(self):
        # PG = /8
        ina_i2c.writeto_mem(self.address, self.REG_CONFIG, b"\x19\x9F")

        # Calibration disabled because current is calculated manually
        ina_i2c.writeto_mem(self.address, self.REG_CALIBRATION, b"\x00\x00")


# ============================================================
# Main loop
# ============================================================

while True:

    if first_run:
        ina = ina219(SHUNT_OHMS, 64, 5)
        ina.configure()

        if SAVE_RESULTS:
            results = open(RESULTS_FILE, "w")
            results.write(
                "time_ms,Va,Vb,iL,power,import_power,export_power,"
                "duty,pwm_out,v_ref,v_err,v_err_int,v_pi_out\n"
            )
            results.flush()

        log_start_ms = utime.ticks_ms()
        last_log_ms = log_start_ms
        last_flush_ms = log_start_ms

        first_run = 0

        loop_timer = Timer(mode=Timer.PERIODIC, freq=1000, callback=tick)

        print("================================")
        print("RUNNING EXTERNAL GRID CODE WITHOUT WIFI/MQTT")
        print("CSV logging =", SAVE_RESULTS)
        print("CSV file =", RESULTS_FILE)
        print("LOG_DURATION_MS =", LOG_DURATION_MS)
        print("Target Vb v_ref =", v_ref)
        print("kp =", kp)
        print("ki =", ki)
        print("================================")

    if timer_elapsed == 1:

        # ------------------------------------------------------------
        # Measurements
        # ------------------------------------------------------------

        va = 1.017 * (12490 / 2490) * 3.3 * (va_pin.read_u16() / 65536)
        vb = 1.015 * (12490 / 2490) * 3.3 * (vb_pin.read_u16() / 65536)

        Vshunt = ina.vshunt()
        iL = Vshunt / SHUNT_OHMS

        power = vb * iL

        if power < 0:
            export_power = -power
            import_power = 0
        else:
            export_power = 0
            import_power = power

        # ------------------------------------------------------------
        # PI voltage controller
        # ------------------------------------------------------------

        v_err = v_ref - vb

        v_err_int = v_err_int + v_err
        v_err_int = saturate(v_err_int, V_ERR_INT_LIMIT, -V_ERR_INT_LIMIT)

        v_pi_out = (kp * v_err) + (ki * v_err_int)

        min_pwm = 0
        max_pwm = 64536

        pwm_out = saturate(v_pi_out, max_pwm, min_pwm)

        # Original code uses inverted PWM
        duty = int(65536 - pwm_out)

        pwm.duty_u16(duty)

        # ------------------------------------------------------------
        # CSV logging
        # ------------------------------------------------------------

        if SAVE_RESULTS and (not log_done):
            now_ms = utime.ticks_ms()
            elapsed_ms = utime.ticks_diff(now_ms, log_start_ms)

            if utime.ticks_diff(now_ms, last_log_ms) >= LOG_PERIOD_MS:
                last_log_ms = now_ms

                results.write(
                    "{},{:.3f},{:.3f},{:.4f},{:.4f},{:.4f},{:.4f},"
                    "{},{:.3f},{:.3f},{:.4f},{:.4f},{:.3f}\n".format(
                        elapsed_ms,
                        va,
                        vb,
                        iL,
                        power,
                        import_power,
                        export_power,
                        duty,
                        pwm_out,
                        v_ref,
                        v_err,
                        v_err_int,
                        v_pi_out
                    )
                )

            if utime.ticks_diff(now_ms, last_flush_ms) >= LOG_FLUSH_PERIOD_MS:
                last_flush_ms = now_ms
                results.flush()

            if elapsed_ms >= LOG_DURATION_MS:
                results.flush()
                results.close()
                log_done = True
                print("CSV logging finished. File saved as:", RESULTS_FILE)

        # ------------------------------------------------------------
        # Slow serial print
        # ------------------------------------------------------------

        count = count + 1
        timer_elapsed = 0

        if count > 1000:
            count = 0

            print("Va = {:.3f}".format(va))
            print("Vb = {:.3f}".format(vb))
            print("iL = {:.3f}".format(iL))
            print("Power = {:.3f}".format(power))
            print("Import power = {:.3f}".format(import_power))
            print("Export power = {:.3f}".format(export_power))
            print("v_ref = {:.3f}".format(v_ref))
            print("v_err = {:.4f}".format(v_err))
            print("v_err_int = {:.4f}".format(v_err_int))
            print("v_pi_out = {:.3f}".format(v_pi_out))
            print("pwm_out = {:.3f}".format(pwm_out))
            print("duty cycle = {:d}".format(duty))
            print("logging_done =", log_done)
            print("----------------------")
