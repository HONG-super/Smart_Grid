from machine import Pin, I2C, ADC, PWM
import utime

# ============================================================
# Cascaded Control MPPT: Outer P&O + Inner PI (Filtered)
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
PV_ON_PORT_A = False
IPV_SIGN = -1

# =========================
# Control Loop Settings
# =========================
DUTY_MIN = 1000
DUTY_MAX = 40000
duty = 18000

# Inner PI voltage loop
KP = 1000.0
KI = 20.0
PI_PERIOD_MS = 5

INT_MAX = 30000
INT_MIN = -30000
mppt_err_int = 0.0

# Software Low-Pass Filter
FILTER_ALPHA = 0.15
vpv_filtered = 0.0

# Outer P&O loop
VREF_STEP = 0.05
VREF_MIN = 2.0
VREF_MAX = 9.0
MPPT_PERIOD_MS = 100
v_ref = 6.0

# Safety limits
VPV_MIN_SAFE = 2.0
VPV_MAX_SAFE = 9.5
IPV_MAX_SAFE = 1.20
VBUS_MAX_SAFE = 11.5

# Test duration
TEST_DURATION_MS = 60000

# Logging
SAVE_RESULTS = True
RESULTS_FILE = "externalGridIntegrate_2.csv"
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
# Helpers
# ============================================================

def saturate(x, upper, lower):
    if x > upper:
        return upper
    if x < lower:
        return lower
    return x


def read_power(ina):
    va = 1.017 * (12490 / 2490) * 3.3 * (va_pin.read_u16() / 65536)
    vb = 1.015 * (12490 / 2490) * 3.3 * (vb_pin.read_u16() / 65536)
    iL = ina.vshunt() / SHUNT_OHMS

    if PV_ON_PORT_A:
        vpv, vbus = va, vb
    else:
        vpv, vbus = vb, va

    ipv = IPV_SIGN * iL
    ppv = vpv * ipv if ipv > 0 else 0.0

    return va, vb, vpv, ipv, ppv, vbus, iL


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
    results.write("time_ms,Vpv_raw,Vpv_filt,Ipv,Ppv,Vref,duty,unsafe\n")
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
unsafe = False

if vpv_filtered == 0.0:
    vpv_filtered = vpv

print("=======================================")
print("FILTERED CASCADED P&O + PI MPPT TEST")
print("CSV file =", RESULTS_FILE)
print("LOG_PERIOD_MS =", LOG_PERIOD_MS)
print("LOG_FLUSH_PERIOD_MS =", LOG_FLUSH_PERIOD_MS)
print("PRINT_PERIOD_MS =", PRINT_PERIOD_MS)
print("=======================================")

# ============================================================
# Main loop
# ============================================================

while True:
    now_ms = utime.ticks_ms()
    elapsed_ms = utime.ticks_diff(now_ms, start_ms)

    if elapsed_ms >= TEST_DURATION_MS:
        pwm.duty_u16(DUTY_MIN)
        if SAVE_RESULTS:
            results.flush()
            results.close()
        print("Test finished. PWM set to minimum.")
        break

    # --------------------------------------------------------
    # INNER LOOP: Fast PI Voltage Control
    # --------------------------------------------------------
    if utime.ticks_diff(now_ms, last_pi_ms) >= PI_PERIOD_MS:
        last_pi_ms = now_ms
        va, vb, vpv, ipv, ppv, vbus, iL = read_power(ina)

        unsafe = (
            vpv < VPV_MIN_SAFE or
            vpv > VPV_MAX_SAFE or
            ipv > IPV_MAX_SAFE or
            vbus > VBUS_MAX_SAFE
        )

        if unsafe:
            duty = int(saturate(duty - 500, DUTY_MAX, DUTY_MIN))
            pwm.duty_u16(duty)
            mppt_err_int = 0.0
        else:
            vpv_filtered = (
                FILTER_ALPHA * vpv
                + (1.0 - FILTER_ALPHA) * vpv_filtered
            )

            # Increasing duty is assumed to pull Vpv DOWN.
            v_err = vpv_filtered - v_ref

            mppt_err_int = saturate(
                mppt_err_int + v_err,
                INT_MAX,
                INT_MIN
            )

            pi_out = (KP * v_err) + (KI * mppt_err_int)

            duty = int(saturate(duty + pi_out, DUTY_MAX, DUTY_MIN))
            pwm.duty_u16(duty)

    # --------------------------------------------------------
    # OUTER LOOP: Slow P&O
    # --------------------------------------------------------
    if utime.ticks_diff(now_ms, last_mppt_ms) >= MPPT_PERIOD_MS:
        last_mppt_ms = now_ms

        if not unsafe:
            if first_mppt:
                p_prev = ppv
                v_prev = vpv_filtered
                first_mppt = False
            else:
                delta_p = ppv - p_prev
                delta_v = vpv_filtered - v_prev

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

                p_prev = ppv
                v_prev = vpv_filtered

    # --------------------------------------------------------
    # Logging & Printing
    # --------------------------------------------------------
    if SAVE_RESULTS and utime.ticks_diff(now_ms, last_log_ms) >= LOG_PERIOD_MS:
        last_log_ms = now_ms

        results.write(
            "{},{:.3f},{:.3f},{:.5f},{:.5f},{:.3f},{},{}\n".format(
                elapsed_ms,
                vpv,
                vpv_filtered,
                ipv,
                ppv,
                v_ref,
                duty,
                int(unsafe)
            )
        )

    if SAVE_RESULTS and utime.ticks_diff(now_ms, last_flush_ms) >= LOG_FLUSH_PERIOD_MS:
        last_flush_ms = now_ms
        results.flush()

    if utime.ticks_diff(now_ms, last_print_ms) >= PRINT_PERIOD_MS:
        last_print_ms = now_ms

        print(
            "t={}ms | Vraw={:.2f}V | Vfilt={:.2f}V | Vref={:.2f}V | Ppv={:.2f}W | duty={}".format(
                elapsed_ms,
                vpv,
                vpv_filtered,
                v_ref,
                ppv,
                duty
            )
        )
