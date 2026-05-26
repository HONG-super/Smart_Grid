import time
import requests
import serial

# =========================
# Web server
# =========================
API_URL = "https://icelec50015.azurewebsites.net"

# =========================
# CPX400SP USB settings
# =========================
PORT = "COM5"
BAUDRATE = 9600

# =========================
# Temperature model
# =========================
T_AMB = 25.0
T_REF = 25.0
DELTA_T_MAX = 8.0
TAU = 180.0
BETA_V = -0.024

# =========================
# Irradiance-dependent PV parameters
# s, Voc_base, Isc, B, C
# =========================
PV_TABLE = [
    (0.00, 0.00, 0.000, 3.95, 1.40),
    (0.25, 7.05, 0.262, 3.95, 1.40),
    (0.50, 7.55, 0.505, 3.65, 1.65),
    (1.00, 7.86, 0.920, 3.00, 2.50),
]

UPDATE_INTERVAL = 0.5

# Safety limits
MAX_ALLOWED_VOLTAGE = 8.5
MAX_ALLOWED_CURRENT = 1.0

# Smooth voltage changes
MAX_VOLTAGE_STEP = 0.25


def get_sun():
    response = requests.get(API_URL + "/sun", timeout=3)
    response.raise_for_status()
    data = response.json()

    tick = data["tick"]
    sun = data["sun"]

    return tick, sun


def open_psu():
    return serial.Serial(
        port=PORT,
        baudrate=BAUDRATE,
        bytesize=8,
        parity="N",
        stopbits=1,
        timeout=2
    )


def send_command(psu, command):
    psu.write((command + "\n").encode())


def ask(psu, command):
    psu.write((command + "\n").encode())
    return psu.readline().decode().strip()


def extract_number(reply):
    number_text = ""

    for character in reply:
        if character.isdigit() or character == "." or character == "-":
            number_text += character
        elif number_text != "":
            break

    if number_text == "":
        raise ValueError("Could not extract number from reply: " + reply)

    return float(number_text)


def read_vout(psu):
    return extract_number(ask(psu, "V1O?"))


def read_iout(psu):
    return extract_number(ask(psu, "I1O?"))


def clip(x, low, high):
    return max(low, min(x, high))


def interpolate(x, x0, y0, x1, y1):
    if x1 == x0:
        return y0

    ratio = (x - x0) / (x1 - x0)
    return y0 + ratio * (y1 - y0)


def calculate_s_from_sun(sun):
    return clip(sun / 100.0, 0.0, 1.0)


def get_pv_parameters(s):
    s = clip(s, 0.0, 1.0)

    for i in range(len(PV_TABLE) - 1):
        s0, voc0, isc0, b0, c0 = PV_TABLE[i]
        s1, voc1, isc1, b1, c1 = PV_TABLE[i + 1]

        if s0 <= s <= s1:
            voc_base = interpolate(s, s0, voc0, s1, voc1)
            isc = interpolate(s, s0, isc0, s1, isc1)
            b_shape = interpolate(s, s0, b0, s1, b1)
            c_shape = interpolate(s, s0, c0, s1, c1)

            return voc_base, isc, b_shape, c_shape

    return PV_TABLE[-1][1], PV_TABLE[-1][2], PV_TABLE[-1][3], PV_TABLE[-1][4]


def update_t_cell(t_cell, s, dt):
    target_temperature = T_AMB + DELTA_T_MAX * s
    dT_cell_dt = (target_temperature - t_cell) / TAU
    return t_cell + dT_cell_dt * dt


def calculate_voc_with_temperature(voc_base, t_cell):
    voc = voc_base + BETA_V * (t_cell - T_REF)
    return clip(voc, 0.0, MAX_ALLOWED_VOLTAGE)


def calculate_v_target_from_iout(iout, isc, voc, b_shape, c_shape):
    if isc <= 0.0 or voc <= 0.0:
        return 0.0

    x = clip(iout / isc, 0.0, 1.0)

    v_target = voc * ((1.0 - x ** b_shape) ** c_shape)

    return clip(v_target, 0.0, voc)


def set_psu_voltage_current(psu, voltage_set, current_limit):
    send_command(psu, f"V1 {voltage_set:.3f}")
    send_command(psu, f"I1 {current_limit:.3f}")
    send_command(psu, "OP1 1")


psu = open_psu()

t_cell = T_AMB
last_time = time.time()
voltage_set = 0.0

try:
    send_command(psu, "OP1 0")
    time.sleep(0.2)

    send_command(psu, "V1 0.000")
    send_command(psu, "I1 0.050")
    send_command(psu, "OP1 1")
    time.sleep(0.5)

    while True:
        now = time.time()
        dt = now - last_time
        last_time = now

        tick, sun = get_sun()
        s = calculate_s_from_sun(sun)

        t_cell = update_t_cell(t_cell, s, dt)

        voc_base, isc, b_shape, c_shape = get_pv_parameters(s)
        voc = calculate_voc_with_temperature(voc_base, t_cell)

        vout = read_vout(psu)
        iout = read_iout(psu)

        v_target = calculate_v_target_from_iout(
            iout,
            isc,
            voc,
            b_shape,
            c_shape
        )

        voltage_step = v_target - voltage_set
        voltage_step = clip(voltage_step, -MAX_VOLTAGE_STEP, MAX_VOLTAGE_STEP)

        voltage_set = voltage_set + voltage_step
        voltage_set = clip(voltage_set, 0.0, voc)

        current_limit = clip(isc, 0.0, MAX_ALLOWED_CURRENT)

        set_psu_voltage_current(psu, voltage_set, current_limit)

        print(
            f"tick={tick:02d} | "
            f"sun={sun:6.2f} | "
            f"s={s:5.3f} | "
            f"Tcell={t_cell:6.2f} C | "
            f"Voc={voc:6.3f} V | "
            f"Isc={isc:6.3f} A | "
            f"B={b_shape:5.2f} | "
            f"C={c_shape:5.2f} | "
            f"Iout={iout:6.3f} A | "
            f"Vtarget={v_target:6.3f} V | "
            f"Vset={voltage_set:6.3f} V | "
            f"Vout={vout:6.3f} V | "
            f"Ilimit={current_limit:6.3f} A"
        )

        time.sleep(UPDATE_INTERVAL)

except KeyboardInterrupt:
    send_command(psu, "OP1 0")
    psu.close()
    print("Stopped. PSU output off.")