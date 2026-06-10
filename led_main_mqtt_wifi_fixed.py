from machine import Pin, PWM, Timer, SPI
from PID import PID
import time
import sys
import select

# MQTT/WiFi imports
try:
    import network
except Exception:
    network = None

try:
    import ujson as json
except Exception:
    import json

MQTT_IMPORT_ERROR = None
try:
    from umqtt.simple import MQTTClient
except Exception as error:
    MQTTClient = None
    MQTT_IMPORT_ERROR = error


# =========================
# WiFi / MQTT settings
# =========================
WIFI_SSID = "Hong"
WIFI_PASSWORD = "abcdefgh"

MQTT_BROKER = "172.20.10.4"
MQTT_PORT = 1884
MQTT_CLIENT_ID = "team01_led_load"

TOPIC_LOAD_COMMAND = b"smartgrid/team01/load/command"
TOPIC_LOAD_TELEMETRY = b"smartgrid/team01/load/telemetry"
TOPIC_SYSTEM_EMERGENCY = b"smartgrid/team01/system/emergency"
TOPIC_SYSTEM_RESTART = b"smartgrid/team01/system/restart"

MQTT_CHECK_INTERVAL_TICKS = 20       # 20 ms at 1 kHz control loop
MQTT_SERVICE_INTERVAL_TICKS = 200    # 200 ms at 1 kHz control loop
WIFI_RETRY_MS = 5000
MQTT_RETRY_MS = 5000


# =========================
# Original test settings
# =========================
p_target = 1.5
p_sum = 0.0
p_sum_red = 0.0
p_sum_yel = 0.0
p_sum_grn = 0.0

PWM_FREQ = 100000
CONTROL_FREQ = 1000
PRINT_INTERVAL = 100

P_TOTAL_MIN = 0.0
P_TOTAL_MAX = 3.0

PWM_MIN = 0
PWM_MAX = 62500


# Timer
timer_elapsed = 0


def tick(timer):
    global timer_elapsed
    timer_elapsed = 1


# PWM setup
pwm_red = PWM(Pin(11))
pwm_yel = PWM(Pin(9))
pwm_grn = PWM(Pin(7))

pwm_red_en = Pin(10, Pin.OUT)
pwm_yel_en = Pin(8, Pin.OUT)
pwm_grn_en = Pin(6, Pin.OUT)

pwm_red.freq(PWM_FREQ)
pwm_yel.freq(PWM_FREQ)
pwm_grn.freq(PWM_FREQ)

pwm_red.duty_u16(0)
pwm_yel.duty_u16(0)
pwm_grn.duty_u16(0)

pwm_red_en.value(0)
pwm_yel_en.value(0)
pwm_grn_en.value(0)


# ADC setup
spi = SPI(0, baudrate=400000)
adc_cs = Pin(17, mode=Pin.OUT, value=1)


def readadc(channel):
    txdata = bytearray([6 + (channel >> 2), (channel & 3) << 6, 0])
    rxdata = bytearray(len(txdata))

    try:
        adc_cs.value(0)
        time.sleep_us(10)
        spi.write_readinto(txdata, rxdata)
    finally:
        adc_cs.value(1)

    return ((rxdata[1] & 15) << 8) + rxdata[2]


# Serial input setup
serial_poll = select.poll()
serial_poll.register(sys.stdin, select.POLLIN)


def limit_target(value):
    return min(max(value, P_TOTAL_MIN), P_TOTAL_MAX)


def read_serial_target(current_target):
    try:
        events = serial_poll.poll(0)

        if events:
            line = sys.stdin.readline().strip()

            if line.startswith("P,"):
                received_target = float(line.split(",", 1)[1])
                received_target = limit_target(received_target)

                print("ACK_TARGET,{:.3f}".format(received_target))

                return received_target

    except Exception as error:
        print("SERIAL_ERROR,{}".format(error))

    return current_target


# Functions
def saturate(duty):
    return int(min(max(duty, PWM_MIN), PWM_MAX))


def shutdown():
    pwm_red.duty_u16(0)
    pwm_yel.duty_u16(0)
    pwm_grn.duty_u16(0)

    pwm_red_en.value(0)
    pwm_yel_en.value(0)
    pwm_grn_en.value(0)


# =========================
# MQTT state and functions
# =========================
wlan = None
mqtt_client = None
mqtt_connected = False

last_wifi_attempt_ms = 0
last_mqtt_attempt_ms = 0

mqtt_target_pending = None
emergency_enabled = False
load_status = "enabled"


def topic_to_str(topic):
    try:
        return topic.decode()
    except Exception:
        return str(topic)


def payload_to_str(payload):
    try:
        return payload.decode()
    except Exception:
        return str(payload)


def bool_value(value, default=False):
    if value is None:
        return default

    if value is True:
        return True

    if value is False:
        return False

    text = str(value).lower()
    return text in ("1", "true", "yes", "on", "enable", "enabled")


def queue_mqtt_target(value, source):
    global mqtt_target_pending, emergency_enabled, load_status

    try:
        target = limit_target(float(value))
    except Exception as error:
        print("MQTT_TARGET_ERROR,{},{}".format(source, error))
        return

    mqtt_target_pending = target
    emergency_enabled = False
    load_status = "enabled"

    print("MQTT_TARGET,{}, {:.3f}".format(source, target))


def read_mqtt_target(current_target):
    global mqtt_target_pending

    if mqtt_target_pending is None:
        return current_target

    target = mqtt_target_pending
    mqtt_target_pending = None

    print("ACK_TARGET_MQTT,{:.3f}".format(target))

    return target


def mqtt_callback(topic, msg):
    global mqtt_target_pending, emergency_enabled, load_status

    topic_text = topic_to_str(topic)
    msg_text = payload_to_str(msg)

    # Debug: confirms that the Pico actually received this MQTT message.
    print("MQTT_RX,{},{}".format(topic_text, msg_text))

    try:
        data = json.loads(msg_text)
    except Exception:
        # Raw text payload, for example: 0 or 1.5
        try:
            queue_mqtt_target(float(msg_text), "raw_text")
            return
        except Exception:
            print("MQTT_RX_BAD_JSON,{},{}".format(topic_text, msg_text))
            return

    # Important fix:
    # json.loads("0") returns int 0, not a dict.
    # json.loads("1.5") returns float 1.5, not a dict.
    # The old version then tried data.get(...), so the target was not applied.
    if isinstance(data, (int, float)):
        queue_mqtt_target(data, "raw_json_number")
        return

    # json.loads('"0"') returns string "0".
    if isinstance(data, str):
        try:
            queue_mqtt_target(float(data), "raw_json_string")
            return
        except Exception:
            print("MQTT_RX_BAD_STRING,{},{}".format(topic_text, msg_text))
            return

    if not isinstance(data, dict):
        print("MQTT_RX_BAD_TYPE,{},{}".format(topic_text, msg_text))
        return

    command = data.get("command", "")

    if topic == TOPIC_SYSTEM_EMERGENCY or command == "emergency_stop":
        enable = bool_value(data.get("enable", True), True)

        if enable:
            emergency_enabled = True
            mqtt_target_pending = 0.0
            load_status = "emergency"
            shutdown()
            print("MQTT_EMERGENCY_STOP")
        else:
            emergency_enabled = False
            load_status = "enabled"
            print("MQTT_EMERGENCY_CLEAR")

        return

    if topic == TOPIC_SYSTEM_RESTART or command == "restart":
        emergency_enabled = False
        load_status = "enabled"
        print("MQTT_RESTART_RECEIVED")
        return

    # Main LED load command.
    # Accept several possible backend field names.
    for key in (
        "power_ref_w",
        "demand_power",
        "power",
        "target_power_w",
        "p_target",
        "value",
        "load_power",
        "load_power_w",
        "target",
        "target_power",
        "power_w"
    ):
        if key in data:
            queue_mqtt_target(data[key], key)
            return

    # Also support command-style messages, for example:
    # {"command":"set_power", "value":0}
    # {"command":"set_load_power", "power":1.5}
    if command in ("set_power", "set_load_power", "set_target", "set_load"):
        for key in (
            "value",
            "power",
            "power_ref_w",
            "demand_power",
            "target_power_w",
            "load_power",
            "load_power_w"
        ):
            if key in data:
                queue_mqtt_target(data[key], "command_" + key)
                return

        print("MQTT_CMD_NO_VALUE,{},{}".format(topic_text, msg_text))
        return

    if command in ("stop", "disable"):
        mqtt_target_pending = 0.0
        load_status = "disabled"
        print("MQTT_LOAD_DISABLED")
        return

    if command in ("enable", "start"):
        emergency_enabled = False
        load_status = "enabled"
        print("MQTT_LOAD_ENABLED")
        return

    print("MQTT_RX_NO_POWER,{},{}".format(topic_text, msg_text))


def start_wifi_if_needed():
    global wlan, last_wifi_attempt_ms

    if network is None:
        return False

    if wlan is None:
        wlan = network.WLAN(network.STA_IF)
        wlan.active(True)

    if wlan.isconnected():
        return True

    now = time.ticks_ms()
    if (
        last_wifi_attempt_ms == 0 or
        time.ticks_diff(now, last_wifi_attempt_ms) >= WIFI_RETRY_MS
    ):
        last_wifi_attempt_ms = now
        print("WIFI_CONNECTING,{}".format(WIFI_SSID))

        try:
            wlan.connect(WIFI_SSID, WIFI_PASSWORD)
        except Exception as error:
            print("WIFI_CONNECT_ERROR,{}".format(error))

    return False


def connect_mqtt_if_needed():
    global mqtt_client, mqtt_connected, last_mqtt_attempt_ms

    if MQTTClient is None:
        if MQTT_IMPORT_ERROR is not None:
            print("MQTTClient not found: {}".format(MQTT_IMPORT_ERROR))
        else:
            print("MQTTClient not found")
        return

    if mqtt_connected:
        return

    if not start_wifi_if_needed():
        return

    now = time.ticks_ms()
    if (
        last_mqtt_attempt_ms != 0 and
        time.ticks_diff(now, last_mqtt_attempt_ms) < MQTT_RETRY_MS
    ):
        return

    last_mqtt_attempt_ms = now

    try:
        client_id = "{}_{}".format(MQTT_CLIENT_ID, time.ticks_ms())

        mqtt_client = MQTTClient(
            client_id,
            MQTT_BROKER,
            port=MQTT_PORT,
            keepalive=30
        )

        mqtt_client.set_callback(mqtt_callback)
        mqtt_client.connect()

        mqtt_client.subscribe(TOPIC_LOAD_COMMAND)
        mqtt_client.subscribe(TOPIC_SYSTEM_EMERGENCY)
        mqtt_client.subscribe(TOPIC_SYSTEM_RESTART)

        mqtt_connected = True

        print("MQTT_CONNECTED,{}:{}".format(MQTT_BROKER, MQTT_PORT))
        print("MQTT_SUB,{}".format(topic_to_str(TOPIC_LOAD_COMMAND)))
        print("MQTT_SUB,{}".format(topic_to_str(TOPIC_SYSTEM_EMERGENCY)))
        print("MQTT_SUB,{}".format(topic_to_str(TOPIC_SYSTEM_RESTART)))

    except Exception as error:
        mqtt_connected = False
        mqtt_client = None
        print("MQTT_CONNECT_ERROR,{}".format(error))


def mqtt_check_messages():
    global mqtt_connected, mqtt_client

    if not mqtt_connected or mqtt_client is None:
        return

    try:
        mqtt_client.check_msg()
    except Exception as error:
        mqtt_connected = False
        mqtt_client = None
        print("MQTT_CHECK_ERROR,{}".format(error))


def mqtt_publish_telemetry(
    demand_power,
    actual_power,
    red_power,
    yellow_power,
    green_power,
    pwm_red_value,
    pwm_yel_value,
    pwm_grn_value
):
    global mqtt_connected, mqtt_client

    if not mqtt_connected or mqtt_client is None:
        return

    try:
        payload = {
            "device": "load",
            "demand_power": round(demand_power, 3),
            "actual_power": round(actual_power, 3),
            "power": round(actual_power, 3),
            "red_power": round(red_power, 3),
            "yellow_power": round(yellow_power, 3),
            "green_power": round(green_power, 3),
            "pwm_red": int(pwm_red_value),
            "pwm_yellow": int(pwm_yel_value),
            "pwm_green": int(pwm_grn_value),
            "status": load_status
        }

        payload_text = json.dumps(payload)
        mqtt_client.publish(TOPIC_LOAD_TELEMETRY, payload_text.encode())

    except Exception as error:
        mqtt_connected = False
        mqtt_client = None
        print("MQTT_PUBLISH_ERROR,{}".format(error))


# PID setup
p_target = limit_target(p_target)
channel_target = p_target / 3.0
prev_request = p_target

controller_red = PID(
    0.01,
    2,
    0,
    setpoint=channel_target,
    scale="ms"
)

controller_yel = PID(
    0.01,
    2,
    0,
    setpoint=channel_target,
    scale="ms"
)

controller_grn = PID(
    0.01,
    2,
    0,
    setpoint=channel_target,
    scale="ms"
)


# Main loop variables
count = 0
mqtt_check_count = 0
mqtt_service_count = 0
start_time = time.ticks_ms()

pwm_red_out = 0
pwm_yel_out = 0
pwm_grn_out = 0


print("PICO_READY")
print(
    "D_HEADER,time_s,p_total_target_w,p_total_actual_w,"
    "p_red_target_w,p_red_actual_w,"
    "p_yel_target_w,p_yel_actual_w,"
    "p_grn_target_w,p_grn_actual_w,"
    "pwm_red,pwm_yel,pwm_grn"
)


loop_timer = Timer(
    mode=Timer.PERIODIC,
    freq=CONTROL_FREQ,
    callback=tick
)


try:
    while True:

        if timer_elapsed == 1:
            timer_elapsed = 0
            count += 1
            mqtt_check_count += 1
            mqtt_service_count += 1

            # MQTT service runs outside the PID calculation.
            # It only updates the same p_target variable that serial input used.
            if mqtt_service_count >= MQTT_SERVICE_INTERVAL_TICKS:
                mqtt_service_count = 0
                connect_mqtt_if_needed()

            if mqtt_check_count >= MQTT_CHECK_INTERVAL_TICKS:
                mqtt_check_count = 0
                mqtt_check_messages()

            # Read total target power from serial or MQTT backend.
            p_target = read_serial_target(p_target)

            old_target = p_target
            p_target = read_mqtt_target(p_target)

            if abs(p_target - old_target) > 0.001:
                print("TARGET_APPLIED_TO_CONTROL,{:.3f}".format(p_target))

            p_target = limit_target(p_target)

            # Update PID setpoints only when target changes.
            if abs(p_target - prev_request) > 0.001:
                prev_request = p_target
                channel_target = p_target / 3.0

                controller_red.setpoint = channel_target
                controller_yel.setpoint = channel_target
                controller_grn.setpoint = channel_target

                p_sum = 0.0
                p_sum_red = 0.0
                p_sum_yel = 0.0
                p_sum_grn = 0.0
                count = 0

                print(
                    "SETPOINT_UPDATED,{:.3f},{:.3f}".format(
                        p_target,
                        channel_target
                    )
                )

            if emergency_enabled:
                shutdown()

                if count >= PRINT_INTERVAL:
                    print(
                        "D,{:.3f},{:.3f},{:.3f},"
                        "{:.3f},{:.3f},"
                        "{:.3f},{:.3f},"
                        "{:.3f},{:.3f},"
                        "{},{},{}".format(
                            time.ticks_diff(time.ticks_ms(), start_time) / 1000,
                            p_target,
                            0.0,
                            controller_red.setpoint,
                            0.0,
                            controller_yel.setpoint,
                            0.0,
                            controller_grn.setpoint,
                            0.0,
                            0,
                            0,
                            0
                        )
                    )

                    mqtt_publish_telemetry(
                        p_target,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0,
                        0,
                        0
                    )

                    count = 0

                continue

            pwm_red_en.value(1)
            pwm_yel_en.value(1)
            pwm_grn_en.value(1)

            # Read current-sense ADC values
            ired_pin = 2.497 * (readadc(4) / 4096)
            iyel_pin = 2.497 * (readadc(2) / 4096)
            igrn_pin = 2.497 * (readadc(0) / 4096)

            # Read voltage-sense ADC values
            vred_pin = 2.497 * (readadc(5) / 4096)
            vyel_pin = 2.497 * (readadc(3) / 4096)
            vgrn_pin = 2.497 * (readadc(1) / 4096)

            # Calculate LED voltages
            vred = max(2 * vred_pin - ired_pin, 0.0)
            vyel = max(2 * vyel_pin - iyel_pin, 0.0)
            vgrn = max(2 * vgrn_pin - igrn_pin, 0.0)

            # Calculate LED currents
            ired = max(3 * ired_pin, 0.0)
            iyel = max(3 * iyel_pin, 0.0)
            igrn = max(3 * igrn_pin, 0.0)

            # Calculate actual power
            p_red = vred * ired
            p_yel = vyel * iyel
            p_grn = vgrn * igrn

            p_total_actual = p_red + p_yel + p_grn

            # Accumulate power for averaging
            p_sum += p_total_actual
            p_sum_red += p_red
            p_sum_yel += p_yel
            p_sum_grn += p_grn

            # PID always runs
            pwm_red_ref = controller_red(p_red)
            pwm_yel_ref = controller_yel(p_yel)
            pwm_grn_ref = controller_grn(p_grn)

            pwm_red_out = saturate(int(pwm_red_ref * 65536))
            pwm_yel_out = saturate(int(pwm_yel_ref * 65536))
            pwm_grn_out = saturate(int(pwm_grn_ref * 65536))

            pwm_red.duty_u16(pwm_red_out)
            pwm_yel.duty_u16(pwm_yel_out)
            pwm_grn.duty_u16(pwm_grn_out)

            # Print averaged telemetry
            if count >= PRINT_INTERVAL:
                time_s = (
                    time.ticks_diff(time.ticks_ms(), start_time)
                    / 1000
                )

                p_avg = p_sum / PRINT_INTERVAL
                p_avg_red = p_sum_red / PRINT_INTERVAL
                p_avg_yel = p_sum_yel / PRINT_INTERVAL
                p_avg_grn = p_sum_grn / PRINT_INTERVAL

                print(
                    "D,{:.3f},{:.3f},{:.3f},"
                    "{:.3f},{:.3f},"
                    "{:.3f},{:.3f},"
                    "{:.3f},{:.3f},"
                    "{},{},{}".format(
                        time_s,
                        p_target,
                        p_avg,
                        controller_red.setpoint,
                        p_avg_red,
                        controller_yel.setpoint,
                        p_avg_yel,
                        controller_grn.setpoint,
                        p_avg_grn,
                        pwm_red_out,
                        pwm_yel_out,
                        pwm_grn_out
                    )
                )

                mqtt_publish_telemetry(
                    p_target,
                    p_avg,
                    p_avg_red,
                    p_avg_yel,
                    p_avg_grn,
                    pwm_red_out,
                    pwm_yel_out,
                    pwm_grn_out
                )

                p_sum = 0.0
                p_sum_red = 0.0
                p_sum_yel = 0.0
                p_sum_grn = 0.0
                count = 0


except KeyboardInterrupt:
    pass


finally:
    shutdown()

    try:
        loop_timer.deinit()
    except:
        pass

    print("PICO_STOPPED")
