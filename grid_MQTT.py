from machine import Pin, I2C, ADC, PWM, Timer
import utime


try:
    import network
    from umqtt.simple import MQTTClient
except Exception as e:
    network = None
    MQTTClient = None
    print("WiFi/MQTT import failed:", e)

#CSV
SAVE_RESULTS = True
RESULTS_FILE = "external_grid_10V_nopv_2.csv"
LOG_DURATION_MS = 60000        
LOG_PERIOD_MS = 10             
LOG_FLUSH_PERIOD_MS = 1000    

#WiFi / MQTT settings
MQTT_ENABLE = True

WIFI_SSID = "Hong"
WIFI_PASSWORD = "abcdefgh"

#broker host
MQTT_BROKER = "172.20.10.4"
MQTT_PORT = 1884
MQTT_CLIENT_ID = b"grid_pico_w"
MQTT_TOPIC_GRID_TELEMETRY = b"smartgrid/team01/grid/telemetry"

MQTT_PUBLISH_PERIOD_MS = 1000
WIFI_CONNECT_TIMEOUT_MS = 15000
MQTT_RECONNECT_PERIOD_MS = 5000
WIFI_RECONNECT_PERIOD_MS = 5000
LED_BLINK_PERIOD_MS = 250

# Hardware setup
led = Pin("LED", Pin.OUT)
led.off()

va_pin = ADC(Pin(28))
vb_pin = ADC(Pin(26))

ina_i2c = I2C(0, scl=Pin(1), sda=Pin(0), freq=2400000)

pwm = PWM(Pin(9))
pwm.freq(100000)
min_pwm = 1000
max_pwm = 64536
pwm_out = min_pwm

# Controller settings

v_ref = 10.00
v_err = 0.0
v_err_int = 0.0
v_pi_out = 0.0
kp = 150
ki = 300

timer_elapsed = 0
count = 0
first_run = 1

SHUNT_OHMS = 0.10

# log
results = None;
log_start_ms = 0
last_log_ms = 0
last_flush_ms = 0
log_done = False

# WiFi / MQTT variables
wlan = None
mqtt_client = None
wifi_connected = False
mqtt_connected = False
wifi_connecting = False
wifi_connect_start_ms = 0
last_wifi_attempt_ms = 0
last_mqtt_ms = 0
last_mqtt_attempt_ms = 0
last_led_ms = 0
led_state = 0

def saturate(signal, upper, lower):
    if signal > upper:
        signal = upper
    if signal < lower:
        signal = lower
    return signal

def tick(t):
    global timer_elapsed
    timer_elapsed = 1

def led_blink_update(now_ms):
    global last_led_ms
    global led_state

    if utime.ticks_diff(now_ms, last_led_ms) >= LED_BLINK_PERIOD_MS:
        last_led_ms = now_ms
        led_state = 1 - led_state
        led.value(led_state)

def led_status_update():             # solid is success
    if (not MQTT_ENABLE) or (network is None) or (MQTTClient is None):
        led.off()
        return
    if wifi_connected and mqtt_connected:
        led.on()
    else:
        led_blink_update(utime.ticks_ms())

def get_wlan():
    global wlan

    if network is None:
        return None

    if wlan is None:
        wlan = network.WLAN(network.STA_IF)
        wlan.active(True)
    return wlan

def start_wifi_connect(now_ms):
    global wifi_connecting
    global wifi_connect_start_ms
    global last_wifi_attempt_ms
    w = get_wlan()
    if w is None:
        return False
    try:
        w.active(True)
        try:
            w.disconnect()
        except Exception:
            pass
        print("WiFi connecting to SSID:", WIFI_SSID)
        w.connect(WIFI_SSID, WIFI_PASSWORD)
        wifi_connecting = True
        wifi_connect_start_ms = now_ms
        last_wifi_attempt_ms = now_ms
        return True
    except Exception as e:
        print("WiFi start connect exception:", e)
        wifi_connecting = False
        last_wifi_attempt_ms = now_ms
        return False

def update_wifi_connection():
    global wifi_connected
    global mqtt_connected
    global mqtt_client
    global wifi_connecting
    global last_wifi_attempt_ms

    if (not MQTT_ENABLE) or (network is None):
        wifi_connected = False
        return False
    now_ms = utime.ticks_ms()
    w = get_wlan()
    if w is None:
        wifi_connected = False
        return False
    try:
        if w.isconnected():
            if not wifi_connected:
                print("WiFi connected. IP =", w.ifconfig()[0])
            wifi_connected = True
            wifi_connecting = False
            return True

    except Exception as e:
        print("WiFi status check failed:", e)
    if wifi_connected:
        print("WiFi lost. Reconnecting...")

    wifi_connected = False
    mqtt_connected = False
    mqtt_client = None

    if wifi_connecting:
        if utime.ticks_diff(now_ms, wifi_connect_start_ms) >= WIFI_CONNECT_TIMEOUT_MS:
            print("WiFi connect timeout. Will retry.")
            wifi_connecting = False
            last_wifi_attempt_ms = now_ms

            try:
                w.disconnect()
            except Exception:
                pass

        return False

    if utime.ticks_diff(now_ms, last_wifi_attempt_ms) >= WIFI_RECONNECT_PERIOD_MS:
        start_wifi_connect(now_ms)

    return False


def connect_wifi():
    global wifi_connected
    global wifi_connecting
    global last_wifi_attempt_ms

    if (not MQTT_ENABLE) or (network is None):
        print("WiFi disabled or network module unavailable.")
        return False

    now_ms = utime.ticks_ms()
    start_wifi_connect(now_ms)

    start_connect_ms = now_ms
    last_status_print_ms = start_connect_ms

    while True:
        now_connect_ms = utime.ticks_ms()
        if update_wifi_connection():
            return True
        led_status_update()
        if utime.ticks_diff(now_connect_ms, last_status_print_ms) >= 1000:
            last_status_print_ms = now_connect_ms
            try:
                status = get_wlan().status()
                ifconfig = get_wlan().ifconfig()
                print("WiFi still connecting... status=", status, "ifconfig=", ifconfig)
            except Exception:
                print("WiFi still connecting...")

        if utime.ticks_diff(now_connect_ms, start_connect_ms) >= WIFI_CONNECT_TIMEOUT_MS:
            print("WiFi connect failed. Will keep retrying in main loop.")
            wifi_connected = False
            wifi_connecting = False
            last_wifi_attempt_ms = now_connect_ms
            return False

        utime.sleep_ms(50)


def connect_mqtt():
    global mqtt_client

    if (not MQTT_ENABLE) or (MQTTClient is None):
        print("MQTT disabled or MQTTClient unavailable.")
        return False

    try:
        print("MQTT connecting to {}:{} ...".format(MQTT_BROKER, MQTT_PORT))
        mqtt_client = MQTTClient(MQTT_CLIENT_ID, MQTT_BROKER, port=MQTT_PORT)
        mqtt_client.connect()
        print("MQTT connected. Publish topic =", MQTT_TOPIC_GRID_TELEMETRY)
        return True

    except Exception as e:
        print("MQTT connect failed:", e)
        try:
            mqtt_client.disconnect()
        except Exception:
            pass
        mqtt_client = None
        return False


def publish_grid_telemetry(vb, power, import_power, export_power):
    global mqtt_connected
    global mqtt_client

    if (not MQTT_ENABLE) or (not mqtt_connected) or (mqtt_client is None):
        return

    payload = '{{"Vb":{:.3f},"power":{:.4f},"import_power":{:.4f},"export_power":{:.4f}}}'.format(
        vb,
        power,
        import_power,
        export_power
    )

    try:
        mqtt_client.publish(MQTT_TOPIC_GRID_TELEMETRY, payload.encode())
        print("MQTT publish:", payload)

    except Exception as e:
        print("MQTT publish failed:", e)
        mqtt_connected = False
        try:
            mqtt_client.disconnect()
        except Exception:
            pass
        mqtt_client = None

# INA219
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

        if reg_value >= 0x8000:
            reg_value = reg_value - 0x10000

        return float(reg_value) * 1e-5

    def vbus(self):
        reg_bytes = ina_i2c.readfrom_mem(self.address, self.REG_BUSVOLTAGE, 2)
        reg_value = int.from_bytes(reg_bytes, "big") >> 3
        return float(reg_value) * 0.004

    def configure(self):
        ina_i2c.writeto_mem(self.address, self.REG_CONFIG, b"\x19\x9F")
        ina_i2c.writeto_mem(self.address, self.REG_CALIBRATION, b"\x00\x00")

# Main loop
while True:

    if first_run:
        ina = ina219(SHUNT_OHMS, 64, 5)
        ina.configure()
        first_run = 0
        loop_timer = Timer(mode=Timer.PERIODIC, freq=1000, callback=tick)

        if MQTT_ENABLE:
            wifi_connected = connect_wifi()
            if wifi_connected:
                mqtt_connected = connect_mqtt()
            else:
                mqtt_connected = False
            led_status_update()

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
        last_mqtt_ms = log_start_ms
        last_mqtt_attempt_ms = log_start_ms

    if timer_elapsed == 1:
        va = 1.017 * (12490 / 2490) * 3.3 * (va_pin.read_u16() / 65536)
        vb = 1.015 * (12490 / 2490) * 3.3 * (vb_pin.read_u16() / 65536)

        Vshunt = ina.vshunt()
        iL = Vshunt / SHUNT_OHMS

        power = vb * iL
        
        min_pwm = 0
        max_pwm = 64536    
        v_err = v_ref - vb
        v_err_int = v_err_int + v_err
        v_err_int = saturate(v_err_int, 10000, -10000)
        v_pi_out = (kp * v_err) + (ki * v_err_int)
        pwm_out = saturate(v_pi_out, max_pwm, min_pwm)
        duty = int(65536 - pwm_out)
        pwm.duty_u16(duty)

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

# MQTT telemetry publishe
        if MQTT_ENABLE:
            now_mqtt_ms = utime.ticks_ms()
            wifi_connected = update_wifi_connection() #update wifi 

            if (not mqtt_connected) and wifi_connected:
                if utime.ticks_diff(now_mqtt_ms, last_mqtt_attempt_ms) >= MQTT_RECONNECT_PERIOD_MS:
                    last_mqtt_attempt_ms = now_mqtt_ms
                    mqtt_connected = connect_mqtt()

            if utime.ticks_diff(now_mqtt_ms, last_mqtt_ms) >= MQTT_PUBLISH_PERIOD_MS:
                last_mqtt_ms = now_mqtt_ms
                publish_grid_telemetry(vb, power)

            led_status_update()
            
        count = count + 1
        timer_elapsed = 0

        if count > 1000:
            count = 0
