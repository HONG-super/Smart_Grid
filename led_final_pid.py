from machine import Pin, I2C, ADC, PWM, Timer, SPI
from PID import PID ### !!!! Ensure PID.py is copied to the Pico for this to function.
import time

# Test settings
p_target = 1.5    #this is the variable that recives power demand at the current server tick
p_sum = 0
p_sum_red = 0
p_sum_yel = 0
p_sum_grn = 0
deadband = 0.01

PWM_FREQ = 100000
CONTROL_FREQ = 1000
PRINT_INTERVAL = 100

P_CHANNEL_MAX = 1.1
I_CHANNEL_MAX = 0.4
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

# Functions
def saturate(duty):
    if duty > PWM_MAX:
        duty = PWM_MAX
    if duty < 100:
        duty = 100
    return duty


def shutdown():
    pwm_red.duty_u16(0)
    pwm_yel.duty_u16(0)
    pwm_grn.duty_u16(0)

    pwm_red_en.value(0)
    pwm_yel_en.value(0)
    pwm_grn_en.value(0)

# PID setup
controller_red = PID(0.01, 2, 0, setpoint=1, scale="ms")
controller_yel = PID(0.01, 2, 0, setpoint=1, scale="ms")
controller_grn = PID(0.01, 2, 0, setpoint=1, scale="ms")

# Main loop
count = 0
start_time = time.ticks_ms()
power_num = 0
count1 = 0
channel_target = 0


print(
    "time_s,p_total_target_w,p_total_actual_w,"
    "p_red_target_w,p_red_actual_w,"
    "p_yel_target_w,p_yel_actual_w,"
    "p_grn_target_w,p_grn_actual_w"
)

loop_timer = Timer(mode=Timer.PERIODIC, freq=CONTROL_FREQ, callback=tick)

try:
    
    while True:
        
        if timer_elapsed == 1:
            pwm_red_en.value(1)
            pwm_yel_en.value(1)
            pwm_grn_en.value(1)
            timer_elapsed = 0
            count += 1
            count1 += 1
            prev_request = 3*channel_target
            
            if prev_request != min(p_target, 3):
                print ( str(prev_request)+"->"+str(p_target))
                prev_request = min(p_target,3)
                channel_target = p_target/3
                controller_red.setpoint = min(channel_target,1)
                controller_yel.setpoint = min(channel_target,1)
                controller_grn.setpoint = min(channel_target,1)
            
            ired_pin = 2.497 * (readadc(4) / 4096)
            iyel_pin = 2.497 * (readadc(2) / 4096)
            igrn_pin = 2.497 * (readadc(0) / 4096)
            
            vred_pin = 2.497 * (readadc(5) / 4096)
            vyel_pin = 2.497 * (readadc(3) / 4096)
            vgrn_pin = 2.497 * (readadc(1) / 4096)
            
            vred = max(2 * vred_pin - ired_pin, 0.0)
            vyel = max(2 * vyel_pin - iyel_pin, 0.0)
            vgrn = max(2 * vgrn_pin - igrn_pin, 0.0)
            
            ired = max(3 * ired_pin, 0.0)
            iyel = max(3 * iyel_pin, 0.0)
            igrn = max(3 * igrn_pin, 0.0)
            
            p_red = vred * ired
            p_yel = vyel * iyel
            p_grn = vgrn * igrn
            
            p_total_actual = p_red + p_yel + p_grn
            
            p_sum = p_sum + p_total_actual
            p_sum_red = p_sum_red + p_red
            p_sum_yel = p_sum_yel + p_yel
            p_sum_grn = p_sum_grn + p_grn
            
            
            pwm_red_ref = controller_red(p_red)
            pwm_red_out = saturate(int(pwm_red_ref * 65536))
            pwm_red.duty_u16(pwm_red_out)
            
            pwm_yel_ref = controller_yel(p_yel)
            pwm_yel_out = saturate(int(pwm_yel_ref * 65536))
            pwm_yel.duty_u16(pwm_yel_out)
            
            pwm_grn_ref = controller_grn(p_grn)
            pwm_grn_out = saturate(int(pwm_grn_ref * 65536))
            pwm_grn.duty_u16(pwm_grn_out)
                
            
            
            
            if count >= PRINT_INTERVAL:
                time_s = time.ticks_diff(time.ticks_ms(), start_time) / 1000
                #avg power calculation
                p_avg = p_sum/PRINT_INTERVAL
                p_avg_red = p_sum_red/PRINT_INTERVAL
                p_avg_yel = p_sum_yel/PRINT_INTERVAL
                p_avg_grn = p_sum_grn/PRINT_INTERVAL
                
                #power sums set back to 0
                p_sum = 0
                p_sum_red = 0
                p_sum_yel = 0
                p_sum_grn = 0
                
                print(
                "{:.3f},{:.3f},{:.3f},"
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
                
                count = 0
                
except KeyboardInterrupt:
    pass

finally:
    shutdown()

    try:
        loop_timer.deinit()
    except:
        pass