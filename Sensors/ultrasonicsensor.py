from time import sleep
import time
from machine import Pin, PWM

pwm = PWM(Pin(0))
pwm.freq(50)

def setServoCycle(position):
    pwm.duty_u16(position)
    sleep(0.01)


LEFT_LIMIT = 2500     
RIGHT_LIMIT = 6500    

start1 = 0
end1 = 0

def echo_ISR(pin):
    global start1, end1

    if pin.value():
        start1 = time.ticks_us()
    else:
        end1 = time.ticks_us()

def detectObject():
    global start1, end1

    pulse_time = time.ticks_diff(end1, start1)
    if pulse_time <= 0:
        return None

    return (pulse_time * 0.0343) / 2  # cm

def triggerSensor(trig):
    trig.low()
    time.sleep_us(2)
    trig.high()
    time.sleep_us(10)
    trig.low()

echo1 = Pin(2, Pin.IN)
trig1 = Pin(3, Pin.OUT)

echo1.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=echo_ISR)

tracking = False
current_pos = LEFT_LIMIT
step = 100
direction = step

THRESHOLD_CM = 30

while True:

    triggerSensor(trig1)
    sleep(0.03)

    distance = detectObject()

    if distance is not None and distance <= THRESHOLD_CM:
        tracking = True

    if tracking:
        setServoCycle(current_pos)

        print("TRACKING OBJECT:", distance, "cm")

        if distance is None or distance > THRESHOLD_CM:
            tracking = False


    else:
        setServoCycle(current_pos)

        current_pos += direction

        if current_pos >= RIGHT_LIMIT:
            current_pos = RIGHT_LIMIT
            direction = -step

        elif current_pos <= LEFT_LIMIT:
            current_pos = LEFT_LIMIT
            direction = step

        print("SWEEP:", current_pos, "DIST:", distance)
