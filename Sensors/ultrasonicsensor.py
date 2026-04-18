from time import sleep
import time
from machine import Pin, PWM

# ---------------- SERVO SETUP ----------------
pwm = PWM(Pin(0))
pwm.freq(50)

def setServoCycle(position):
    pwm.duty_u16(position)
    sleep(0.01)

# ---------------- SERVO LIMITS (REDUCED RANGE) ----------------
# was: 1000 -> 8000 (full sweep)
LEFT_LIMIT = 2500     # reduced left travel
RIGHT_LIMIT = 6500    # reduced right travel

# ---------------- ULTRASONIC SETUP ----------------
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

# ---------------- PINS ----------------
echo1 = Pin(2, Pin.IN)
trig1 = Pin(3, Pin.OUT)

echo1.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=echo_ISR)

# ---------------- STATE ----------------
tracking = False
current_pos = LEFT_LIMIT
step = 100
direction = step

# 12 inches ≈ 30 cm
THRESHOLD_CM = 30

# ---------------- MAIN LOOP ----------------
while True:

    triggerSensor(trig1)
    sleep(0.03)

    distance = detectObject()

    # ---------------- TRACKING MODE ----------------
    if distance is not None and distance <= THRESHOLD_CM:
        tracking = True

    if tracking:
        setServoCycle(current_pos)

        print("TRACKING OBJECT:", distance, "cm")

        if distance is None or distance > THRESHOLD_CM:
            tracking = False

    # ---------------- SWEEP MODE ----------------
    else:
        setServoCycle(current_pos)

        current_pos += direction

        # reverse at reduced limits
        if current_pos >= RIGHT_LIMIT:
            current_pos = RIGHT_LIMIT
            direction = -step

        elif current_pos <= LEFT_LIMIT:
            current_pos = LEFT_LIMIT
            direction = step

        print("SWEEP:", current_pos, "DIST:", distance)
