from machine import Pin, PWM
from time import sleep


class Servo:
    def __init__(self, pin_num):
        self.servo = PWM(Pin(pin_num))
        self.servo.freq(50)

    def set_servo_angle(self, angle):
        angle = max(0, min(180, angle))
        duty = int(1638 + (angle / 180.0) * (8192 - 1638))
        self.servo.duty_u16(duty)


# ---------------- SERVO SETUP ----------------
servo9 = Servo(9)
servo22 = Servo(22)


# ---------------- CALIBRATED ANGLES ----------------
# Open positions
SERVO9_OPEN = 110
SERVO22_OPEN = 90

# Closed positions
SERVO9_CLOSED = 180
SERVO22_CLOSED = 10


# ---------------- FUNCTIONS ----------------
def ready():
    servo9.set_servo_angle(SERVO9_OPEN)
    servo22.set_servo_angle(SERVO22_OPEN)


def close():
    # Close pin 9 first
    servo9.set_servo_angle(SERVO9_CLOSED)

    # Wait 500 ms
    sleep(0.3)

    # Then close pin 22
    servo22.set_servo_angle(SERVO22_CLOSED)

while True:
    # Initialize claw open
    ready()
    sleep(2)
    close()
    sleep(2)
    ready()
