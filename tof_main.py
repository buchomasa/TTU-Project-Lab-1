from machine import I2C, Pin
import vl53l1x
import time
i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=400000)
tof = vl53l1x.VL53L1X(i2c)

OFFSET = 30 
BALL_DETECT_RANGE  = 150   
BALL_APPROACH_RANGE = 400  

def get_distance():
    """Returns distance in mm. Returns None if reading is invalid."""
    d = tof.read() - OFFSET
    if d <= -10 or d >= 8190:   # sensor returns 8191 on out-of-range
        return None
    return d

def ball_status():

    d = get_distance()
    if d is None:
        return 'NONE'
    elif d <= BALL_DETECT_RANGE:
        return 'IN_RANGE'
    elif d <= BALL_APPROACH_RANGE:
        return 'APPROACH'
    else:
        return 'NONE'

print("ToF ball detector ready\n")

while True:
    status = ball_status()
    dist   = get_distance()

    if status == 'IN_RANGE':
        print(f"BALL IN RANGE  → {dist} mm  — execute kick/grab")
    elif status == 'APPROACH':
        print(f"BALL AHEAD     → {dist} mm  — keep driving")
    else:
        print("No ball detected — scanning")

    time.sleep_ms(1000)
