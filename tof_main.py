from machine import I2C, Pin
import vl53l1x
import time

# ---- I2C & Sensor Setup ----
i2c = I2C(0, sda=Pin(4), scl=Pin(5), freq=400000)
tof = vl53l1x.VL53L1X(i2c)

# ---- Detection Thresholds (in mm) ----
# A 40mm ping pong ball will cause a sudden close reading
OFFSET = 30 # offset distance for calibration
BALL_DETECT_RANGE  = 150   # ball is within grabbing/kicking range
BALL_APPROACH_RANGE = 400  # ball is ahead, rover should approach

def get_distance():
    """Returns distance in mm. Returns None if reading is invalid."""
    d = tof.read() - OFFSET
    if d <= -10 or d >= 8190:   # sensor returns 8191 on out-of-range
        return None
    return d

def ball_status():
    """
    Returns one of three states:
      'IN_RANGE'  - ball is close, ready to kick/grab
      'APPROACH'  - ball detected ahead, keep driving toward it
      'NONE'      - no ball detected in front of sensor
    """
    d = get_distance()
    if d is None:
        return 'NONE'
    elif d <= BALL_DETECT_RANGE:
        return 'IN_RANGE'
    elif d <= BALL_APPROACH_RANGE:
        return 'APPROACH'
    else:
        return 'NONE'

# ---- Main Loop (for testing) ----
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