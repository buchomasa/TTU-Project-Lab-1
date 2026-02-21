from machine import I2C, Pin
from tcs34725 import TCS34725
import time

def scanColors(timeout_s=10):
    i2c = I2C(0, scl=Pin(1), sda=Pin(0))
    sensor = TCS34725(i2c)

    start = time.ticks_ms()
    detected = None

    while True:
        r, g, b, c = sensor.read()

        if c > 50:  
            if r > g and r > b:
                detected = "Red"
                break
            elif g > r and g > b:
                detected = "Green"
                break
            elif b > r and b > g:
                detected = "Blue"
                break

        if time.ticks_diff(time.ticks_ms(), start) > timeout_s * 1000:
            break

        time.sleep(0.05)

    sensor.disable()

    if detected:
        print(detected + " detected")
    else:
        print("Nothing detected")

scanColors(10)