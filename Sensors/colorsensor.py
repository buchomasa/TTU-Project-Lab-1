"""
Red Ball Detection — TCS34725 on Pico 2 (2 GPIO pins)
Wiring:
"""
from machine import I2C, Pin
import time
i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=400000)
ADDR = 0x29
def write(reg, val):
    i2c.writeto_mem(ADDR, 0x80 | reg, bytes([val]))
def read16(reg):
    d = i2c.readfrom_mem(ADDR, 0x80 | reg, 2)
    return d[1] << 8 | d[0]
# Enable sensor: power on + RGBC enable
write(0x00, 0x03)
# Integration time ~154ms
write(0x01, 0xC0)
# Gain 4x
write(0x0F, 0x01)
time.sleep_ms(200)
def check_ball():
    c = read16(0x14)
    r = read16(0x16)
    g = read16(0x18)
    b = read16(0x1A)
    print("R={} G={} B={} C={}".format(r, g, b, c))
    if c < 30 or g == 0 or b == 0:
        return "NONE"
    if (r / g) > 1.5 and (r / b) > 2.0:
        return "RED"
    return "NOT RED"
while True:
    result = check_ball()
    if result == "RED":
        print("RED BALL - avoid")
    elif result == "NOT RED":
        print("NOT RED - safe")
    time.sleep_ms(2000)
