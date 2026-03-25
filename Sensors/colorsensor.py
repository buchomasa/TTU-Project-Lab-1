from machine import Pin
import time

s2  = Pin(2, Pin.OUT)
s3  = Pin(3, Pin.OUT)
out = Pin(4, Pin.IN)

def read_channel(s2v, s3v):
    s2.value(s2v); s3.value(s3v)
    time.sleep_ms(20)
    count = 0
    end = time.ticks_add(time.ticks_ms(), 100)
    last = out.value()
    while time.ticks_diff(end, time.ticks_ms()) > 0:
        cur = out.value()
        if last == 1 and cur == 0:
            count += 1
        last = cur
    return count

def check_ball():
    r = read_channel(0, 0)
    g = read_channel(1, 1)
    b = read_channel(0, 1)
    c = read_channel(1, 0)
    if c < 30 or g == 0 or b == 0:
        return "NONE"
    if (r / g) > 1.1 and (r / b) > 1.1:
        return "RED"
    return "NOT RED"

while True:
    result = check_ball()
    if result == "RED":
        print("RED BALL - avoid")
    elif result == "NOT RED":
        print("NOT RED - safe")
    time.sleep_ms(2000)
