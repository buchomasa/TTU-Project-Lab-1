from machine import ADC, Pin
import time

sensor = ADC(Pin(26))

BLACK_MAX = 15000
BLUE_MIN  = 20000
BLUE_MAX  = 35000

def surface():
    val = sum(sensor.read_u16() for _ in range(4)) >> 2
    if val <= BLACK_MAX:
        print(f"RAW: {val} → BLACK")
        return "BLACK"
    if BLUE_MIN <= val <= BLUE_MAX:
        print(f"RAW: {val} → BLUE")
        return "BLUE"
    print(f"RAW: {val} → FLOOR")
    return "FLOOR"

IS_GOALIE = True

while True:
    s = surface()
    if s == "BLACK":
        print("BORDER → reverse!")
    elif IS_GOALIE and s == "BLUE":
        print("CENTER → goalie stop!")
    time.sleep_ms(2000)
