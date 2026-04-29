from machine import Pin
import time


pin = Pin(11, Pin.OUT, Pin.PULL_DOWN)

def kick(duration=0.1):

    pin.value(1)          
    time.sleep(duration)  
    pin.value(0)          

while True:
    kick()
    time.sleep(0.1)
