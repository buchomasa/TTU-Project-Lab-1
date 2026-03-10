from machine import Pin, PWM
from time import sleep

in3 = Pin(14,Pin.OUT)
in4 = Pin(15, Pin.OUT)
in5 = Pin(12, Pin.OUT)
in6 = Pin(13, Pin.OUT)


enb = PWM(Pin(13))
enb.freq(1000)

def motor_forward(speed):
    in3.high()
    in4.low()
    in5.high()
    in6.low()
    enb.duty_u16(speed)
    
def motor_backward(speed):
    in3.low()
    in4.high()
    in5.low()
    in6.high()
    enb.duty_u16(speed)

def motor_stop():
    in3.low()
    in4.low()
    in5.low()
    in6.low()
    enb.duty_u16(0)
    
def turn_right():
    in3.high()
    in4.low()
    in5.low()
    in6.low()
    
def turn_left():
    in3.low()
    in4.low()
    in5.high()
    in6.low

try:
    while True:
        print("Moving Forward")
        motor_forward(32768)
        sleep(1)
        
        print("Stopping")
        motor_stop()
        sleep(1)
        
        print("Moving Backwards")
        motor_backward(32768)
        sleep(1)
        
        print("Stopping")
        motor_stop()
        sleep(1)
        
        print("turning right")
        turn_right()
        sleep(1)
        
        print("stopping")
        motor_stop()
        sleep(1)
        
        print("turning left")
        turn_left()
        sleep(1)
except:
    sleep(1)
