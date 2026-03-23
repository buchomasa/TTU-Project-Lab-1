from machine import Pin
import time

start1 = 0
end1 = 0
start2 = 0
end2 = 0

def echo_ISR(pin):
    global start1, end1, start2, end2
    
    if pin == echo1:
        if pin.value() == 1:
            start1 = time.ticks_us()
        else:
            end1 = time.ticks_us()
    elif pin == echo2:
        if pin.value() == 1:
            start2 = time.ticks_us()
        else:
            end2 = time.ticks_us()

def detectObject(sensor):
    global start1, end1, start2, end2
    
    if sensor == 1:
        pulse_time = time.ticks_diff(end1, start1)
    elif sensor == 2:
        pulse_time = time.ticks_diff(end2, start2)
    else:
        return None
    if pulse_time <= 0:
        return None
    
    distance = (pulse_time * 0.0343) / 2
    return distance

def triggerSensor(trig):
    trig.low()
    time.sleep_us(2)
    trig.high()
    time.sleep_us(10)
    trig.low()

echo1 = Pin(2, Pin.IN)
trig1 = Pin(3, Pin.OUT)
echo2 = Pin(4, Pin.IN)
trig2 = Pin(5, Pin.OUT)

echo1.irq(trigger = Pin.IRQ_RISING | Pin.IRQ_FALLING, handler = echo_ISR)
echo2.irq(trigger = Pin.IRQ_RISING | Pin.IRQ_FALLING, handler = echo_ISR)

while True:
    
    triggerSensor(trig1)
    time.sleep_ms(50)
    
    d1 = detectObject(1)
    
    triggerSensor(trig2)
    time.sleep_ms(50)
    
    d2 = detectObject(2)
    
    print("Sensor1: ", d1, "cm")
    print("Sensor2: ", d2, "cm")
    
        
    
