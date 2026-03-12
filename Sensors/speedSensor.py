from machine import Pin
import time

left_count = 0
right_count = 0
pulseREV = 20
def speedISR1(pin):
    global left_count
    left_count += 1 
    
def speedISR2(pin):
    global right_count
    right_count += 1
    
def measureSpeed(interval=0.5):
    global left_count, right_count
    start_left = left_count
    start_right = right_count
    time.sleep(interval)
    pulses_left = left_count - start_left
    pulses_right = right_count - start_right
    
    pps_left = pulses_left / interval
    pps_right = pulses_right / interval
    
    rpm_left = (pps_left * 60) / pulseREV
    rpm_right = (pps_right * 60) / pulseREV
    
    return rmp_left, rpm_right

#Code for testing

while True:
    left_speed, right_speed = measure_speed()

    print("Left RPM:", left_speed)
    print("Right RPM:", right_speed)
