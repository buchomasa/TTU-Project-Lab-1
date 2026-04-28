from machine import Pin
import machine

class WheelEncoders:
    def __init__(self, left_pin=28, right_pin=16):
        self.left_enc = Pin(left_pin, Pin.IN)
        self.right_enc = Pin(right_pin, Pin.IN)
        self.left_pulses = 0
        self.right_pulses = 0
        self.left_enc.irq(trigger=Pin.IRQ_RISING, handler=self._left_isr)
        self.right_enc.irq(trigger=Pin.IRQ_RISING, handler=self._right_isr)

    def _left_isr(self, pin):
        self.left_pulses += 1

    def _right_isr(self, pin):
        self.right_pulses += 1

    def get_pulses(self):
        return self.left_pulses, self.right_pulses

    def reset(self):
        state = machine.disable_irq()
        self.left_pulses = 0
        self.right_pulses = 0
        machine.enable_irq(state)