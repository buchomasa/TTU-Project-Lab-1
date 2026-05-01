"""
Red Ball Detection - TCS34725 on Pico 2 (I2C0)
Wiring:
    VIN -> 3V3
    GND -> GND
    SDA -> GP0 (config.PIN_COLOR_SDA)
    SCL -> GP1 (config.PIN_COLOR_SCL)
    LED -> GND (keeps onboard LED on for consistent illumination)
"""
from machine import I2C, Pin, SoftI2C
import time
import config

class RedBallSensor:
    def __init__(self, i2c_bus=config.COLOR_I2C_BUS, addr=config.COLOR_SENSOR_ADDR):
        """Initializes the I2C bus and configures the color sensor."""
        self.addr = addr
        if config.COLOR_USE_SOFT_I2C:
            # Allows any GPIO pair (including GP20/GP21) even when
            # hardware I2C bus pin mapping would reject the pair.
            self.i2c = SoftI2C(
                sda=Pin(config.PIN_COLOR_SDA),
                scl=Pin(config.PIN_COLOR_SCL),
                freq=config.COLOR_I2C_FREQ,
            )
        else:
            self.i2c = I2C(
                i2c_bus,
                sda=Pin(config.PIN_COLOR_SDA),
                scl=Pin(config.PIN_COLOR_SCL),
                freq=config.COLOR_I2C_FREQ,
            )
        
        # Power on and initialize the sensor hardware automatically
        self._write(0x00, 0x03)                      # Power on + RGBC enable
        self._write(0x01, config.COLOR_INTEGRATION_TIME)
        self._write(0x0F, config.COLOR_GAIN)
        time.sleep_ms(200)

    def _write(self, reg, val):
        self.i2c.writeto_mem(self.addr, 0x80 | reg, bytes([val]))

    def _read16(self, reg):
        d = self.i2c.readfrom_mem(self.addr, 0x80 | reg, 2)
        return d[1] << 8 | d[0]

    def check_ball(self, debug=False):
        """
        Reads the sensor and determines if a red ball is present.
        Returns:
            "RED"     - red ball detected
            "NOT RED" - anything else (empty space, green/blue ball, etc.)
        """
        c = self._read16(0x14)
        r = self._read16(0x16)
        g = self._read16(0x18)
        b = self._read16(0x1A)

        if debug:
            print("R={} G={} B={} C={}".format(r, g, b, c))

        # Prevent division by zero errors if it reads pitch black
        if g == 0 or b == 0:
            return "NOT RED"

        # If the red ratio is high enough, it's a red ball
        if (r / g) > config.COLOR_RED_R_OVER_G and (r / b) > config.COLOR_RED_R_OVER_B:
            return "RED"
            
        # If it fails the red check, it's not red
        return "NOT RED"