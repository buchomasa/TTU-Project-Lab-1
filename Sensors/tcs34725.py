from machine import I2C
import time

# Default I2C address
_TCS34725_ADDR = 0x29

# Registers
_ENABLE  = 0x00
_ATIME   = 0x01
_CONTROL = 0x0F
_CDATA   = 0x14

# Command bit
_CMD = 0x80

# Enable register bits
_PON = 0x01   # Power ON
_AEN = 0x02   # ADC enable


class TCS34725:
    def __init__(self, i2c, addr=_TCS34725_ADDR,
                 integration=0xEB, gain=0x01):
        """
        integration:
            0xFF = 2.4ms
            0xF6 = 24ms
            0xEB = 50ms (default)
            0xD5 = 101ms
            0xC0 = 154ms
            0x00 = 700ms

        gain:
            0x00 = 1x
            0x01 = 4x (default)
            0x02 = 16x
            0x03 = 60x
        """
        self.i2c = i2c
        self.addr = addr

        # Power on
        self._write8(_ENABLE, _PON)
        time.sleep_ms(3)

        # Enable ADC
        self._write8(_ENABLE, _PON | _AEN)

        # Set timing & gain
        self._write8(_ATIME, integration)
        self._write8(_CONTROL, gain)

    # ---------- low level ----------
    def _write8(self, reg, val):
        self.i2c.writeto_mem(self.addr, _CMD | reg, bytes([val]))

    def _read16(self, reg):
        data = self.i2c.readfrom_mem(self.addr, _CMD | reg, 2)
        return data[1] << 8 | data[0]

    # ---------- public API ----------
    def read(self):
        """Return (r, g, b, clear)"""
        c = self._read16(_CDATA)
        r = self._read16(_CDATA + 2)
        g = self._read16(_CDATA + 4)
        b = self._read16(_CDATA + 6)
        return r, g, b, c

    def raw(self):
        """Return dict with raw channels"""
        r, g, b, c = self.read()
        return {"r": r, "g": g, "b": b, "c": c}

    def disable(self):
        """Power down sensor (lowest power)"""
        self._write8(_ENABLE, 0x00)

    def enable(self):
        """Re-enable sensor after disable"""
        self._write8(_ENABLE, _PON)
        time.sleep_ms(3)
        self._write8(_ENABLE, _PON | _AEN)

    def set_gain(self, gain):
        self._write8(_CONTROL, gain)

    def set_integration(self, integration):
        self._write8(_ATIME, integration)