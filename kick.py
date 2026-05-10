from machine import Pin
import time
import config

pin = Pin(config.PIN_KICK, Pin.OUT, Pin.PULL_DOWN)


def kick(duration_ms=None):
    """
    Fire solenoid kicker with a sustained pulse.
    Uses direct hold-high timing similar to standalone working test.
    """
    if duration_ms is None:
        duration_ms = config.KICK_PULSE_MS

    try:
        pin.value(1)
        # Using sleep(seconds) mirrors the known-good standalone script behavior.
        time.sleep(duration_ms / 1000.0)
        pin.value(0)
        return True
    finally:
        pin.value(0)