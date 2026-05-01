from machine import Pin
import time
import config

pin = Pin(config.PIN_KICK, Pin.OUT, Pin.PULL_DOWN)
_last_kick_ms = 0


def kick(duration_ms=None, cooldown_ms=None):
    """
    Fire the kicker for a short pulse.
    Cooldown avoids repeated rapid firing.
    """
    global _last_kick_ms

    if duration_ms is None:
        duration_ms = config.KICK_PULSE_MS
    if cooldown_ms is None:
        cooldown_ms = config.KICK_COOLDOWN_MS

    now = time.ticks_ms()
    if time.ticks_diff(now, _last_kick_ms) < cooldown_ms:
        return False

    try:
        pin.value(1)
        time.sleep_ms(duration_ms)
        pin.value(0)
        _last_kick_ms = time.ticks_ms()
        return True
    finally:
        pin.value(0)