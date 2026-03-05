from machine import Pin
import time

# --- Pin setup ---
s0 = Pin(2, Pin.OUT)
s1 = Pin(3, Pin.OUT)
s2 = Pin(4, Pin.OUT)
s3 = Pin(5, Pin.OUT)
out = Pin(6, Pin.IN)

# Set frequency scaling to 20% (good balance of speed and accuracy)
# S0=HIGH, S1=LOW = 20% scaling
s0.value(1)
s1.value(0)

# --- Read frequency for a given color filter ---

def read_frequency(filter_s2, filter_s3, sample_ms=100):
    """
        Sets the color filter via S2/S3, then counts pulses
    on the OUT pin over sample_ms milliseconds.
    Returns the pulse count (proportional to frequency).
    """
    s2.value(filter_s2)
    s3.value(filter_s3)
    time.sleep_ms(20)  # Let filter settle

    count = 0
    # Wait for a falling edge to sync up
    while out.value() == 0:
        pass
    while out.value() == 1:
        pass

    # Count falling edges for the sample window
    start = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), start) < sample_ms:
        while out.value() == 0:
            if time.ticks_diff(time.ticks_ms(), start) >= sample_ms:
                return count
        while out.value() == 1:
            if time.ticks_diff(time.ticks_ms(), start) >= sample_ms:
                return count
        count += 1

    return count

def read_colors():
    """
    Reads all three color channels.
    Filter selection (S2, S3):
      Red:   S2=LOW,  S3=LOW
      Green: S2=HIGH, S3=HIGH
      Blue:  S2=LOW,  S3=HIGH
    
    Returns (red, green, blue) pulse counts.
    Higher count = more of that color detected.
    """
    r = read_frequency(0, 0)
    g = read_frequency(1, 1)
    b = read_frequency(0, 1)
    return r, g, b

# --- Red detection logic ---

def is_red(r, g, b):
    """
    Returns True if the ball is RED (meaning: AVOID it, -2 points).
    Returns False if the ball is green/blue (meaning: GO FOR it).
    
    Tune the thresholds with your actual ping pong balls!
    """
    if r == 0:
        return False

    # Red ball: red channel significantly higher than green and blue
    red_over_green = r > g * 1.4
    red_over_blue = r > b * 1.4
    min_threshold = r > 10

    return red_over_green and red_over_blue and min_threshold

# --- Main loop ---

def main():
    print("TCS3200 initialized. Scanning for RED balls...\n")

    while True:
        r, g, b = read_colors()

        if is_red(r, g, b):
            print(f"RED — IGNORE  | R={r} G={g} B={b}")
            # TODO: Skip this ball, keep searching for green/blue
        else:
            print(f"GREEN/BLUE — GO! | R={r} G={g} B={b}")
            # TODO: Drive toward ball, push it into opponent's goal

        time.sleep_ms(200)

main()
