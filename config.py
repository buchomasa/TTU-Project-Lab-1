"""
Centralized rover calibration and hardware configuration.
Adjust this file during field tuning.
"""

# ---------------------------
# Hardware Pins
# ---------------------------

# L298N + overcurrent board
PIN_MOTOR_IN1 = 4
PIN_MOTOR_IN2 = 5
PIN_MOTOR_IN3 = 6
PIN_MOTOR_IN4 = 7
PIN_MOTOR_ENA = 3
PIN_MOTOR_ENB = 2
PIN_OC_A = 15
PIN_OC_B = 14

# Front sweeping ultrasonic
PIN_FRONT_TRIG = 12
PIN_FRONT_ECHO = 8
PIN_FRONT_SWEEP_SERVO = 10

# Rear ultrasonic
PIN_REAR_TRIG = 17
PIN_REAR_ECHO = 18

# ToF (VL53L1X) on I2C0
TOF_I2C_BUS = 0
PIN_TOF_SDA = 0
PIN_TOF_SCL = 1
PIN_TOF_INT = 19

# Color sensor on I2C1
COLOR_I2C_BUS = 1
PIN_COLOR_SDA = 20
PIN_COLOR_SCL = 21
COLOR_SENSOR_ADDR = 0x29
COLOR_USE_SOFT_I2C = True
COLOR_I2C_FREQ = 100000

# Claw servos
PIN_CLAW_SERVO_A = 9
PIN_CLAW_SERVO_B = 22

# Solenoid MOSFET gate
PIN_KICK = 11

# Encoders
PIN_ENCODER_LEFT = 28
PIN_ENCODER_RIGHT = 16

# Line sensors (ADC)
PIN_LINE_BLACK = 27
PIN_LINE_BLUE = 26


# ---------------------------
# Motion + Avoidance Tuning
# ---------------------------

OBSTACLE_THRESHOLD_CM = 20.0
REAR_THRESHOLD_CM = 20.0
ULTRASONIC_SAMPLES = 5
ULTRASONIC_MIN_CM = 2.0
ULTRASONIC_MAX_CM = 300.0

FWD_LEFT_SPEED = 51.2
FWD_RIGHT_SPEED = 60.0
TURN_LEFT_SPEED = 70
TURN_RIGHT_SPEED = 60

TRACK_WIDTH_CM = 20.0
WHEEL_DIAMETER_CM = 6.0
DISK_SLOTS = 20

# Main loop and anti-stuck controls
MAIN_LOOP_MS = 20
STATE_TIMEOUT_MS = 4000
CAPTURE_VERIFY_TIMEOUT_MS = 1800
RECOVERY_REVERSE_MS = 800
RECOVERY_TURN_DEG = 50
RECOVERY_MAX_CONSECUTIVE_FAILS = 3
RECOVERY_LINE_ESCAPE_ATTEMPTS = 3
RECOVERY_LINE_ESCAPE_REVERSE_MS = 350
RECOVERY_LINE_ESCAPE_TURN_DEG = 35


# ---------------------------
# ToF Ball Logic
# ---------------------------

# Stop rover and close claw when ToF sees <= this value
TOF_CAPTURE_MM = 60

# Ball considered inside box when very close to zero
TOF_SECURED_MM = 5

# Optional long-range detect/approach hint
TOF_APPROACH_MM = 400
# Ignore invalid negative compensated values for approach.
TOF_MIN_VALID_MM = 0
TOF_CLEAR_MM = 60
TOF_CLEAR_COUNT = 3
# Enter controlled creep before capture trigger to avoid overshoot.
TOF_APPROACH_MID_MM = 150
TOF_APPROACH_SLOW_MM = 95
APPROACH_CREEP_LEFT_SPEED = 30.0
APPROACH_CREEP_RIGHT_SPEED = 34.0
APPROACH_MID_LEFT_SPEED = 44.0
APPROACH_MID_RIGHT_SPEED = 50.0

# Strategy-critical distances and timing
START_DELAY_ENABLE_MS = 3000
START_FORWARD_DISTANCE_CM = 100.0
TOF_SCAN_MAX_MM = 1000
TOF_ALIGN_STOP_MM = 80
TOF_FINAL_CAPTURE_MM = 45
TOF_SECURE_CONFIRM_MM = 5
OPPONENT_GOAL_DISTANCE_CM = 304.8  # ~10 ft
MICRO_ALIGN_TURN_DEG = 4
MICRO_ALIGN_MAX_TRIES = 8

# Search pattern behavior
START_SCAN_TURN_DEG = 90
SCAN_SETTLE_MS = 180
SCAN_SWEEP_RECHECK_MS = 120
SCAN_FORWARD_BUMP_MS = 260
SEARCH_STEP_DISTANCE_CM = 5.0

# Rover-vs-ball proximity discrimination
ROVER_PROX_ULTRA_CM = 22.0
ROVER_PROX_TOF_MM = 220
STUCK_REVERSE_MS = 500
OBSTACLE_STUCK_LIMIT = 2

# Boundary-evasion profile (ported from boundary_live_drive.py behavior)
BOUNDARY_REVERSE_SPEED = 45
BOUNDARY_TURN_SPEED = 52
BOUNDARY_REVERSE_MS = 350
BOUNDARY_TURN_MS_SINGLE = 420
BOUNDARY_TURN_MS_BOTH = 700

# Current mechanical offset compensation.
# Tune so a ball fully inside the box reads near 0mm.
TOF_OFFSET_MM = 26


# ---------------------------
# Claw + Kicker Timing
# ---------------------------

CLAW_SETTLE_MS = 250
CLAW_SERVO_PULSE_MS = 120
KICK_PULSE_MS = 90
KICK_COOLDOWN_MS = 350


# ---------------------------
# Line Sensor Thresholds
# ---------------------------

LINE_BLACK_THRESHOLD = 9000
LINE_LEFT_BLACK_THRESHOLD = 12000
LINE_RIGHT_BLACK_THRESHOLD = 9000
LINE_STOP_DEBOUNCE_COUNT = 1

# Thin boundary lines can be missed on a single read while moving.
# Sample multiple times quickly and trigger if any sample crosses threshold.
LINE_SAMPLE_COUNT = 24
LINE_SAMPLE_DELAY_US = 0

# Adaptive line sensing (recommended for varying lighting/floor reflectance).
# Robot should start on normal field floor (not on black tape) for calibration.
LINE_USE_ADAPTIVE = False
LINE_CALIBRATION_SAMPLES = 60
LINE_CALIBRATION_DELAY_MS = 5
LINE_LEFT_MARGIN = 1800
LINE_RIGHT_MARGIN = 2500

# If ToF no longer sees a candidate for N loops, drop APPROACH and rescan.
TOF_LOST_COUNT = 5


# ---------------------------
# Color Sensor Thresholds
# ---------------------------

COLOR_INTEGRATION_TIME = 0xD5
COLOR_GAIN = 0x01
COLOR_RED_R_OVER_G = 1.35
COLOR_RED_R_OVER_B = 1.30
COLOR_SAMPLES = 5

# ---------------------------
# Serial Debug Telemetry
# ---------------------------
DEBUG_ENABLE = True
DEBUG_PERIOD_MS = 250

# ---------------------------
# Test Mode Controls
# ---------------------------
# Keep both True on the real field.
# For suspended bench tests, you can disable these to reduce false events.
ENABLE_LINE_SAFETY = True
ENABLE_OBSTACLE_AVOID = True

# Motor Speeds (0-100%)
FWD_SPEED_L = 51.2
FWD_SPEED_R = 60
TURN_SPEED_L = 85
TURN_SPEED_R = 85
ALIGN_SPEED_L = 70
ALIGN_SPEED_R = 70

# Distance & Target Thresholds (mm)
DETECT_THRESHOLD_MM = 200       
STOP_THRESHOLD_MM = 70          
SLOWDOWN_THRESHOLD_MM = 120     
CAPTURE_SUCCESS_MM = 25         

# Sensor Validation
FALSE_TRIGGER_MARGIN_MM = 30    
MAX_LOST_FRAMES = 4             

# Mission Navigation Parameters
START_FORWARD_DIST_CM = 150.0   
SWEEP_ARC_HALF = 120            
REPOSITION_DIST_CM = 10.0       
BOUNDARY_TURN_PULSES = 30.0     
BOUNDARY_REVERSE_CM = 20.0      

# Operational Factors
SCAN_SPEED_FACTOR = 0.85        
SLOWDOWN_FACTOR = 0.70          
SLOW_FWD_LEFT = FWD_SPEED_L * SLOWDOWN_FACTOR
SLOW_FWD_RIGHT = FWD_SPEED_R * SLOWDOWN_FACTOR

# System Timing
SETTLE_TIME_S = 0.3             
CLAW_WAIT_TIME_S = 1            
NUDGE_PULSES = 1
