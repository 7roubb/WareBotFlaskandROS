"""
Enhanced Constants for WareBot Task Runner with 4-Tag Alignment
"""

# ============================================================================
# AprilTag Configuration
# ============================================================================

TAG_SIZE = 0.05  # 6.8cm tags
TOLERANCE_X = 0.2 # 8cm strafe tolerance
TOLERANCE_Z = 0.1# 8cm forward tolerance
TOLERANCE_YAW = 5.0  # 7 degree rotation tolerance

# 4-Tag Grid Configuration
EXPECTED_TAG_IDS = [294, 323, 420, 530]  # [TOP-LEFT, TOP-RIGHT, BOTTOM-LEFT, BOTTOM-RIGHT]
TAG_GRID_SPACING = 0.14  # Expected spacing between tags (meters)

# Multi-tag centering parameters
CENTER_TOLERANCE_PIXELS = 20  # pixels for grid center tolerance
MOVEMENT_THRESHOLD_PIXELS = 10  # pixels threshold for movement commands

# ============================================================================
# PID Controller Parameters - Standard
# ============================================================================

PID_YAW_KP = 0.06
PID_YAW_KI = 0.0
PID_YAW_KD = 0.020
PID_YAW_I_LIMIT = 0.5
PID_YAW_OUT_LIMIT = 0.35

PID_STRAFE_KP = 1.2
PID_STRAFE_KI = 0.0
PID_STRAFE_KD = 0.25
PID_STRAFE_I_LIMIT = 0.4
PID_STRAFE_OUT_LIMIT = 0.15

PID_FORWARD_KP = 1.2
PID_FORWARD_KI = 0.0
PID_FORWARD_KD = 0.25
PID_FORWARD_I_LIMIT = 0.4
PID_FORWARD_OUT_LIMIT = 0.10

# ============================================================================
# PID Controller Parameters - Enhanced for 4-Tag (Auto-scaled in controller)
# ============================================================================
# These are applied as multipliers to standard PID gains:
# - Enhanced Yaw: Kp * 1.1, Kd * 1.05, OutLimit * 1.1
# - Enhanced Strafe: Kp * 1.05, OutLimit * 1.05
# - Enhanced Forward: Kp * 1.05, OutLimit * 1.05
# This provides smoother, more responsive control for multi-tag alignment

PID_ENHANCEMENT_MULTIPLIER = 1.05  # General enhancement multiplier

# ============================================================================
# Minimum Command Magnitudes
# ============================================================================

MIN_WZ = 0.05  # Minimum angular velocity
MIN_VY = 0.02  # Minimum strafe velocity
MIN_VX = 0.02  # Minimum forward velocity

# ============================================================================
# AprilTag Object Points (Single Tag)
# ============================================================================
# For 0.068m tags (6.8cm)

APRILTAG_OBJ_POINTS = [
    [-0.034,  0.034, 0],  # Top-left
    [ 0.034,  0.034, 0],  # Top-right
    [ 0.034, -0.034, 0],  # Bottom-right
    [-0.034, -0.034, 0],  # Bottom-left
]

# ============================================================================
# Timing Parameters
# ============================================================================

ACTUATOR_EXTENSION_TIME = 22.0  # Time for actuators to fully extend (seconds)
ACTUATOR_RETRACTION_TIME = 22.0  # Time for actuators to fully retract (seconds)
ARDUINO_RESET_TIME = 2.5  # Time to wait after Arduino reset (seconds)

# Alignment timing
ALIGNMENT_HOLD_TIME_DEFAULT = 30.0  # Default hold time after alignment (seconds)
LOST_TAG_TIMEOUT = 0.7  # Timeout before losing tag track (seconds)

# ============================================================================
# Control Loop Parameters
# ============================================================================

CONTROL_LOOP_RATE = 0.05  # Control loop period (20Hz = 0.05s)
ALIGNMENT_FILTER_ALPHA = 0.3  # Low-pass filter coefficient for position smoothing
ALIGNMENT_LOG_INTERVAL = 10  # Log every N frames

# ============================================================================
# Status Publishing
# ============================================================================

MAX_STATUS_RETRIES = 10  # Maximum retries for status publishing
STATUS_PUBLISH_INTERVAL = 0.5  # Status publish interval (seconds)

# ============================================================================
# Phase Control for 4-Tag Alignment
# ============================================================================

# Phase 1: CENTERING - when less than 4 tags visible
# - Goal: Center grid center on camera optical axis
# - Uses standard PID gains
# - Exits when all 4 tags visible or grid is centered

# Phase 2: ALIGNMENT - when 3+ tags visible
# - Goal: Fine-tune position/orientation for perfect alignment
# - Uses enhanced PID gains for smoother response
# - Stricter tolerances (75% of centering tolerances)
# - Actuators extend only during perfect alignment

PHASE_TRANSITION_TAG_COUNT = 3  # Min tags required to enter alignment phase

# ============================================================================
# Camera Calibration
# ============================================================================

CALIBRATION_FILE = "calibration.npy"  # Default calibration file path

# ============================================================================
# Debug and Logging
# ============================================================================

SHOW_DEBUG_IMAGE = True  # Whether to display debug images (set to False for headless)
ALIGNMENT_VERBOSE = True  # Verbose alignment logging

# ============================================================================
# Task Runner Configuration
# ============================================================================

# Footprint management
FOOTPRINT_ORIGINAL_WIDTH = 0.6  # meters
FOOTPRINT_ORIGINAL_HEIGHT = 0.6  # meters
FOOTPRINT_EXTENDED_WIDTH = 1.0  # meters
FOOTPRINT_EXTENDED_HEIGHT = 1.0  # meters

# Costmap switching
SWITCH_FOOTPRINT_AT_PICKUP = False  # Switch footprint when arriving at pickup
SWITCH_FOOTPRINT_AT_DROP = True  # Switch to extended footprint when moving to drop
SWITCH_BACK_AFTER_RETRACT = True  # Switch back to original after retraction