"""
Helper functions for WareBot Task Runner
"""
import math


def apply_min(cmd, min_mag):
    """Apply minimum magnitude to command"""
    if abs(cmd) < 1e-6:
        return 0.0
    return math.copysign(max(min_mag, abs(cmd)), cmd)