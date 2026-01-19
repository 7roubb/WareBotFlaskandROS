"""
Enumerations for WareBot Task Runner
"""
from enum import Enum


class TaskState(Enum):
    """Task execution states"""
    IDLE = "IDLE"
    ASSIGNED = "ASSIGNED"
    MOVING_TO_PICKUP = "MOVING_TO_PICKUP"
    ARRIVED_AT_PICKUP = "ARRIVED_AT_PICKUP"
    ALIGNING = "ALIGNING"
    ALIGNED = "ALIGNED"
    MOVING_TO_DROP = "MOVING_TO_DROP"
    ARRIVED_AT_DROP = "ARRIVED_AT_DROP"
    RELEASED = "RELEASED"
    MOVING_TO_REFERENCE = "MOVING_TO_REFERENCE"
    COMPLETED = "COMPLETED"
    ERROR = "ERROR"