"""
WareBotTaskNavigator - ROS2 package for robot task navigation
Provides task assignment, movement execution, and reference point management
"""

__version__ = '0.0.1'
__author__ = 'WareBotTeam'

from .task_navigator import TaskNavigator
from .task_executor import TaskExecutor
from .reference_point_manager import ReferencePointManager

__all__ = [
    'TaskNavigator',
    'TaskExecutor',
    'ReferencePointManager',
]
