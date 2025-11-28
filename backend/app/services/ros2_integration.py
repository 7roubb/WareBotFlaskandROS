"""
ROS2 Integration Service - Bridges Flask backend with ROS2 robot control
Handles task assignment and publishes to ROS2 topics via MQTT
"""

from typing import Dict, Any, Optional
from datetime import datetime
from flask import current_app
import json
import logging
from ..core import TaskStatus, RobotStatus


class ROS2IntegrationService:
    """
    Service for integrating Flask backend with ROS2 robot control
    - Publishes task assignments to ROS2
    - Receives robot status updates
    - Manages reference points
    """

    def __init__(self):
        self.logger = logging.getLogger('ROS2IntegrationService')
        self.mqtt_client = None

    def set_mqtt_client(self, mqtt_client):
        """Set the MQTT client for publishing to ROS2"""
        self.mqtt_client = mqtt_client
        self.logger.info('MQTT client set for ROS2 integration')

    def assign_task_to_robot(
        self, task_id: str, robot_id: str, target_x: float, target_y: float,
        target_yaw: float = 0.0, priority: int = 0
    ) -> bool:
        """
        Assign a task to a robot and publish to ROS2
        
        Args:
            task_id: Unique task identifier
            robot_id: Target robot identifier
            target_x: Target X coordinate (meters)
            target_y: Target Y coordinate (meters)
            target_yaw: Target orientation (radians)
            priority: Task priority (0=lowest, 10=highest)
            
        Returns:
            True if task was published successfully
        """
        try:
            if not self.mqtt_client:
                self.logger.error('MQTT client not configured')
                return False

            # Prepare task assignment message
            task_msg = {
                'task_id': task_id,
                'robot_id': robot_id,
                'target_x': target_x,
                'target_y': target_y,
                'target_yaw': target_yaw,
                'priority': priority,
                'timestamp': datetime.utcnow().isoformat()
            }

            # Publish to ROS2 task assignment topic via MQTT
            topic = 'ros2/task/assignment'
            self.mqtt_client.publish(topic, json.dumps(task_msg), qos=1)

            self.logger.info(
                f'Task assigned via ROS2: {task_id} -> {robot_id} '
                f'to ({target_x}, {target_y})'
            )
            return True

        except Exception as e:
            self.logger.error(f'Failed to assign task via ROS2: {e}')
            return False

    def set_robot_reference_point(
        self, robot_id: str, x: float, y: float, yaw: float = 0.0,
        description: str = ""
    ) -> bool:
        """
        Set the reference point (home/dock) for a robot
        
        Args:
            robot_id: Robot identifier
            x: Home X coordinate
            y: Home Y coordinate
            yaw: Home orientation
            description: Reference point description
            
        Returns:
            True if reference point was set successfully
        """
        try:
            if not self.mqtt_client:
                self.logger.error('MQTT client not configured')
                return False

            # Prepare reference point message
            ref_msg = {
                'command': 'set',
                'robot_id': robot_id,
                'x': x,
                'y': y,
                'yaw': yaw,
                'description': description or f'Reference point for {robot_id}'
            }

            # Publish to ROS2 reference point topic
            topic = 'ros2/reference_point/command'
            self.mqtt_client.publish(topic, json.dumps(ref_msg), qos=1)

            self.logger.info(
                f'Reference point set for {robot_id}: ({x}, {y})'
            )
            return True

        except Exception as e:
            self.logger.error(f'Failed to set reference point: {e}')
            return False

    def get_robot_reference_point(self, robot_id: str) -> Optional[Dict[str, Any]]:
        """
        Get the reference point for a robot
        
        Args:
            robot_id: Robot identifier
            
        Returns:
            Reference point data or None
        """
        try:
            if not self.mqtt_client:
                return None

            # Query reference point (would need ROS2 service bridge)
            ref_msg = {
                'command': 'get',
                'robot_id': robot_id
            }

            topic = 'ros2/reference_point/command'
            self.mqtt_client.publish(topic, json.dumps(ref_msg), qos=1)

            return {
                'robot_id': robot_id,
                'note': 'Reference point query published. Check ROS2 logs for response.'
            }

        except Exception as e:
            self.logger.error(f'Failed to get reference point: {e}')
            return None

    def start_robot_task_execution(self, robot_id: str) -> bool:
        """
        Signal ROS2 to start executing tasks for a robot
        
        Args:
            robot_id: Robot identifier
            
        Returns:
            True if signal was sent successfully
        """
        try:
            if not self.mqtt_client:
                self.logger.error('MQTT client not configured')
                return False

            msg = {
                'command': 'start',
                'robot_id': robot_id,
                'timestamp': datetime.utcnow().isoformat()
            }

            topic = f'ros2/robot/{robot_id}/command'
            self.mqtt_client.publish(topic, json.dumps(msg), qos=1)

            self.logger.info(f'Task execution started for {robot_id}')
            return True

        except Exception as e:
            self.logger.error(f'Failed to start task execution: {e}')
            return False

    def stop_robot_task_execution(self, robot_id: str) -> bool:
        """
        Signal ROS2 to stop executing tasks for a robot
        
        Args:
            robot_id: Robot identifier
            
        Returns:
            True if signal was sent successfully
        """
        try:
            if not self.mqtt_client:
                self.logger.error('MQTT client not configured')
                return False

            msg = {
                'command': 'stop',
                'robot_id': robot_id,
                'timestamp': datetime.utcnow().isoformat()
            }

            topic = f'ros2/robot/{robot_id}/command'
            self.mqtt_client.publish(topic, json.dumps(msg), qos=1)

            self.logger.info(f'Task execution stopped for {robot_id}')
            return True

        except Exception as e:
            self.logger.error(f'Failed to stop task execution: {e}')
            return False


# Singleton instance
_ros2_service: Optional[ROS2IntegrationService] = None


def get_ros2_service() -> ROS2IntegrationService:
    """Get or create ROS2 integration service singleton"""
    global _ros2_service
    if _ros2_service is None:
        _ros2_service = ROS2IntegrationService()
    return _ros2_service
