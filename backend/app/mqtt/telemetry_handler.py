"""
MQTT telemetry handler - Processes robot telemetry messages
"""
import json
from flask import current_app
from .base_handler import BaseMQTTHandler
from ..services.robot_service import get_robot_service
from ..events.socket_events import emit_robot_telemetry


class TelemetryHandler(BaseMQTTHandler):
    """Handles robot telemetry MQTT messages"""
    
    def __init__(self):
        super().__init__("TelemetryHandler")
        self.robot_service = get_robot_service()
    
    def handle(self, topic: str, payload: str) -> None:
        """
        Handle telemetry message
        Topic pattern: robots/mp400/<robot>/status
        """
        try:
            # Extract robot name from topic: robots/mp400/{robot_name}/status
            parts = topic.split("/")
            if len(parts) != 4:
                self.log_error(f"Invalid telemetry topic: {topic}")
                return
            
            robot_name = parts[2]
            data = json.loads(payload)
            
            # Validate telemetry data
            required_fields = ["x", "y", "battery_level", "cpu_usage", "ram_usage", "temperature", "status"]
            if not all(field in data for field in required_fields):
                self.log_error(f"Missing required telemetry fields from {robot_name}")
                return
            
            self.log_debug(f"Processing telemetry from {robot_name}")
            
            # Update robot in database
            self.robot_service.update_robot_telemetry(robot_name, data)
            
            # Emit to WebSocket - include multiple ID fields for robust frontend matching
            robot = self.robot_service.repo.find_by_robot_id(robot_name)
            if robot:
                emit_payload = {
                    "robot": robot_name,
                    "name": robot.get("name"),
                    "robot_id": robot.get("robot_id"),
                    "id": str(robot.get("_id")),
                    **data,
                }
                emit_robot_telemetry(emit_payload)
                self.log_debug(f"Telemetry emitted for {robot_name}")
            
        except json.JSONDecodeError:
            self.log_error(f"Invalid JSON in telemetry message from {topic}")
        except Exception as e:
            self.log_error(f"Error handling telemetry from {topic}", e)
