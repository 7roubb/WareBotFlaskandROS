"""
Robot service - Business logic for robot operations
"""
from typing import Dict, List, Any, Optional
from datetime import datetime
from bson import ObjectId
from flask import current_app

from .base_service import BaseService
from ..repositories.robot_repository import RobotRepository
from ..core import RobotStatus
from ..core.exceptions import ValidationError, NotFoundError, DuplicateError
from ..extensions import get_db, get_influx


class RobotService(BaseService):
    """Service for robot operations"""
    
    def __init__(self):
        self.repo = RobotRepository()
        super().__init__(self.repo, "RobotService")
    
    def create_robot(self, data: Dict[str, Any]) -> Dict[str, Any]:
        """Create a new robot"""
        self.log_debug(f"Creating robot: {data.get('robot_id')}")
        
        # Validate required fields
        self.validate_required_fields(data, ["robot_id", "name"])
        
        # Check for duplicates
        if self.repo.find_by_robot_id(data["robot_id"]):
            raise DuplicateError("Robot", "robot_id", data["robot_id"])
        
        # Prepare document
        robot_id = data["robot_id"].strip()
        doc = {
            "name": data["name"].strip(),
            "robot_id": robot_id,
            "topic": f"robots/mp400/{robot_id}/status",
            "available": data.get("available", True),
            "status": data.get("status", RobotStatus.IDLE.value),
            "current_task_id": data.get("current_task_id"),
            "current_shelf_id": data.get("current_shelf_id"),
            "cpu_usage": None,
            "ram_usage": None,
            "battery_level": None,
            "temperature": None,
            "x": None,
            "y": None,
            "deleted": False,
        }
        
        doc = self.add_timestamps(doc)
        robot_id = self.repo.create(doc)
        
        self.log_info(f"Robot created: {robot_id}")
        return self.get_robot(robot_id)
    
    def get_robot(self, robot_id: str) -> Dict[str, Any]:
        """Get robot by id"""
        robot = self.repo.find_by_id(robot_id)
        if not robot:
            raise NotFoundError("Robot", robot_id)
        return self.serialize_id(robot)
    
    def list_robots(self, page: int = 1, page_size: int = 20) -> Dict[str, Any]:
        """List all robots with pagination"""
        items, total = self.paginate({}, page, page_size)
        items = [self.serialize_id(r) for r in items]
        
        return {
            "results": items,
            "pagination": self.get_pagination_info(page, page_size, total)
        }
    
    def list_active_robots(self) -> List[Dict[str, Any]]:
        """Get all active robots"""
        robots = self.repo.find_active_robots()
        return [self.serialize_id(r) for r in robots]
    
    def list_available_robots(self) -> List[Dict[str, Any]]:
        """Get robots available for task assignment"""
        robots = self.repo.find_available_robots()
        return [self.serialize_id(r) for r in robots]
    
    def update_robot(self, robot_id: str, data: Dict[str, Any]) -> Dict[str, Any]:
        """Update robot information"""
        self.log_debug(f"Updating robot: {robot_id}")
        
        # Ensure robot exists
        self.get_robot(robot_id)
        
        # Remove readonly fields
        data.pop("_id", None)
        data.pop("created_at", None)
        data.pop("topic", None)
        
        # Add update timestamp
        data["updated_at"] = datetime.utcnow()
        
        # Update robot_id field if provided
        if "robot_id" in data and data["robot_id"] != robot_id:
            # Check for duplicates
            if self.repo.find_by_robot_id(data["robot_id"]):
                raise DuplicateError("Robot", "robot_id", data["robot_id"])
            data["topic"] = f"robots/mp400/{data['robot_id']}/status"
        
        self.repo.update_by_id(robot_id, data)
        self.log_info(f"Robot updated: {robot_id}")
        
        return self.get_robot(robot_id)
    
    def update_robot_telemetry(self, robot_name: str, telemetry: Dict[str, Any]) -> None:
        """Update robot telemetry from MQTT"""
        self.log_debug(f"Updating telemetry for {robot_name}")
        
        # Find robot by robot_id
        robot = self.repo.find_by_robot_id(robot_name)
        if not robot:
            self.log_error(f"Robot not found: {robot_name}")
            return
        
        robot_id = str(robot["_id"])
        
        # Prepare update data
        update_data = {
            "cpu_usage": float(telemetry.get("cpu_usage", 0)),
            "ram_usage": float(telemetry.get("ram_usage", 0)),
            "battery_level": float(telemetry.get("battery_level", 0)),
            "temperature": float(telemetry.get("temperature", 0)),
            "x": float(telemetry.get("x", 0)),
            "y": float(telemetry.get("y", 0)),
            "status": telemetry.get("status", robot.get("status")),
            "last_seen": datetime.utcnow(),
            "updated_at": datetime.utcnow(),
        }
        
        # Update in MongoDB
        self.repo.update_by_id(robot_id, update_data)
        
        # Write to InfluxDB if configured
        self._write_telemetry_to_influx(robot_name, update_data)
    
    def _write_telemetry_to_influx(self, robot_name: str, telemetry: Dict[str, Any]) -> None:
        """Write telemetry to InfluxDB"""
        try:
            influx_client, write_api = get_influx()
            if not write_api:
                return
            
            from influxdb_client import Point
            
            point = (
                Point("robot_telemetry")
                .tag("robot", robot_name)
                .field("cpu_usage", telemetry.get("cpu_usage", 0))
                .field("ram_usage", telemetry.get("ram_usage", 0))
                .field("battery_level", telemetry.get("battery_level", 0))
                .field("temperature", telemetry.get("temperature", 0))
                .field("x", telemetry.get("x", 0))
                .field("y", telemetry.get("y", 0))
                .field("status_code", RobotStatus.to_code(telemetry.get("status", "")))
            )
            
            write_api.write(
                bucket=current_app.config["INFLUX_BUCKET"],
                org=current_app.config["INFLUX_ORG"],
                record=point
            )
        except Exception as e:
            self.log_error(f"Failed to write telemetry to InfluxDB", e)
    
    def delete_robot(self, robot_id: str) -> bool:
        """Soft delete robot"""
        self.log_debug(f"Deleting robot: {robot_id}")
        
        # Ensure robot exists
        self.get_robot(robot_id)
        
        success = self.repo.delete_by_id(robot_id)
        if success:
            self.log_info(f"Robot deleted: {robot_id}")
        
        return success
    
    def get_robot_stats(self) -> Dict[str, Any]:
        """Get statistics about robots"""
        stats = self.repo.get_robot_stats()
        
        # Ensure all keys exist
        stats.setdefault("total", 0)
        stats.setdefault("offline", 0)
        stats.setdefault("idle", 0)
        stats.setdefault("moving", 0)
        stats.setdefault("avg_battery", None)
        stats.setdefault("avg_cpu", None)
        stats.setdefault("avg_ram", None)
        stats.setdefault("avg_temp", None)
        
        return stats
    
    def get_robots_in_area(self, x_min: float, x_max: float, y_min: float, y_max: float) -> List[Dict[str, Any]]:
        """Find robots in rectangular area"""
        robots = self.repo.find_robots_in_area(x_min, x_max, y_min, y_max)
        return [self.serialize_id(r) for r in robots]
    
    def get_low_battery_robots(self, threshold: float = 20.0) -> List[Dict[str, Any]]:
        """Get robots with low battery"""
        robots = self.repo.get_low_battery_robots(threshold)
        return [self.serialize_id(r) for r in robots]
    
    def assign_task(self, robot_id: str, task_id: str) -> Dict[str, Any]:
        """Assign task to robot"""
        self.log_debug(f"Assigning task {task_id} to robot {robot_id}")
        
        robot = self.get_robot(robot_id)
        
        update_data = {
            "current_task_id": task_id,
            "status": RobotStatus.BUSY.value,
            "updated_at": datetime.utcnow(),
        }
        
        self.repo.update_by_id(robot_id, update_data)
        self.log_info(f"Task {task_id} assigned to robot {robot_id}")
        
        return self.get_robot(robot_id)
    
    def complete_task(self, robot_id: str) -> Dict[str, Any]:
        """Mark robot task as complete"""
        self.log_debug(f"Completing task for robot {robot_id}")
        
        robot = self.get_robot(robot_id)
        
        update_data = {
            "current_task_id": None,
            "current_shelf_id": None,
            "status": RobotStatus.IDLE.value,
            "updated_at": datetime.utcnow(),
        }
        
        self.repo.update_by_id(robot_id, update_data)
        self.log_info(f"Task completed for robot {robot_id}")
        
        return self.get_robot(robot_id)


# Singleton instance
_robot_service: Optional[RobotService] = None


def get_robot_service() -> RobotService:
    """Get or create robot service singleton"""
    global _robot_service
    if _robot_service is None:
        _robot_service = RobotService()
    return _robot_service


# =========================================================
# BACKWARD COMPATIBILITY WRAPPERS
# =========================================================
def create_robot(data: Dict[str, Any]) -> Dict[str, Any]:
    """Create a new robot (backward compatible wrapper)"""
    return get_robot_service().create_robot(data)


def get_robot(robot_id: str) -> Dict[str, Any]:
    """Get robot by id (backward compatible wrapper)"""
    try:
        return get_robot_service().get_robot(robot_id)
    except NotFoundError:
        return None


def list_robots() -> List[Dict[str, Any]]:
    """List all robots (backward compatible wrapper)"""
    result = get_robot_service().list_robots(page=1, page_size=1000)
    return result.get("results", [])


def update_robot(robot_id: str, data: Dict[str, Any]) -> Dict[str, Any]:
    """Update robot (backward compatible wrapper)"""
    try:
        return get_robot_service().update_robot(robot_id, data)
    except NotFoundError:
        return None


def soft_delete_robot(robot_id: str) -> bool:
    """Soft delete robot (backward compatible wrapper)"""
    try:
        return get_robot_service().delete_robot(robot_id)
    except NotFoundError:
        return False


def update_robot_telemetry(robot_name: str, t: Dict[str, Any]) -> None:
    """Update robot telemetry (backward compatible wrapper)"""
    get_robot_service().update_robot_telemetry(robot_name, t)


def write_robot_telemetry_influx(robot_name: str, t: Dict[str, Any]) -> None:
    """Write telemetry to InfluxDB (backward compatible wrapper)"""
    get_robot_service()._write_telemetry_to_influx(robot_name, t)
