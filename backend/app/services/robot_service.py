"""
Robot service - handles robot CRUD and telemetry updates
Uses STATUS field (IDLE/BUSY/ERROR/OFFLINE) instead of available.
"""
from datetime import datetime
from bson import ObjectId
from typing import Optional, List, Dict, Any

from ..extensions import get_db


def update_robot_telemetry(robot_name: str, telemetry: dict) -> bool:
    """
    Update robot telemetry AND position in MongoDB.
    
    CRITICAL: This updates BOTH telemetry metrics AND position coordinates
    so that robots.list() API returns live position data.
    
    Args:
        robot_name: The robot_id from MQTT topic
        telemetry: Dict containing cpu_usage, ram_usage, battery_level, temperature, x, y, yaw, status
    
    Returns:
        True if update successful, False otherwise
    """
    db = get_db()
    
    # Handle None values safely
    x = telemetry.get("x")
    y = telemetry.get("y")
    yaw = telemetry.get("yaw")
    
    update_data = {
        "updated_at": datetime.utcnow(),
        
        # Telemetry metrics
        "cpu_usage": float(telemetry.get("cpu_usage", 0)),
        "ram_usage": float(telemetry.get("ram_usage", 0)),
        "battery_level": float(telemetry.get("battery_level", 0)),
        "temperature": float(telemetry.get("temperature", 0)),
        
        # ✓ CRITICAL: Update position in MongoDB with None safety
        "current_x": float(x) if x is not None else 0.0,
        "current_y": float(y) if y is not None else 0.0,
        "current_yaw": float(yaw) if yaw is not None else 0.0,
        
        # Status
        "status": telemetry.get("status", "IDLE"),
    }
    
    # Update robot by robot_id (NOT robot_name)
    result = db.robots.update_one(
        {"robot_id": robot_name, "deleted": False},
        {"$set": update_data}
    )
    
    return result.modified_count > 0


def list_robots() -> List[Dict[str, Any]]:
    """
    List all non-deleted robots with position data.
    
    Returns robots with BOTH x/y/yaw AND current_x/current_y/current_yaw
    for frontend compatibility.
    
    Returns:
        List of robot dictionaries
    """
    db = get_db()
    robots = list(db.robots.find({"deleted": False}))
    
    result = []
    for robot in robots:
        # Convert _id to id
        robot_dict = {
            "id": str(robot["_id"]),
            "name": robot.get("name"),
            "robot_id": robot.get("robot_id"),
            "topic": robot.get("topic"),
            "status": robot.get("status", "IDLE"),  # Primary status field
            "current_shelf_id": robot.get("current_shelf_id"),
            
            # ✓ Position fields (primary storage format)
            "current_x": float(robot.get("current_x", 0)),
            "current_y": float(robot.get("current_y", 0)),
            "current_yaw": float(robot.get("current_yaw", 0)),
            
            # ✓ Alias for frontend compatibility (x/y/yaw)
            "x": float(robot.get("current_x", 0)),
            "y": float(robot.get("current_y", 0)),
            "yaw": float(robot.get("current_yaw", 0)),
            
            # ✓ Telemetry data
            "cpu_usage": float(robot.get("cpu_usage", 0)),
            "ram_usage": float(robot.get("ram_usage", 0)),
            "battery_level": float(robot.get("battery_level", 0)),
            "temperature": float(robot.get("temperature", 0)),
            
            # Metadata
            "created_at": robot.get("created_at"),
            "updated_at": robot.get("updated_at"),
        }
        
        result.append(robot_dict)
    
    return result


def get_robot(robot_id: str) -> Optional[Dict[str, Any]]:
    """
    Get single robot with position data.
    
    Args:
        robot_id: Robot ID (ObjectId string or robot_id field)
    
    Returns:
        Robot dictionary or None if not found
    """
    db = get_db()
    
    # Try by ObjectId first
    try:
        oid = ObjectId(robot_id)
        robot = db.robots.find_one({"_id": oid, "deleted": False})
    except:
        # Try by robot_id field
        robot = db.robots.find_one({"robot_id": robot_id, "deleted": False})
    
    if not robot:
        return None
    
    return {
        "id": str(robot["_id"]),
        "name": robot.get("name"),
        "robot_id": robot.get("robot_id"),
        "topic": robot.get("topic"),
        "status": robot.get("status", "IDLE"),
        "current_shelf_id": robot.get("current_shelf_id"),
        
        # Position (both formats)
        "current_x": float(robot.get("current_x", 0)),
        "current_y": float(robot.get("current_y", 0)),
        "current_yaw": float(robot.get("current_yaw", 0)),
        "x": float(robot.get("current_x", 0)),
        "y": float(robot.get("current_y", 0)),
        "yaw": float(robot.get("current_yaw", 0)),
        
        # Telemetry
        "cpu_usage": float(robot.get("cpu_usage", 0)),
        "ram_usage": float(robot.get("ram_usage", 0)),
        "battery_level": float(robot.get("battery_level", 0)),
        "temperature": float(robot.get("temperature", 0)),
        
        # Metadata
        "created_at": robot.get("created_at"),
        "updated_at": robot.get("updated_at"),
    }


def create_robot(data: dict) -> Dict[str, Any]:
    """
    Create a new robot with initial position.
    
    CRITICAL: Handles None values properly for position fields.
    
    Args:
        data: Robot creation data (name, robot_id, etc.)
    
    Returns:
        Created robot dictionary
    """
    db = get_db()
    
    # Handle None values for position fields
    current_x = data.get("current_x")
    current_y = data.get("current_y")
    current_yaw = data.get("current_yaw")
    
    robot_doc = {
        "name": data["name"],
        "robot_id": data["robot_id"],
        "status": data.get("status", "IDLE"),  # Use status instead of available
        "current_shelf_id": data.get("current_shelf_id"),
        
        # ✓ Initialize position with None safety
        "current_x": float(current_x) if current_x is not None else 0.0,
        "current_y": float(current_y) if current_y is not None else 0.0,
        "current_yaw": float(current_yaw) if current_yaw is not None else 0.0,
        
        # Initialize telemetry
        "cpu_usage": 0.0,
        "ram_usage": 0.0,
        "battery_level": 100.0,
        "temperature": 0.0,
        
        # Generate topic
        "topic": f"robots/mp400/{data['robot_id']}/status",
        
        # Metadata
        "deleted": False,
        "created_at": datetime.utcnow(),
        "updated_at": datetime.utcnow(),
    }
    
    result = db.robots.insert_one(robot_doc)
    robot_doc["id"] = str(result.inserted_id)
    robot_doc.pop("_id", None)
    
    # Add x/y/yaw aliases
    robot_doc["x"] = robot_doc["current_x"]
    robot_doc["y"] = robot_doc["current_y"]
    robot_doc["yaw"] = robot_doc["current_yaw"]
    
    return robot_doc


def update_robot(robot_id: str, data: dict) -> Optional[Dict[str, Any]]:
    """
    Update robot (excluding position - use telemetry for that).
    
    Args:
        robot_id: Robot ID (ObjectId string or robot_id field)
        data: Update data
    
    Returns:
        Updated robot dictionary or None if not found
    """
    db = get_db()
    
    # Try by ObjectId first
    try:
        oid = ObjectId(robot_id)
        query = {"_id": oid, "deleted": False}
    except:
        query = {"robot_id": robot_id, "deleted": False}
    
    update_data = {"updated_at": datetime.utcnow()}
    
    # Only allow updating these fields via this endpoint
    if "name" in data:
        update_data["name"] = data["name"]
    
    if "robot_id" in data:
        update_data["robot_id"] = data["robot_id"]
        update_data["topic"] = f"robots/mp400/{data['robot_id']}/status"
    
    if "status" in data:
        update_data["status"] = data["status"]
    
    if "current_shelf_id" in data:
        update_data["current_shelf_id"] = data["current_shelf_id"]
    
    # Handle position updates with None safety
    if "current_x" in data:
        current_x = data["current_x"]
        update_data["current_x"] = float(current_x) if current_x is not None else 0.0
    
    if "current_y" in data:
        current_y = data["current_y"]
        update_data["current_y"] = float(current_y) if current_y is not None else 0.0
    
    if "current_yaw" in data:
        current_yaw = data["current_yaw"]
        update_data["current_yaw"] = float(current_yaw) if current_yaw is not None else 0.0
    
    result = db.robots.update_one(query, {"$set": update_data})
    
    if result.matched_count == 0:
        return None
    
    return get_robot(robot_id)


def soft_delete_robot(robot_id: str) -> bool:
    """
    Soft delete a robot.
    
    Args:
        robot_id: Robot ID (ObjectId string or robot_id field)
    
    Returns:
        True if deleted, False if not found
    """
    db = get_db()
    
    try:
        oid = ObjectId(robot_id)
        query = {"_id": oid}
    except:
        query = {"robot_id": robot_id}
    
    result = db.robots.update_one(
        query,
        {"$set": {"deleted": True, "updated_at": datetime.utcnow()}}
    )
    
    return result.modified_count > 0


def update_robot_position(robot_id: str, x: float, y: float, yaw: float = 0.0) -> bool:
    """
    Update robot position directly.
    
    Args:
        robot_id: Robot ID (ObjectId string or robot_id field)
        x: X coordinate
        y: Y coordinate
        yaw: Yaw angle (default 0.0)
    
    Returns:
        True if successful, False otherwise
    """
    db = get_db()
    
    # Try by ObjectId first
    try:
        oid = ObjectId(robot_id)
        query = {"_id": oid, "deleted": False}
    except:
        query = {"robot_id": robot_id, "deleted": False}
    
    result = db.robots.update_one(
        query,
        {"$set": {
            "current_x": float(x),
            "current_y": float(y),
            "current_yaw": float(yaw),
            "updated_at": datetime.utcnow()
        }}
    )
    
    return result.modified_count > 0


def update_robot_battery(robot_id: str, battery_level: float) -> bool:
    """
    Update robot battery level.
    
    Args:
        robot_id: Robot ID (ObjectId string or robot_id field)
        battery_level: Battery level (0-100)
    
    Returns:
        True if successful, False otherwise
    """
    db = get_db()
    
    # Try by ObjectId first
    try:
        oid = ObjectId(robot_id)
        query = {"_id": oid, "deleted": False}
    except:
        query = {"robot_id": robot_id, "deleted": False}
    
    result = db.robots.update_one(
        query,
        {"$set": {
            "battery_level": float(battery_level),
            "updated_at": datetime.utcnow()
        }}
    )
    
    return result.modified_count > 0


def set_robot_status(robot_id: str, status: str) -> bool:
    """
    Set robot status (IDLE/BUSY/ERROR/OFFLINE).
    
    Args:
        robot_id: Robot ID (ObjectId string or robot_id field)
        status: New status (IDLE/BUSY/ERROR/OFFLINE)
    
    Returns:
        True if successful, False otherwise
    """
    db = get_db()
    
    valid_statuses = {"IDLE", "BUSY", "ERROR", "OFFLINE"}
    if status not in valid_statuses:
        return False
    
    # Try by ObjectId first
    try:
        oid = ObjectId(robot_id)
        query = {"_id": oid, "deleted": False}
    except:
        query = {"robot_id": robot_id, "deleted": False}
    
    result = db.robots.update_one(
        query,
        {"$set": {
            "status": status,
            "updated_at": datetime.utcnow()
        }}
    )
    
    return result.modified_count > 0


def get_robot_battery(robot_id: str) -> Optional[float]:
    """
    Get the current battery level for a robot.
    
    Args:
        robot_id: Robot ID (ObjectId string or robot_id field)
    
    Returns:
        Battery level or None if robot not found
    """
    robot = get_robot(robot_id)
    return robot.get("battery_level") if robot else None


def write_robot_telemetry_influx(robot_name: str, telemetry: dict) -> bool:
    """
    Write robot telemetry to InfluxDB time-series database.
    
    Args:
        robot_name: The robot_id from MQTT topic
        telemetry: Dict containing cpu_usage, ram_usage, battery_level, temperature, x, y, yaw, status
    
    Returns:
        True if write successful, False otherwise
    """
    try:
        from ..extensions import get_influx
        from influxdb_client import Point
        from flask import current_app
        
        influx_client, write_api = get_influx()
        
        if influx_client is None or write_api is None:
            return False
        
        # Handle None values safely
        x = telemetry.get("x")
        y = telemetry.get("y")
        yaw = telemetry.get("yaw")
        
        point = (
            Point("robot_telemetry")
            .tag("robot", robot_name)
            .field("cpu_usage", float(telemetry.get("cpu_usage", 0)))
            .field("ram_usage", float(telemetry.get("ram_usage", 0)))
            .field("battery_level", float(telemetry.get("battery_level", 0)))
            .field("temperature", float(telemetry.get("temperature", 0)))
            .field("x", float(x) if x is not None else 0.0)
            .field("y", float(y) if y is not None else 0.0)
            .field("yaw", float(yaw) if yaw is not None else 0.0)
        )
        
        write_api.write(
            bucket=current_app.config.get("INFLUX_BUCKET", "telemetry"),
            org=current_app.config.get("INFLUX_ORG", "warebot"),
            record=point
        )
        
        return True
    except Exception as e:
        try:
            from flask import current_app
            current_app.logger.error(f"[InfluxDB Write Error] {e}")
        except:
            pass
        return False