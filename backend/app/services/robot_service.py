from datetime import datetime
from bson import ObjectId
from influxdb_client import Point
from flask import current_app
from ..extensions import get_db, get_influx
from .utils_service import serialize


def create_robot(data: dict):
    db = get_db()
    now = datetime.utcnow()

    robot_id = data["robot_id"].strip()
    topic = f"robots/mp400/{robot_id}/status"

    doc = {
        "name": data["name"],
        "robot_id": robot_id,
        "topic": topic,
        "available": data.get("available", True),
        "status": data.get("status", "IDLE"),
        "current_shelf_id": data.get("current_shelf_id"),
        "cpu_usage": None,
        "ram_usage": None,
        "battery_level": None,
        "temperature": None,
        "x": None,
        "y": None,
        "created_at": now,
        "updated_at": now,
        "deleted": False,
    }

    res = db.robots.insert_one(doc)
    return serialize(db.robots.find_one({"_id": res.inserted_id}))


def list_robots():
    return [serialize(r) for r in get_db().robots.find({"deleted": False})]


def get_robot(id: str):
    try:
        oid = ObjectId(id)
    except:
        return None
    return serialize(get_db().robots.find_one({"_id": oid, "deleted": False}))


def update_robot(id: str, data: dict):
    db = get_db()
    try:
        oid = ObjectId(id)
    except:
        return None

    if "robot_id" in data:
        r = data["robot_id"].strip()
        data["topic"] = f"robots/mp400/{r}/status"

    data["updated_at"] = datetime.utcnow()
    db.robots.update_one({"_id": oid, "deleted": False}, {"$set": data})
    return get_robot(id)


def soft_delete_robot(id: str):
    try:
        oid = ObjectId(id)
    except:
        return False

    res = get_db().robots.update_one({"_id": oid}, {"$set": {"deleted": True}})
    return res.modified_count > 0


def update_robot_telemetry(robot_name: str, t: dict):
    db = get_db()
    now = datetime.utcnow()
    t["updated_at"] = now
    t["last_seen"] = now

    db.robots.update_one(
        {"robot_id": robot_name, "deleted": False},
        {"$set": t},
        upsert=True,
    )

    write_robot_telemetry_influx(robot_name, t)


def update_robot_pose(robot_name: str, pose_data: dict):
    """Update robot pose including odometry with yaw (x, y, yaw from odom)."""
    db = get_db()
    now = datetime.utcnow()
    
    update_doc = {
        "x": float(pose_data.get("x", 0.0)),
        "y": float(pose_data.get("y", 0.0)),
        "yaw": float(pose_data.get("yaw", 0.0)),  # Odometry yaw
        "updated_at": now,
        "last_seen": now,
    }
    
    db.robots.update_one(
        {"robot_id": robot_name, "deleted": False},
        {"$set": update_doc},
        upsert=True,
    )
    
    write_robot_pose_influx(robot_name, update_doc)


def write_robot_telemetry_influx(robot_name: str, t: dict):
    influx_client, write_api = get_influx()

    point = (
        Point("robot_telemetry")
        .tag("robot", robot_name)
        .field("cpu_usage", float(t["cpu_usage"]))
        .field("ram_usage", float(t["ram_usage"]))
        .field("battery_level", float(t["battery_level"]))
        .field("temperature", float(t["temperature"]))
        .field("x", float(t["x"]))
        .field("y", float(t["y"]))
        .field("status_code", int(status_to_code(t["status"])))
        .time(datetime.utcnow())
    )

    write_api.write(
        bucket=current_app.config["INFLUX_BUCKET"],
        org=current_app.config["INFLUX_ORG"],
        record=point,
    )


def write_robot_pose_influx(robot_name: str, pose_data: dict):
    """Write robot pose (x, y, yaw) to InfluxDB time-series."""
    influx_client, write_api = get_influx()

    point = (
        Point("robot_pose")
        .tag("robot", robot_name)
        .field("x", float(pose_data["x"]))
        .field("y", float(pose_data["y"]))
        .field("yaw", float(pose_data["yaw"]))
        .time(datetime.utcnow())
    )

    write_api.write(
        bucket=current_app.config["INFLUX_BUCKET"],
        org=current_app.config["INFLUX_ORG"],
        record=point,
    )


def status_to_code(status: str) -> int:
    if status is None:
        return -1
    s = status.upper()
    return {"IDLE": 0, "BUSY": 1, "CHARGING": 2, "OFFLINE": 3}.get(s, -1)


def update_shelf_location(shelf_id: str, x: float, y: float, uncertainty: float = 0.0, observation_count: int = 1):
    """Update shelf location in MongoDB with uncertainty tracking."""
    db = get_db()
    now = datetime.utcnow()
    
    update_doc = {
        "x": x,
        "y": y,
        "uncertainty": uncertainty,
        "observation_count": observation_count,
        "last_corrected": now,
    }
    
    result = db.shelf_locations.update_one(
        {"shelf_id": shelf_id},
        {"$set": update_doc},
        upsert=True,
    )
    
    return result.modified_count > 0 or result.upserted_id is not None


def apply_shelf_correction(shelf_id: str, old_x: float, old_y: float, 
                          new_x: float, new_y: float, drift_distance: float, 
                          action_type: str = "AUTO_CORRECTED"):
    """Apply shelf location correction and log the change."""
    db = get_db()
    now = datetime.utcnow()
    
    # Update shelf location
    shelf_update = {
        "x": new_x,
        "y": new_y,
        "last_corrected": now,
        "correction_history": []
    }
    
    shelf_doc = db.shelf_locations.find_one({"shelf_id": shelf_id})
    
    # Create correction record
    correction_record = {
        "timestamp": now,
        "old_x": old_x,
        "old_y": old_y,
        "new_x": new_x,
        "new_y": new_y,
        "drift_distance": drift_distance,
        "action_type": action_type,
    }
    
    # Apply correction
    result = db.shelf_locations.update_one(
        {"shelf_id": shelf_id},
        {
            "$set": shelf_update,
            "$push": {"correction_history": correction_record}
        },
        upsert=True,
    )
    
    # Write to InfluxDB for time-series tracking
    write_shelf_correction_influx(shelf_id, old_x, old_y, new_x, new_y, drift_distance, action_type)
    
    return result.modified_count > 0 or result.upserted_id is not None


def suggest_shelf_review(shelf_id: str, current_x: float, current_y: float, 
                        uncertainty: float, observation_count: int):
    """Create a shelf review suggestion for manual verification."""
    db = get_db()
    now = datetime.utcnow()
    
    suggestion = {
        "shelf_id": shelf_id,
        "current_x": current_x,
        "current_y": current_y,
        "uncertainty": uncertainty,
        "observation_count": observation_count,
        "created_at": now,
        "status": "PENDING",
        "reviewed_by": None,
        "review_notes": None,
    }
    
    result = db.shelf_review_suggestions.insert_one(suggestion)
    return str(result.inserted_id)


def write_shelf_correction_influx(shelf_id: str, old_x: float, old_y: float, 
                                 new_x: float, new_y: float, drift_distance: float, 
                                 action_type: str):
    """Write shelf correction to InfluxDB time-series."""
    influx_client, write_api = get_influx()
    
    point = (
        Point("shelf_correction")
        .tag("shelf_id", shelf_id)
        .tag("action_type", action_type)
        .field("old_x", old_x)
        .field("old_y", old_y)
        .field("new_x", new_x)
        .field("new_y", new_y)
        .field("drift_distance", drift_distance)
        .time(datetime.utcnow())
    )
    
    write_api.write(
        bucket=current_app.config["INFLUX_BUCKET"],
        org=current_app.config["INFLUX_ORG"],
        record=point,
    )


def get_shelf_location(shelf_id: str):
    """Retrieve shelf location with all metadata."""
    db = get_db()
    shelf = db.shelf_locations.find_one({"shelf_id": shelf_id})
    return serialize(shelf) if shelf else None


def get_shelf_correction_history(shelf_id: str, limit: int = 10):
    """Get correction history for a specific shelf."""
    db = get_db()
    shelf = db.shelf_locations.find_one({"shelf_id": shelf_id})
    
    if not shelf or "correction_history" not in shelf:
        return []
    
    # Return most recent corrections (sorted by timestamp desc)
    return sorted(
        shelf["correction_history"],
        key=lambda x: x.get("timestamp", datetime.utcnow()),
        reverse=True
    )[:limit]


def list_shelf_review_suggestions(status: str = "PENDING"):
    """List pending shelf review suggestions."""
    db = get_db()
    suggestions = list(db.shelf_review_suggestions.find({"status": status}))
    return [serialize(s) for s in suggestions]


def resolve_shelf_review(suggestion_id: str, approved: bool, notes: str = ""):
    """Resolve a shelf review suggestion."""
    db = get_db()
    
    try:
        oid = ObjectId(suggestion_id)
    except:
        return False
    
    update_doc = {
        "status": "APPROVED" if approved else "REJECTED",
        "reviewed_by": "admin",  # TODO: get from user context
        "review_notes": notes,
        "reviewed_at": datetime.utcnow(),
    }
    
    result = db.shelf_review_suggestions.update_one(
        {"_id": oid},
        {"$set": update_doc}
    )
    
    return result.modified_count > 0
