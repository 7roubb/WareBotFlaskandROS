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


def status_to_code(status: str) -> int:
    if status is None:
        return -1
    s = status.upper()
    return {"IDLE": 0, "BUSY": 1, "CHARGING": 2, "OFFLINE": 3}.get(s, -1)
