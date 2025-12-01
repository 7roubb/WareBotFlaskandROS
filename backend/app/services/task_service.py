from datetime import datetime
from typing import Optional, Dict, Any, List
from bson import ObjectId
from ..extensions import get_db
from .utils_service import serialize  # adjust import if utils_service is elsewhere


def _to_str_id(oid):
    try:
        return str(oid)
    except Exception:
        return oid


def create_task_and_assign(shelf_id: str, priority: int, desc: Optional[str] = None,
                           zone_id: Optional[str] = None) -> Dict[str, Any]:
    """
    Create a task for the shelf and pick the best available robot based on distance + battery.
    Returns the serialized task document.
    Raises ValueError on invalid input or if no resources.
    """
    db = get_db()

    # Validate shelf id (allow ObjectId string or raw string that won't convert)
    try:
        shelf_oid = ObjectId(shelf_id)
        shelf = db.shelves.find_one({"_id": shelf_oid, "deleted": False})
    except Exception:
        # try to find by shelf_id field (string)
        shelf = db.shelves.find_one({"shelf_id": shelf_id, "deleted": False})

    if not shelf:
        raise ValueError("shelf_not_found")

    # find available robots
    robots_cursor = db.robots.find({"deleted": False, "available": True})
    robots = list(robots_cursor)
    if not robots:
        raise ValueError("no_available_robots")

    # shelf coordinates
    sx = float(shelf.get("x_coord", shelf.get("x", 0.0)))
    sy = float(shelf.get("y_coord", shelf.get("y", 0.0)))
    syaw = float(shelf.get("yaw", 0.0))

    # choose best robot by simple cost function (distance * 0.7 + (100-battery)*0.3)
    best = None
    best_cost = float("inf")
    for r in robots:
        rx = r.get("x") if r.get("x") is not None else r.get("x_coord")
        ry = r.get("y") if r.get("y") is not None else r.get("y_coord")
        battery = float(r.get("battery_level", 100))
        if rx is None or ry is None:
            # skip robots with no telemetry location
            continue
        try:
            dist = ((float(rx) - sx) ** 2 + (float(ry) - sy) ** 2) ** 0.5
        except Exception:
            continue
        cost = dist * 0.7 + (100.0 - battery) * 0.3
        if cost < best_cost:
            best_cost = cost
            best = r

    if not best:
        raise ValueError("no_suitable_robot")

    now = datetime.utcnow()

    # assigned_robot_name: prefer robot_id field (e.g. "robot_1"), fallback to name
    assigned_robot_name = best.get("robot_id") or best.get("name") or ""
    assigned_robot_db_id = best.get("_id")

    # Build task document
    doc = {
        "shelf_id": shelf_id,
        "priority": int(priority),
        "description": desc,
        "assigned_robot_name": assigned_robot_name,
        # store DB _id string so external systems can find robot easily
        "assigned_robot_id": _to_str_id(assigned_robot_db_id),
        # pickup = shelf location
        "target_x": float(sx),
        "target_y": float(sy),
        "target_yaw": float(syaw),
        # include pickup fields named like the robot node expects
        "pickup_x": float(sx),
        "pickup_y": float(sy),
        "pickup_yaw": float(syaw),
        # default drop == pickup; override below if zone provided
        "drop_x": float(sx),
        "drop_y": float(sy),
        "drop_yaw": float(syaw),
        "drop_zone_id": zone_id,
        "status": "ASSIGNED",
        "created_at": now,
        "updated_at": now,
    }

    # If zone_id provided, try to resolve coordinates
    if zone_id:
        zone = None
        try:
            z_oid = ObjectId(zone_id)
            zone = db.zones.find_one({"_id": z_oid, "deleted": False})
        except Exception:
            # try by zone_id field (string)
            zone = db.zones.find_one({"zone_id": zone_id, "deleted": False})

        if zone:
            doc["drop_x"] = float(zone.get("x", doc["drop_x"]))
            doc["drop_y"] = float(zone.get("y", doc["drop_y"]))
            doc["drop_yaw"] = float(zone.get("yaw", doc["drop_yaw"]))
            # store canonical zone _id string if available
            if zone.get("_id"):
                doc["drop_zone_id"] = _to_str_id(zone.get("_id"))

    # Insert and return serialized doc
    res = db.tasks.insert_one(doc)
    created = db.tasks.find_one({"_id": res.inserted_id})
    return serialize(created)


def list_tasks() -> List[Dict[str, Any]]:
    """Return a list of serialized tasks (all)."""
    db = get_db()
    return [serialize(t) for t in db.tasks.find({})]


def update_task_status(task_id: str, status: str) -> bool:
    """
    Update a task status. Returns True if update attempted.
    If invalid task_id, returns False.
    """
    db = get_db()
    try:
        oid = ObjectId(task_id)
    except Exception:
        # maybe the client passed the stringified id already; try to find by id field
        # we'll attempt to match either _id or id-like string
        # but for update we need to return False if it's not a valid hex ObjectId
        return False

    upd = {"$set": {"status": status, "updated_at": datetime.utcnow()}}
    db.tasks.update_one({"_id": oid}, upd)
    return True
