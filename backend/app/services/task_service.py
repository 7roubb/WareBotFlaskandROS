from datetime import datetime
from bson import ObjectId
from .utils_service import serialize
from ..extensions import get_db


def create_task_and_assign(shelf_id: str, priority: int, desc=None):
    db = get_db()
    try:
        oid = ObjectId(shelf_id)
    except:
        raise ValueError("invalid_shelf_id")

    shelf = db.shelves.find_one({"_id": oid, "deleted": False})
    if not shelf:
        raise ValueError("shelf_not_found")

    robots = list(db.robots.find({"deleted": False, "available": True}))
    if not robots:
        raise ValueError("no_available_robots")

    sx, sy = shelf["x_coord"], shelf["y_coord"]

    best = None
    best_cost = 1e12

    for r in robots:
        rx, ry = r.get("x"), r.get("y")
        battery = r.get("battery_level", 100)

        if rx is None or ry is None:
            continue

        dist = ((rx - sx)**2 + (ry - sy)**2) ** 0.5
        cost = dist * 0.7 + (100 - battery) * 0.3

        if cost < best_cost:
            best_cost = cost
            best = r

    if not best:
        raise ValueError("no_suitable_robot")

    now = datetime.utcnow()

    doc = {
        "shelf_id": shelf_id,
        "priority": priority,
        "description": desc,
        "assigned_robot_name": best["name"],
        "assigned_robot_id": best["_id"],
        "status": "PENDING",
        "created_at": now,
        "updated_at": now,
    }

    res = db.tasks.insert_one(doc)
    return serialize(db.tasks.find_one({"_id": res.inserted_id}))


def list_tasks():
    return [serialize(t) for t in get_db().tasks.find({})]


def update_task_status(task_id: str, status: str):
    try:
        oid = ObjectId(task_id)
    except:
        return False

    get_db().tasks.update_one(
        {"_id": oid},
        {"$set": {"status": status, "updated_at": datetime.utcnow()}},
    )
    return True
