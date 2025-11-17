from datetime import datetime
from typing import Optional, List, Dict, Any
from bson import ObjectId
from werkzeug.security import generate_password_hash, check_password_hash
from math import sqrt

from .extensions import get_db


def serialize_doc(doc: Dict[str, Any]) -> Dict[str, Any]:
    if not doc:
        return doc
    doc["id"] = str(doc["_id"])
    doc.pop("_id", None)
    return doc


# =========================
# Product Services
# =========================
def create_product(data: dict) -> dict:
    db = get_db()
    now = datetime.utcnow()
    doc = {
        "name": data["name"],
        "sku": data["sku"],
        "quantity": data.get("quantity", 0),
        "category": data.get("category"),
        "brand": data.get("brand"),
        "price": data.get("price"),
        "weight_kg": data.get("weight_kg"),
        "dimensions_cm": data.get("dimensions_cm"),
        "barcode": data.get("barcode"),
        "main_image_url": data.get("main_image_url"),
        "image_urls": data.get("image_urls", []),
        "shelf_id": data.get("shelf_id"),
        "description": data.get("description"),
        "created_at": now,
        "updated_at": now,
        "deleted": False,
    }
    res = db.products.insert_one(doc)
    return serialize_doc(db.products.find_one({"_id": res.inserted_id}))


def list_products() -> List[dict]:
    db = get_db()
    docs = db.products.find({"deleted": False})
    return [serialize_doc(d) for d in docs]


def get_product(product_id: str) -> Optional[dict]:
    db = get_db()
    try:
        oid = ObjectId(product_id)
    except Exception:
        return None
    doc = db.products.find_one({"_id": oid, "deleted": False})
    return serialize_doc(doc) if doc else None


def update_product(product_id: str, data: dict) -> Optional[dict]:
    db = get_db()
    try:
        oid = ObjectId(product_id)
    except Exception:
        return None
    data["updated_at"] = datetime.utcnow()
    db.products.update_one({"_id": oid, "deleted": False}, {"$set": data})
    return get_product(product_id)


def soft_delete_product(product_id: str) -> bool:
    db = get_db()
    try:
        oid = ObjectId(product_id)
    except Exception:
        return False
    res = db.products.update_one({"_id": oid}, {"$set": {"deleted": True}})
    return res.modified_count > 0


def search_products_by_name(query: str) -> List[dict]:
    db = get_db()
    docs = db.products.find(
        {"deleted": False, "name": {"$regex": query, "$options": "i"}}
    )
    return [serialize_doc(d) for d in docs]


def get_shelf_for_product(product_id: str) -> Optional[dict]:
    db = get_db()
    product = get_product(product_id)
    if not product:
        return None
    shelf_id = product.get("shelf_id")
    if not shelf_id:
        return None
    return get_shelf(shelf_id)


# =========================
# Shelf Services
# =========================
def create_shelf(data: dict) -> dict:
    db = get_db()
    now = datetime.utcnow()
    doc = {
        "warehouse_id": data["warehouse_id"],
        "x_coord": data["x_coord"],
        "y_coord": data["y_coord"],
        "level": data["level"],
        "available": data.get("available", True),
        "status": data.get("status", "IDLE"),
        "created_at": now,
        "updated_at": now,
        "deleted": False,
    }
    res = db.shelves.insert_one(doc)
    return serialize_doc(db.shelves.find_one({"_id": res.inserted_id}))


def list_shelves() -> List[dict]:
    db = get_db()
    docs = db.shelves.find({"deleted": False})
    return [serialize_doc(d) for d in docs]


def get_shelf(shelf_id: str) -> Optional[dict]:
    db = get_db()
    try:
        oid = ObjectId(shelf_id)
    except Exception:
        return None
    doc = db.shelves.find_one({"_id": oid, "deleted": False})
    return serialize_doc(doc) if doc else None


def update_shelf(shelf_id: str, data: dict) -> Optional[dict]:
    db = get_db()
    try:
        oid = ObjectId(shelf_id)
    except Exception:
        return None
    data["updated_at"] = datetime.utcnow()
    db.shelves.update_one({"_id": oid, "deleted": False}, {"$set": data})
    return get_shelf(shelf_id)


def soft_delete_shelf(shelf_id: str) -> bool:
    db = get_db()
    try:
        oid = ObjectId(shelf_id)
    except Exception:
        return False
    res = db.shelves.update_one({"_id": oid}, {"$set": {"deleted": True}})
    return res.modified_count > 0


def get_products_for_shelf(shelf_id: str) -> List[dict]:
    db = get_db()
    docs = db.products.find({"deleted": False, "shelf_id": shelf_id})
    return [serialize_doc(d) for d in docs]


# =========================
# Robot Services
# =========================
def create_robot(data: dict) -> dict:
    db = get_db()
    now = datetime.utcnow()
    doc = {
        "name": data["name"],
        "available": data.get("available", True),
        "status": data.get("status", "IDLE"),
        "current_shelf_id": data.get("current_shelf_id"),
        "created_at": now,
        "updated_at": now,
        "deleted": False,
        "cpu_usage": None,
        "ram_usage": None,
        "battery_level": None,
        "temperature": None,
        "x": None,
        "y": None,
    }
    res = db.robots.insert_one(doc)
    return serialize_doc(db.robots.find_one({"_id": res.inserted_id}))


def list_robots() -> List[dict]:
    db = get_db()
    docs = db.robots.find({"deleted": False})
    return [serialize_doc(d) for d in docs]


def get_robot(robot_id: str) -> Optional[dict]:
    db = get_db()
    try:
        oid = ObjectId(robot_id)
    except Exception:
        return None
    doc = db.robots.find_one({"_id": oid, "deleted": False})
    return serialize_doc(doc) if doc else None


def update_robot(robot_id: str, data: dict) -> Optional[dict]:
    db = get_db()
    try:
        oid = ObjectId(robot_id)
    except Exception:
        return None
    data["updated_at"] = datetime.utcnow()
    db.robots.update_one({"_id": oid, "deleted": False}, {"$set": data})
    return get_robot(robot_id)


def soft_delete_robot(robot_id: str) -> bool:
    db = get_db()
    try:
        oid = ObjectId(robot_id)
    except Exception:
        return False
    res = db.robots.update_one({"_id": oid}, {"$set": {"deleted": True}})
    return res.modified_count > 0


def update_robot_telemetry(robot_name: str, telemetry: dict):
    """
    Called from MQTT client: identify robot by 'name'.
    """
    db = get_db()
    telemetry["updated_at"] = datetime.utcnow()
    db.robots.update_one(
        {"name": robot_name, "deleted": False},
        {"$set": telemetry},
        upsert=False,
    )


# =========================
# Admin / Auth Services
# =========================
def get_admin_by_username(username: str) -> Optional[dict]:
    db = get_db()
    doc = db.admins.find_one({"username": username})
    if not doc:
        return None
    return serialize_doc(doc)


def create_admin(username: str, password: str) -> dict:
    db = get_db()
    now = datetime.utcnow()
    password_hash = generate_password_hash(password)
    doc = {
        "username": username,
        "password_hash": password_hash,
        "role": "ADMIN",
        "created_at": now,
    }
    res = db.admins.insert_one(doc)
    return serialize_doc(db.admins.find_one({"_id": res.inserted_id}))


def verify_admin_password(username: str, password: str) -> bool:
    db = get_db()
    doc = db.admins.find_one({"username": username})
    if not doc:
        return False
    return check_password_hash(doc["password_hash"], password)


def assign_task_to_best_robot(shelf_id):
    db = get_db()
    shelf = db.shelves.find_one({"_id": ObjectId(shelf_id)})

    if not shelf:
        raise ValueError("shelf_not_found")

    sx, sy = shelf["xCoord"], shelf["yCoord"]

    robots = list(db.robots.find({"available": True}))

    if not robots:
        raise ValueError("no_available_robots")

    best = None
    best_cost = float("inf")

    for r in robots:
        rx, ry = r.get("x", 0), r.get("y", 0)
        battery = r.get("battery", 100)

        dist = sqrt((rx - sx) ** 2 + (ry - sy) ** 2)

        cost = dist * 0.7 + (100 - battery) * 0.3

        if cost < best_cost:
            best = r
            best_cost = cost

    return best
def euclidean(x1, y1, x2, y2):
    return sqrt((x1 - x2)**2 + (y1 - y2)**2)

def create_task_and_assign(shelf_id: str, priority: int, description: str | None):
    db = get_db()

    try:
        oid = ObjectId(shelf_id)
    except Exception:
        raise ValueError("invalid_shelf_id")

    shelf = db.shelves.find_one({"_id": oid, "deleted": False})
    if not shelf:
        raise ValueError("shelf_not_found")

    sx, sy = shelf["xCoord"], shelf["yCoord"]

    robots = list(db.robots.find({"deleted": False, "available": True}))
    if not robots:
        raise ValueError("no_available_robots")

    best_robot = None
    best_cost = float("inf")

    for r in robots:
        rx, ry = r.get("x"), r.get("y")
        battery = r.get("battery", 100.0)

        if rx is None or ry is None:
            continue

        dist = euclidean(rx, ry, sx, sy)
        cost = dist * 0.7 + (100.0 - battery) * 0.3

        if cost < best_cost:
            best_cost = cost
            best_robot = r

    if best_robot is None:
        raise ValueError("no_suitable_robot")

    now = datetime.utcnow()
    task_doc = {
        "shelf_id": str(oid),
        "priority": priority,
        "description": description,
        "assigned_robot_name": best_robot["name"],
        "assigned_robot_id": best_robot["_id"],
        "status": "PENDING",  # PENDING → RUNNING → DONE
        "created_at": now,
        "updated_at": now,
    }

    res = db.tasks.insert_one(task_doc)
    return serialize_doc(db.tasks.find_one({"_id": res.inserted_id}))


def update_task_status(task_id: str, status: str):
    db = get_db()
    try:
        oid = ObjectId(task_id)
    except Exception:
        return False
    res = db.tasks.update_one(
        {"_id": oid},
        {"$set": {"status": status, "updated_at": datetime.utcnow()}}
    )
    return res.modified_count > 0

def list_tasks() -> List[dict]:
    db = get_db()
    docs = db.tasks.find({})
    return [serialize_doc(d) for d in docs]