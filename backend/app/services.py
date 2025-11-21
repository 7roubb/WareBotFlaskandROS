from datetime import datetime
from typing import Optional, List, Dict, Any
from bson import ObjectId
from werkzeug.security import generate_password_hash, check_password_hash
import uuid
import io
import os
from influxdb_client import Point
from flask import current_app

from .extensions import get_db, get_influx, get_minio

from .models import ROBOT_STATUSES


# =========================================================
# UTILITIES
# =========================================================
def serialize(doc: Dict[str, Any]) -> Dict[str, Any]:
    """Convert MongoDB document -> serializable JSON"""
    if not doc:
        return None
    doc["id"] = str(doc["_id"])
    doc.pop("_id", None)
    return doc


# =========================================================
# PRODUCT SERVICES
# =========================================================
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
    return serialize(db.products.find_one({"_id": res.inserted_id}))


def list_products() -> List[dict]:
    db = get_db()
    return [serialize(p) for p in db.products.find({"deleted": False})]


def get_product(id: str) -> Optional[dict]:
    db = get_db()
    try:
        oid = ObjectId(id)
    except:
        return None
    return serialize(db.products.find_one({"_id": oid, "deleted": False}))


def update_product(id: str, data: dict) -> Optional[dict]:
    db = get_db()
    try:
        oid = ObjectId(id)
    except:
        return None

    data["updated_at"] = datetime.utcnow()
    db.products.update_one({"_id": oid, "deleted": False}, {"$set": data})
    return get_product(id)


def soft_delete_product(id: str) -> bool:
    db = get_db()
    try:
        oid = ObjectId(id)
    except:
        return False

    res = db.products.update_one({"_id": oid}, {"$set": {"deleted": True}})
    return res.modified_count > 0


def search_products_by_name(q: str) -> List[dict]:
    db = get_db()
    docs = db.products.find(
        {"deleted": False, "name": {"$regex": q, "$options": "i"}}
    )
    return [serialize(p) for p in docs]


def get_products_for_shelf(shelf_id: str) -> List[dict]:
    db = get_db()
    docs = db.products.find({"deleted": False, "shelf_id": shelf_id})
    return [serialize(p) for p in docs]


# =========================================================
# STOCK MANAGEMENT
# =========================================================
def record_transaction(data: dict):
    db = get_db()
    db.product_transactions.insert_one(data)


def subtract_from_product_stock(product_id: str, qty: int, desc=None):
    db = get_db()
    product = get_product(product_id)
    if not product:
        raise ValueError("product_not_found")

    if qty > product["quantity"]:
        raise ValueError("not_enough_stock")

    before = product["quantity"]
    after = before - qty

    db.products.update_one(
        {"_id": ObjectId(product_id)},
        {"$set": {"quantity": after, "updated_at": datetime.utcnow()}}
    )

    record_transaction({
        "product_id": product_id,
        "action": "PICK",
        "quantity": qty,
        "timestamp": datetime.utcnow(),
        "old_quantity": before,
        "new_quantity": after,
        "description": desc,
        "shelf_id": product.get("shelf_id"),
    })

    return after


def return_product_stock(product_id: str, qty: int, desc=None):
    db = get_db()
    product = get_product(product_id)
    if not product:
        raise ValueError("product_not_found")

    before = product["quantity"]
    after = before + qty

    db.products.update_one(
        {"_id": ObjectId(product_id)},
        {"$set": {"quantity": after, "updated_at": datetime.utcnow()}}
    )

    record_transaction({
        "product_id": product_id,
        "action": "RETURN",
        "quantity": qty,
        "timestamp": datetime.utcnow(),
        "old_quantity": before,
        "new_quantity": after,
        "description": desc,
        "shelf_id": product.get("shelf_id"),
    })

    return get_product(product_id)


def adjust_product_stock(product_id: str, new_qty: int, reason=None):
    db = get_db()
    product = get_product(product_id)
    if not product:
        raise ValueError("product_not_found")

    before = product["quantity"]
    db.products.update_one(
        {"_id": ObjectId(product_id)},
        {"$set": {"quantity": new_qty, "updated_at": datetime.utcnow()}}
    )

    record_transaction({
        "product_id": product_id,
        "action": "ADJUST",
        "quantity": abs(before - new_qty),
        "timestamp": datetime.utcnow(),
        "old_quantity": before,
        "new_quantity": new_qty,
        "description": reason,
    })

    return get_product(product_id)


# =========================================================
# SHELVES
# =========================================================
def create_shelf(data: dict):
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
    return serialize(db.shelves.find_one({"_id": res.inserted_id}))


def get_shelf(id: str):
    db = get_db()
    try:
        oid = ObjectId(id)
    except:
        return None
    return serialize(db.shelves.find_one({"_id": oid, "deleted": False}))


def list_shelves():
    db = get_db()
    return [serialize(s) for s in db.shelves.find({"deleted": False})]


def update_shelf(id: str, data: dict):
    db = get_db()
    try:
        oid = ObjectId(id)
    except:
        return None

    data["updated_at"] = datetime.utcnow()
    db.shelves.update_one({"_id": oid, "deleted": False}, {"$set": data})
    return get_shelf(id)


def soft_delete_shelf(id: str):
    db = get_db()
    try:
        oid = ObjectId(id)
    except:
        return False

    res = db.shelves.update_one({"_id": oid}, {"$set": {"deleted": True}})
    return res.modified_count > 0


# =========================================================
# ROBOT SERVICES + TELEMETRY (Mongo + InfluxDB)
# =========================================================
def create_robot(data: dict):
    """
    Create a robot by giving only:
    - name
    - robot_id  (example: robot1)

    The system will generate the MQTT topic automatically:
        robots/mp400/<robot_id>/status
    """
    db = get_db()
    now = datetime.utcnow()

    robot_id = data["robot_id"].strip()
    topic = f"robots/mp400/{robot_id}/status"       # <-- auto topic

    doc = {
        "name": data["name"],
        "robot_id": robot_id,
        "topic": topic,

        "available": data.get("available", True),
        "status": data.get("status", "IDLE"),
        "current_shelf_id": data.get("current_shelf_id"),

        # Telemetry fields
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
    db = get_db()
    return [serialize(r) for r in db.robots.find({"deleted": False})]


def get_robot(id: str):
    db = get_db()
    try:
        oid = ObjectId(id)
    except:
        return None
    return serialize(db.robots.find_one({"_id": oid, "deleted": False}))


def update_robot(id: str, data: dict):
    db = get_db()
    try:
        oid = ObjectId(id)
    except:
        return None

    # If robot_id is changed, regenerate topic
    if "robot_id" in data:
        robot_id = data["robot_id"].strip()
        data["topic"] = f"robots/mp400/{robot_id}/status"

    data["updated_at"] = datetime.utcnow()
    db.robots.update_one({"_id": oid, "deleted": False}, {"$set": data})
    return get_robot(id)


def soft_delete_robot(id: str):
    db = get_db()
    try:
        oid = ObjectId(id)
    except:
        return False

    res = db.robots.update_one({"_id": oid}, {"$set": {"deleted": True}})
    return res.modified_count > 0


def update_robot_telemetry(robot_name: str, t: dict):
    db = get_db()
    t["updated_at"] = datetime.utcnow()
    t["last_seen"] = datetime.utcnow()   # NEW LINE

    db.robots.update_one(
        {"robot_id": robot_name, "deleted": False},
        {"$set": t},
        upsert=False
    )

def write_robot_telemetry_influx(robot_name: str, t: dict):
    """Historical → InfluxDB"""
    influx, write_api = get_influx()

    point = (
        Point("robot_telemetry")
        .tag("robot", robot_name)
        .field("cpu_usage", t["cpu_usage"])
        .field("ram_usage", t["ram_usage"])
        .field("battery_level", t["battery_level"])
        .field("temperature", t["temperature"])
        .field("x", t["x"])
        .field("y", t["y"])
        .field("status_code", status_to_code(t["status"]))
        .time(datetime.utcnow())
    )

    write_api.write(
        bucket=current_app.config["INFLUX_BUCKET"],
        record=point
    )


def status_to_code(status: str) -> int:
    status = status.upper()
    if status == "IDLE": return 0
    if status == "BUSY": return 1
    if status == "ERROR": return 2
    if status == "OFFLINE": return 3
    return -1


# =========================================================
# TASK SYSTEM
# =========================================================
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

    # Find best robot (distance + battery)
    best = None
    best_cost = 999999

    for r in robots:
        rx, ry = r.get("x"), r.get("y")
        battery = r.get("battery_level", 100)

        if rx is None or ry is None:
            continue

        dist = ((rx - sx) ** 2 + (ry - sy) ** 2) ** 0.5
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
    db = get_db()
    return [serialize(t) for t in db.tasks.find({})]


def update_task_status(task_id: str, status: str):
    db = get_db()
    try:
        oid = ObjectId(task_id)
    except:
        return False

    db.tasks.update_one(
        {"_id": oid},
        {"$set": {"status": status, "updated_at": datetime.utcnow()}}
    )
    return True


# =========================================================
# DASHBOARD SERVICES
# =========================================================
def dashboard_top_moving_products():
    db = get_db()
    pipeline = [
        {"$match": {"action": "PICK"}},
        {"$group": {"_id": "$product_id", "total": {"$sum": "$quantity"}}},
        {"$sort": {"total": -1}},
        {"$limit": 10},
    ]
    return list(db.product_transactions.aggregate(pipeline))


def dashboard_shelf_summary():
    db = get_db()
    shelves = list(db.shelves.find({"deleted": False}))
    results = []

    for s in shelves:
        shelf_id = str(s["_id"])     # 🔥 1) تحويل _id → string

        # 🔥 2) المنتجات في DB تخزن shelf_id كـ string
        products = list(db.products.find({
            "shelf_id": shelf_id,
            "deleted": False
        }))

        total_items = sum(p.get("quantity", 0) for p in products)

        results.append({
            "id": shelf_id,                            # 🔥 3) صحيح
            "name": s.get("name", ""),                 # إضافة اسم الرف (مهم لواجهة React)
            "coords": [
                s.get("x_coord", 0),
                s.get("y_coord", 0)
            ],
            "level": s.get("level", 1),
            "products": len(products),
            "total_items": total_items
        })

    return results


def dashboard_daily_movements():
    db = get_db()
    today = datetime.utcnow().date()
    start = datetime(today.year, today.month, today.day)

    pipeline = [
        {"$match": {"timestamp": {"$gte": start}}},
        {"$group": {"_id": "$action", "qty": {"$sum": "$quantity"}}},
    ]
    return list(db.product_transactions.aggregate(pipeline))


# =========================================================
# ADMIN / AUTH
# =========================================================
def get_admin_by_username(username: str):
    db = get_db()
    return serialize(db.admins.find_one({"username": username}))


def create_admin(username: str, password: str):
    db = get_db()
    now = datetime.utcnow()

    doc = {
        "username": username,
        "password_hash": generate_password_hash(password),
        "role": "ADMIN",
        "created_at": now,
    }

    res = db.admins.insert_one(doc)
    return serialize(db.admins.find_one({"_id": res.inserted_id}))


def verify_admin_password(username: str, password: str) -> bool:
    db = get_db()
    doc = db.admins.find_one({"username": username})
    if not doc:
        return False
    return check_password_hash(doc["password_hash"], password)


# =========================================================
# MINIO IMAGE SERVICES
# =========================================================
def upload_image_to_minio(file_content: bytes, filename: str, product_id: str, content_type: str):
    client = get_minio()
    if not client:
        raise Exception("MinIO not initialized")

    bucket = current_app.config["MINIO_BUCKET"]

    # -------------------------------------------
    # Clean filename (remove spaces, strange chars)
    # -------------------------------------------
    filename = filename.strip().replace(" ", "_")

    # Extract name + extension safely
    name, ext = os.path.splitext(filename)

    # If no extension → default to .jpg
    if not ext:
        ext = ".jpg"

    # -------------------------------------------
    # Build correct object path
    # No double "products/products"
    # -------------------------------------------
    object_name = f"{product_id}/{name}{ext}"

    # -------------------------------------------
    # Upload to MinIO
    # -------------------------------------------
    client.put_object(
        bucket_name=bucket,
        object_name=object_name,
        data=io.BytesIO(file_content),
        length=len(file_content),
        content_type=content_type
    )

    # -------------------------------------------
    # Public URL for frontend
    # Use PUBLIC_MINIO instead of MINIO_ENDPOINT
    # -------------------------------------------
    public_domain = current_app.config.get("PUBLIC_MINIO", "localhost:9000")

    return f"http://{public_domain}/{bucket}/{object_name}"

def delete_image_from_minio(url: str) -> bool:
    if not get_minio():
        return False

    bucket = current_app.config["MINIO_BUCKET"]
    prefix = f"http://{current_app.config['MINIO_ENDPOINT']}/{bucket}/"

    if not url.startswith(prefix):
        return False

    object_name = url.replace(prefix, "")
    get_minio().remove_object(bucket, object_name)
    return True


def update_product_images(product_id: str, main_image_url: str = None, image_urls: List[str] = None):
    db = get_db()
    try:
        oid = ObjectId(product_id)
    except:
        return None

    update_data = {}
    if main_image_url is not None:
        update_data["main_image_url"] = main_image_url
    if image_urls is not None:
        update_data["image_urls"] = image_urls

    update_data["updated_at"] = datetime.utcnow()

    db.products.update_one({"_id": oid, "deleted": False}, {"$set": update_data})
    return get_product(product_id)
