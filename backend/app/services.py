from datetime import datetime
from typing import Optional, List, Dict, Any
from bson import ObjectId
from werkzeug.security import generate_password_hash, check_password_hash
import io
import os

# AprilTag generator (moms_apriltag)
from moms_apriltag import TagGenerator2

import cv2
import numpy as np

from influxdb_client import Point
from flask import current_app

from minio import Minio

from .extensions import get_db, get_influx, get_minio
from .models import ROBOT_STATUSES


# =========================================================
# UTILITIES
# =========================================================
def serialize(doc: Dict[str, Any]) -> Dict[str, Any]:
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
    return [serialize(p) for p in get_db().products.find({"deleted": False})]


def get_product(id: str) -> Optional[dict]:
    db = get_db()
    try:
        oid = ObjectId(id)
    except Exception:
        return None
    return serialize(db.products.find_one({"_id": oid, "deleted": False}))


def update_product(id: str, data: dict) -> Optional[dict]:
    db = get_db()
    try:
        oid = ObjectId(id)
    except Exception:
        return None

    data["updated_at"] = datetime.utcnow()
    db.products.update_one({"_id": oid, "deleted": False}, {"$set": data})
    return get_product(id)


def soft_delete_product(id: str) -> bool:
    db = get_db()
    try:
        oid = ObjectId(id)
    except Exception:
        return False

    res = db.products.update_one({"_id": oid}, {"$set": {"deleted": True}})
    return res.modified_count > 0


def search_products_by_name(q: str) -> List[dict]:
    docs = get_db().products.find(
        {"deleted": False, "name": {"$regex": q, "$options": "i"}}
    )
    return [serialize(p) for p in docs]


def get_products_for_shelf(shelf_id: str) -> List[dict]:
    docs = get_db().products.find({"deleted": False, "shelf_id": shelf_id})
    return [serialize(p) for p in docs]


# =========================================================
# STOCK MANAGEMENT
# =========================================================
def record_transaction(data: dict):
    get_db().product_transactions.insert_one(data)


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
        {"$set": {"quantity": after, "updated_at": datetime.utcnow()}},
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
        {"$set": {"quantity": after, "updated_at": datetime.utcnow()}},
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
        {"$set": {"quantity": new_qty, "updated_at": datetime.utcnow()}},
    )

    record_transaction({
        "product_id": product_id,
        "action": "ADJUST",
        "quantity": abs(before - new_qty),
        "timestamp": datetime.utcnow(),
        "old_quantity": before,
        "new_quantity": new_qty,
        "description": reason,
        "shelf_id": product.get("shelf_id"),
    })

    return get_product(product_id)


# =========================================================
# APRILTAG GENERATION (moms_apriltag ONLY)
# =========================================================
def generate_apriltag(tag_id: int, size: int = 500) -> bytes:
    """
    Generate AprilTag (tag36h11) using moms_apriltag TagGenerator2.
    Output: PNG bytes
    """
    gen = TagGenerator2("tag36h11")  # 587 tags available
    tag = gen.generate(tag_id)

    tag_resized = cv2.resize(tag, (size, size), interpolation=cv2.INTER_NEAREST)

    ok, png = cv2.imencode(".png", tag_resized)
    if not ok:
        raise RuntimeError("Failed to encode AprilTag")

    return png.tobytes()


def upload_apriltag(tag_bytes: bytes, shelf_id: str):
    client = Minio(
        "minio:9000",
        access_key="admin",
        secret_key="admin123",
        secure=False,
    )

    bucket = "shelves"
    if not client.bucket_exists(bucket):
        client.make_bucket(bucket)

    fname = f"tag_{shelf_id}.png"

    client.put_object(
        bucket,
        fname,
        io.BytesIO(tag_bytes),
        length=len(tag_bytes),
        content_type="image/png",
    )

    return f"http://localhost:9000/{bucket}/{fname}"


# =========================================================
# SHELVES
# =========================================================
MAX_TAG_IDS = 587  # tag36h11 family size


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
        "april_tag_url": None,
        "name": data.get("name", ""),
    }

    res = db.shelves.insert_one(doc)
    shelf_id = str(res.inserted_id)

    # Unique reproducible AprilTag ID
    raw_int = int(res.inserted_id.binary.hex(), 16)
    tag_id = raw_int % MAX_TAG_IDS  # 0..586

    tag_png = generate_apriltag(tag_id)
    tag_url = upload_apriltag(tag_png, shelf_id)

    db.shelves.update_one(
        {"_id": res.inserted_id},
        {"$set": {"april_tag_url": tag_url, "april_tag_id": tag_id}},
    )

    return serialize(db.shelves.find_one({"_id": res.inserted_id}))


def get_shelf(id: str):
    db = get_db()
    try:
        oid = ObjectId(id)
    except:
        return None
    return serialize(db.shelves.find_one({"_id": oid, "deleted": False}))


def list_shelves():
    return [serialize(s) for s in get_db().shelves.find({"deleted": False})]


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
# ROBOTS
# =========================================================
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


# =========================================================
# TASK SYSTEM
# =========================================================
def create_task_and_assign(shelf_id: str, priority: int, desc=None):
    db = get_db()
    try:
        oid = ObjectId(shelf_id)
    except Exception:
        raise ValueError("invalid_shelf_id")

    shelf = db.shelves.find_one({"_id": oid, "deleted": False})
    if not shelf:
        raise ValueError("shelf_not_found")

    robots = list(
        db.robots.find({"deleted": False, "available": True})
    )
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


# =========================================================
# DASHBOARD
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
        sid = str(s["_id"])
        products = list(
            db.products.find({"shelf_id": sid, "deleted": False})
        )
        total_items = sum(p.get("quantity", 0) for p in products)

        results.append({
            "id": sid,
            "name": s.get("name", ""),
            "coords": [s.get("x_coord", 0), s.get("y_coord", 0)],
            "level": s.get("level", 1),
            "products": len(products),
            "total_items": total_items,
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
# ADMIN + AUTH
# =========================================================
def get_admin_by_username(username: str):
    return serialize(get_db().admins.find_one({"username": username}))


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
    doc = get_db().admins.find_one({"username": username})
    if not doc:
        return False
    return check_password_hash(doc["password_hash"], password)

# =========================================================
# MINIO IMAGE SERVICES  (PRODUCTS)
# =========================================================

def upload_image_to_minio(file_content: bytes, filename: str, product_id: str, content_type: str):
    """
    Upload a product image to MinIO inside bucket: products/<product_id>/
    """
    client = get_minio()
    if not client:
        raise Exception("MinIO not initialized")

    bucket = current_app.config["MINIO_BUCKET"]

    # Clean filename
    filename = filename.strip().replace(" ", "_")
    name, ext = os.path.splitext(filename)
    if not ext:
        ext = ".jpg"

    object_name = f"{product_id}/{name}{ext}"

    client.put_object(
        bucket_name=bucket,
        object_name=object_name,
        data=io.BytesIO(file_content),
        length=len(file_content),
        content_type=content_type,
    )

    # Public URL
    public_domain = current_app.config.get("PUBLIC_MINIO", "localhost:9000")
    return f"http://{public_domain}/{bucket}/{object_name}"


def delete_image_from_minio(url: str) -> bool:
    """
    Delete an image from MinIO by its public URL.
    """
    client = get_minio()
    if not client:
        return False

    bucket = current_app.config["MINIO_BUCKET"]
    prefix = f"http://{current_app.config['MINIO_ENDPOINT']}/{bucket}/"

    if not url.startswith(prefix):
        return False

    object_name = url.replace(prefix, "")
    client.remove_object(bucket, object_name)
    return True


def update_product_images(product_id: str, main_image_url: str = None, image_urls: List[str] = None):
    """
    Update product main image + gallery images
    """
    db = get_db()
    try:
        oid = ObjectId(product_id)
    except Exception:
        return None

    update_data = {}
    if main_image_url is not None:
        update_data["main_image_url"] = main_image_url
    if image_urls is not None:
        update_data["image_urls"] = image_urls

    update_data["updated_at"] = datetime.utcnow()

    db.products.update_one({"_id": oid, "deleted": False}, {"$set": update_data})
    return get_product(product_id)
