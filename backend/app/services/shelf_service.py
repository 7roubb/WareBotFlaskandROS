from datetime import datetime
from bson import ObjectId

from ..extensions import get_db
from .utils_service import serialize
from .apriltag_service import (
    generate_apriltag,
    upload_apriltag,
    MAX_TAG_IDS,
)

# =========================================================
# CREATE SHELF
# =========================================================

def create_shelf(data: dict):
    db = get_db()
    now = datetime.utcnow()

    # -----------------------------
    # CURRENT (LIVE) LOCATION
    # -----------------------------
    current_x = float(data["current_x"])
    current_y = float(data["current_y"])
    current_yaw = float(data.get("current_yaw", 0.0))

    # -----------------------------
    # STORAGE (HOME) LOCATION
    # If not provided → copy from current
    # -----------------------------
    storage_x = float(data.get("storage_x", current_x))
    storage_y = float(data.get("storage_y", current_y))
    storage_yaw = float(data.get("storage_yaw", current_yaw))

    doc = {
        "warehouse_id": data["warehouse_id"],

        # Current location (mutable)
        "current_x": current_x,
        "current_y": current_y,
        "current_yaw": current_yaw,

        # Storage location (immutable)
        "storage_x": storage_x,
        "storage_y": storage_y,
        "storage_yaw": storage_yaw,

        "level": data["level"],
        "available": data.get("available", True),
        "status": data.get("status", "IDLE"),

        "april_tag_url": None,
        "april_tag_id": None,

        "created_at": now,
        "updated_at": now,
        "deleted": False,
    }

    res = db.shelves.insert_one(doc)
    shelf_oid = res.inserted_id
    shelf_id = str(shelf_oid)

    # -----------------------------
    # Generate AprilTag
    # -----------------------------
    raw_int = int(shelf_oid.binary.hex(), 16)
    tag_id = raw_int % MAX_TAG_IDS

    tag_png = generate_apriltag(tag_id)
    tag_url = upload_apriltag(tag_png, shelf_id)

    db.shelves.update_one(
        {"_id": shelf_oid},
        {"$set": {"april_tag_url": tag_url, "april_tag_id": tag_id}},
    )

    return serialize(db.shelves.find_one({"_id": shelf_oid}))


# =========================================================
# GET SHELF
# =========================================================

def get_shelf(shelf_id: str):
    db = get_db()
    try:
        oid = ObjectId(shelf_id)
    except Exception:
        return None

    return serialize(db.shelves.find_one({"_id": oid, "deleted": False}))


# =========================================================
# LIST SHELVES
# =========================================================

def list_shelves():
    db = get_db()
    return [serialize(s) for s in db.shelves.find({"deleted": False})]


# =========================================================
# UPDATE SHELF
# =========================================================

def update_shelf(shelf_id: str, data: dict):
    db = get_db()
    try:
        oid = ObjectId(shelf_id)
    except Exception:
        return None

    # -----------------------------
    # Protect storage location
    # -----------------------------
    manual_reset = bool(data.pop("manual_reset", False))
    storage_fields = {"storage_x", "storage_y", "storage_yaw"}

    if not manual_reset and any(k in data for k in storage_fields):
        return {
            "error": "forbidden_to_modify_storage",
            "message": "Storage location is immutable. Use admin reset endpoint."
        }

    # -----------------------------
    # Normalize numeric fields
    # -----------------------------
    numeric_fields = {
        "current_x",
        "current_y",
        "current_yaw",
        "storage_x",
        "storage_y",
        "storage_yaw",
    }

    for key in numeric_fields:
        if key in data and data[key] is not None:
            data[key] = float(data[key])

    data["updated_at"] = datetime.utcnow()

    db.shelves.update_one(
        {"_id": oid, "deleted": False},
        {"$set": data},
    )

    return get_shelf(shelf_id)


# =========================================================
# ADMIN — SET STORAGE LOCATION
# =========================================================

def set_shelf_storage_location(
    shelf_id: str,
    x: float,
    y: float,
    yaw: float = 0.0,
) -> bool:
    """
    Admin-only endpoint.
    Explicitly resets the storage (home) location.
    """
    db = get_db()
    try:
        oid = ObjectId(shelf_id)
    except Exception:
        return False

    res = db.shelves.update_one(
        {"_id": oid, "deleted": False},
        {
            "$set": {
                "storage_x": float(x),
                "storage_y": float(y),
                "storage_yaw": float(yaw),
                "updated_at": datetime.utcnow(),
            }
        },
    )

    return res.modified_count > 0


# =========================================================
# SOFT DELETE
# =========================================================

def soft_delete_shelf(shelf_id: str) -> bool:
    db = get_db()
    try:
        oid = ObjectId(shelf_id)
    except Exception:
        return False

    res = db.shelves.update_one(
        {"_id": oid},
        {"$set": {"deleted": True, "updated_at": datetime.utcnow()}},
    )

    return res.modified_count > 0
