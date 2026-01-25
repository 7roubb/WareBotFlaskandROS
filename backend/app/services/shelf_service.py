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
# CONSTANTS
# =========================================================

# Prevent shelves from being too close: radius = 0.5m
STORAGE_COLLISION_RADIUS = 0.5  # meters


# =========================================================
# HELPERS
# =========================================================

def shelf_exists_within_radius(
    db,
    warehouse_id,
    x: float,
    y: float,
    radius: float = STORAGE_COLLISION_RADIUS,
    exclude_id: ObjectId | None = None,
):
    """
    Returns a shelf if any shelf exists within a radius from (x,y)
    Excludes the shelf with exclude_id if provided (useful for updates)
    """
    query = {"warehouse_id": warehouse_id, "deleted": False}
    if exclude_id:
        query["_id"] = {"$ne": exclude_id}

    shelves = db.shelves.find(query)
    for s in shelves:
        dx = s["storage_x"] - x
        dy = s["storage_y"] - y
        if dx*dx + dy*dy <= radius*radius:
            return s
    return None


# =========================================================
# CREATE SHELF
# =========================================================

def create_shelf(data: dict):
    """
    Create a new shelf in the warehouse.
    
    IMPORTANT: Each shelf must have a unique storage location.
    Shelves cannot be placed within 0.5m (50cm) of each other.
    
    Example valid locations for multiple shelves:
    - Shelf 1: storage_x=0.0, storage_y=0.0
    - Shelf 2: storage_x=1.0, storage_y=0.0  (1m away, safe)
    - Shelf 3: storage_x=0.0, storage_y=1.0  (1m away, safe)
    - Shelf 4: storage_x=1.5, storage_y=1.5  (2.1m away, safe)
    
    To check if a location is safe:
    - Distance = sqrt((x2-x1)² + (y2-y1)²)
    - Safe if Distance > 0.5m
    """
    db = get_db()
    now = datetime.utcnow()

    # Current location
    current_x = float(data["current_x"])
    current_y = float(data["current_y"])
    current_yaw = float(data.get("current_yaw", 0.0))

    # Storage location
    storage_x = float(data.get("storage_x", current_x))
    storage_y = float(data.get("storage_y", current_y))
    storage_yaw = float(data.get("storage_yaw", current_yaw))

    # Prevent storage collision within radius
    # exclude_id=None because this is a new shelf (no ID yet)
    existing = shelf_exists_within_radius(
        db=db,
        warehouse_id=data["warehouse_id"],
        x=storage_x,
        y=storage_y,
        exclude_id=None,
    )
    if existing:
        return {
            "error": "storage_area_occupied",
            "message": f"Another shelf exists within {STORAGE_COLLISION_RADIUS*100:.0f}cm radius.",
            "conflict_shelf_id": str(existing["_id"]),
            "existing_shelf_location": {
                "storage_x": existing["storage_x"],
                "storage_y": existing["storage_y"],
            },
            "requested_location": {
                "storage_x": storage_x,
                "storage_y": storage_y,
            },
        }

    # Create document
    doc = {
        "warehouse_id": data["warehouse_id"],

        "current_x": current_x,
        "current_y": current_y,
        "current_yaw": current_yaw,

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

    # Generate AprilTag
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
# UPDATE SHELF (NO STORAGE CHANGE)
# =========================================================

def update_shelf(shelf_id: str, data: dict):
    db = get_db()
    try:
        oid = ObjectId(shelf_id)
    except Exception:
        return None

    manual_reset = bool(data.pop("manual_reset", False))
    storage_fields = {"storage_x", "storage_y", "storage_yaw"}

    if not manual_reset and any(k in data for k in storage_fields):
        return {
            "error": "forbidden_to_modify_storage",
            "message": "Storage location is immutable. Use admin endpoint."
        }

    # Normalize numeric fields
    for key in storage_fields | {"current_x", "current_y", "current_yaw"}:
        if key in data and data[key] is not None:
            data[key] = float(data[key])

    data["updated_at"] = datetime.utcnow()

    db.shelves.update_one(
        {"_id": oid, "deleted": False},
        {"$set": data},
    )

    return get_shelf(shelf_id)


# =========================================================
# ADMIN — SET STORAGE LOCATION (RADIUS SAFE)
# =========================================================

def set_shelf_storage_location(
    shelf_id: str,
    x: float,
    y: float,
    yaw: float = 0.0,
) -> dict:
    """
    Admin function to set a shelf's storage location.
    Checks for collisions with other shelves but excludes itself.
    
    Example valid moves for multiple shelves:
    - Moving shelf to a location >0.5m away from all others
    - Each shelf needs unique storage coordinates
    """
    db = get_db()

    try:
        oid = ObjectId(shelf_id)
    except Exception:
        return {"error": "invalid_shelf_id"}

    shelf = db.shelves.find_one({"_id": oid, "deleted": False})
    if not shelf:
        return {"error": "shelf_not_found"}

    # Prevent collision within radius
    # IMPORTANT: exclude_id=oid excludes this shelf from the collision check
    existing = shelf_exists_within_radius(
        db=db,
        warehouse_id=shelf["warehouse_id"],
        x=float(x),
        y=float(y),
        exclude_id=oid,  # Exclude self from collision detection
    )
    if existing:
        return {
            "error": "storage_area_occupied",
            "message": f"Another shelf exists within {STORAGE_COLLISION_RADIUS*100:.0f}cm radius.",
            "conflict_shelf_id": str(existing["_id"]),
            "existing_shelf_location": {
                "storage_x": existing["storage_x"],
                "storage_y": existing["storage_y"],
            },
            "requested_location": {
                "storage_x": float(x),
                "storage_y": float(y),
            },
        }

    db.shelves.update_one(
        {"_id": oid},
        {
            "$set": {
                "storage_x": float(x),
                "storage_y": float(y),
                "storage_yaw": float(yaw),
                "updated_at": datetime.utcnow(),
            }
        },
    )

    return get_shelf(shelf_id)


# =========================================================
# SOFT DELETE
# =========================================================

def soft_delete_shelf(shelf_id: str) -> bool:
    """
    Soft delete a shelf by marking it as deleted.
    Does not remove it from the database.
    """
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