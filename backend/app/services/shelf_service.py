from datetime import datetime
from bson import ObjectId
from ..extensions import get_db
from .utils_service import serialize
from .apriltag_service import generate_apriltag, upload_apriltag, MAX_TAG_IDS


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

    # Initialize storage (immutable) coordinates. If provided, use them,
    # otherwise initialize storage to the current coordinates.
    storage_x = data.get("storage_x")
    storage_y = data.get("storage_y")
    storage_yaw = data.get("storage_yaw")

    if storage_x is None:
        storage_x = data["x_coord"]
    if storage_y is None:
        storage_y = data["y_coord"]
    if storage_yaw is None:
        storage_yaw = data.get("yaw", 0.0)

    doc["storage_x"] = float(storage_x)
    doc["storage_y"] = float(storage_y)
    doc["storage_yaw"] = float(storage_yaw)

    res = db.shelves.insert_one(doc)
    shelf_id = str(res.inserted_id)

    raw_int = int(res.inserted_id.binary.hex(), 16)
    tag_id = raw_int % MAX_TAG_IDS

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

    # Prevent accidental modification of immutable storage coordinates
    # Only allow when caller explicitly passes 'manual_reset': True
    manual_reset = bool(data.pop("manual_reset", False))

    # If storage fields present without manual_reset flag, reject the update
    storage_keys = {"storage_x", "storage_y", "storage_yaw"}
    if not manual_reset and any(k in data for k in storage_keys):
        return {"error": "forbidden_to_modify_storage", "message": "storage_x/storage_y/storage_yaw are immutable; use admin storage-reset endpoint"}

    # Normalize numeric fields
    if "storage_x" in data:
        data["storage_x"] = float(data["storage_x"]) if data["storage_x"] is not None else None
    if "storage_y" in data:
        data["storage_y"] = float(data["storage_y"]) if data["storage_y"] is not None else None
    if "storage_yaw" in data:
        data["storage_yaw"] = float(data["storage_yaw"]) if data["storage_yaw"] is not None else None

    data["updated_at"] = datetime.utcnow()
    db.shelves.update_one({"_id": oid, "deleted": False}, {"$set": data})
    return get_shelf(id)


def set_shelf_storage_location(shelf_id: str, x: float, y: float, yaw: float = 0.0) -> bool:
    """
    Explicit admin-only setter for storage (immutable) coordinates.
    Returns True if successful.
    """
    from .shelf_location_service import initialize_shelf_storage_location

    return initialize_shelf_storage_location(shelf_id, x, y, yaw)


def soft_delete_shelf(id: str):
    try:
        oid = ObjectId(id)
    except:
        return False

    res = get_db().shelves.update_one({"_id": oid}, {"$set": {"deleted": True}})
    return res.modified_count > 0
