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

    data["updated_at"] = datetime.utcnow()
    db.shelves.update_one({"_id": oid, "deleted": False}, {"$set": data})
    return get_shelf(id)


def soft_delete_shelf(id: str):
    try:
        oid = ObjectId(id)
    except:
        return False

    res = get_db().shelves.update_one({"_id": oid}, {"$set": {"deleted": True}})
    return res.modified_count > 0
