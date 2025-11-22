from datetime import datetime
from typing import Optional, List, Dict, Any
from bson import ObjectId
from .utils_service import serialize
from ..extensions import get_db


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
