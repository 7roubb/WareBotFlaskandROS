from datetime import datetime
from bson import ObjectId
from .product_service import get_product
from ..extensions import get_db


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

    return after


def adjust_product_stock(product_id: str, new_qty: int, reason=None):
    db = get_db()
    product = get_product(product_id)
    if not product:
        raise ValueError("product_not_found")

    before = product["quantity"]

    db.products.update_one(
        {"_id": ObjectId(product_id)},   # <-- تم الإصلاح هنا
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

    return new_qty
