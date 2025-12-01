from datetime import datetime
from ..extensions import get_db


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
        products = list(db.products.find({"shelf_id": sid, "deleted": False}))
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
