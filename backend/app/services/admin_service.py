from datetime import datetime
from werkzeug.security import generate_password_hash, check_password_hash
from ..extensions import get_db
from .utils_service import serialize


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
