from flask import Blueprint, request, jsonify, current_app
from datetime import datetime
from flask_jwt_extended import jwt_required, get_jwt
from pydantic import ValidationError
from bson import ObjectId

from ..models import ZoneCreate, ZoneUpdate
from ..extensions import get_db

zones_bp = Blueprint("zones", __name__)


def admin_required(fn):
    from functools import wraps

    @wraps(fn)
    @jwt_required()
    def wrapper(*args, **kwargs):
        claims = get_jwt()
        if claims.get("role") != "ADMIN":
            return jsonify({"error": "forbidden"}), 403
        return fn(*args, **kwargs)

    return wrapper


def handle_validation_error(err: ValidationError):
    return jsonify({"error": "validation_error", "details": err.errors()}), 400


@zones_bp.route("", methods=["POST"])
@admin_required
def create_zone_route():
    try:
        data = ZoneCreate(**request.json).dict()
    except ValidationError as e:
        return handle_validation_error(e)

    db = get_db()

    # prevent duplicates by zone_id
    existing = db.zones.find_one({"zone_id": data["zone_id"], "deleted": False})
    if existing:
        return {"error": "zone_exists"}, 400

    doc = {
        "zone_id": data["zone_id"],
        "name": data.get("name"),
        "x": float(data.get("x")),
        "y": float(data.get("y")),
        "yaw": float(data.get("yaw") or 0.0),
        "deleted": False,
    }

    res = db.zones.insert_one(doc)
    doc = db.zones.find_one({"_id": res.inserted_id})
    # convert ObjectId to string id for response
    doc["id"] = str(doc.get("_id"))
    # remove the raw ObjectId so jsonify won't fail
    if "_id" in doc:
        del doc["_id"]
    return jsonify(doc), 201


@zones_bp.route("", methods=["GET"])
def list_zones_route():
    db = get_db()
    items = list(db.zones.find({"deleted": False}))
    out = []
    for z in items:
        z["id"] = str(z.get("_id"))
        if "_id" in z:
            del z["_id"]
        out.append(z)
    return jsonify(out)


@zones_bp.route("/<id>", methods=["GET"])
def get_zone_route(id):
    db = get_db()
    # try by object id
    zone = None
    try:
        oid = ObjectId(id)
        zone = db.zones.find_one({"_id": oid, "deleted": False})
    except Exception:
        zone = db.zones.find_one({"zone_id": id, "deleted": False})

    if not zone:
        return {"error": "not_found"}, 404

    zone["id"] = str(zone.get("_id"))
    if "_id" in zone:
        del zone["_id"]
    return jsonify(zone)


@zones_bp.route("/<id>", methods=["DELETE"]) 
@admin_required
def delete_zone_route(id):
    db = get_db()
    # try by object id
    try:
        oid = ObjectId(id)
        res = db.zones.delete_one({"_id": oid})
    except Exception:
        res = db.zones.delete_one({"zone_id": id})

    if res.deleted_count == 0:
        return {"error": "not_found"}, 404

    return {"result": "deleted"}, 200
