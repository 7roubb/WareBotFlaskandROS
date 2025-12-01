from flask import Blueprint, request, jsonify
from flask_jwt_extended import jwt_required, get_jwt
from pydantic import ValidationError

from ..models import ShelfCreate, ShelfUpdate
from ..services import (
    create_shelf, list_shelves, get_shelf,
    update_shelf, soft_delete_shelf,
    get_products_for_shelf,
)

shelves_bp = Blueprint("shelves", __name__)


# =========================================================
# HELPERS
# =========================================================
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


# =========================================================
# SHELF CRUD
# =========================================================
@shelves_bp.route("", methods=["POST"])
@admin_required
def create_shelf_route():
    try:
        data = ShelfCreate(**request.json).dict()
    except ValidationError as e:
        return handle_validation_error(e)
    return jsonify(create_shelf(data)), 201


@shelves_bp.route("", methods=["GET"])
def list_shelves_route():
    return jsonify(list_shelves())


@shelves_bp.route("/<id>", methods=["GET"])
def get_shelf_route(id):
    s = get_shelf(id)
    return jsonify(s) if s else ({"error": "not_found"}, 404)


@shelves_bp.route("/<id>", methods=["PUT"])
@admin_required
def update_shelf_route(id):
    try:
        data = ShelfUpdate(**request.json).dict(exclude_none=True)
    except ValidationError as e:
        return handle_validation_error(e)

    result = update_shelf(id, data)
    return jsonify(result) if result else ({"error": "not_found"}, 404)


@shelves_bp.route("/<id>/location", methods=["PUT"])
@admin_required
def update_shelf_location_route(id):
    """
    Update shelf location (coordinates) in real-time.
    Body: {"x_coord": <float>, "y_coord": <float>, "yaw": <float> (optional)}
    """
    try:
        data = request.json or {}
        x = data.get("x_coord")
        y = data.get("y_coord")
        yaw = data.get("yaw")
        if x is None or y is None:
            return {"error": "missing_coordinates", "required": ["x_coord", "y_coord"]}, 400
        result = update_shelf(id, {"x_coord": float(x), "y_coord": float(y), "yaw": float(yaw or 0.0)})
        return jsonify(result) if result else ({"error": "not_found"}, 404)
    except (ValueError, TypeError) as e:
        return {"error": "invalid_coordinates", "details": str(e)}, 400


@shelves_bp.route("/<id>", methods=["DELETE"])
@admin_required
def delete_shelf_route(id):
    if not soft_delete_shelf(id):
        return {"error": "not_found"}, 404
    return {"status": "deleted"}


# =========================================================
# SHELF → PRODUCTS
# =========================================================
@shelves_bp.route("/<id>/products", methods=["GET"])
def shelf_products_route(id):
    return jsonify(get_products_for_shelf(id))
