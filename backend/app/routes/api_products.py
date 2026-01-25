from flask import Blueprint, request, jsonify, current_app
from flask_jwt_extended import jwt_required, get_jwt
from pydantic import ValidationError

from ..models import ProductCreate, ProductUpdate
from ..services import (
    create_product, list_products, get_product,
    update_product, soft_delete_product, search_products_by_name
)


products_bp = Blueprint("products", __name__)


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


def _emit_product_update(data, action):
    """Helper to emit WebSocket event safely"""
    try:
        if "socketio" in current_app.extensions:
            socketio = current_app.extensions["socketio"]
            from .ws_live_updates import emit_product_update
            # Assuming 'data' is the product dict or similar
            if data:
                # Ensure we don't fail serialization
                from ..services.utils_service import serialize
                serialized = serialize(data) if isinstance(data, dict) else data
                emit_product_update(socketio, serialized, action)
    except Exception as e:
        current_app.logger.error(f"Failed to emit product update: {e}")


# =========================================================
# PRODUCT CRUD
# =========================================================
@products_bp.route("", methods=["POST"])
@admin_required
def create_product_route():
    try:
        data = ProductCreate(**request.json).dict()
    except ValidationError as e:
        return handle_validation_error(e)
    
    new_product = create_product(data)
    
    # Emit Websocket Event
    _emit_product_update(new_product, "CREATED")
    
    return jsonify(new_product), 201


@products_bp.route("", methods=["GET"])
def list_products_route():
    return jsonify(list_products())


@products_bp.route("/<id>", methods=["GET"])
def get_product_route(id):
    p = get_product(id)
    return jsonify(p) if p else ({"error": "not_found"}, 404)


@products_bp.route("/<id>", methods=["PUT"])
@admin_required
def update_product_route(id):
    try:
        data = ProductUpdate(**request.json).dict(exclude_none=True)
    except ValidationError as e:
        return handle_validation_error(e)
    
    result = update_product(id, data)
    if not result:
        return jsonify({"error": "not_found"}), 404
        
    # Emit Websocket Event
    _emit_product_update(result, "UPDATED")
    
    return jsonify(result), 200


@products_bp.route("/<id>", methods=["DELETE"])
@admin_required
def delete_product_route(id):
    if not soft_delete_product(id):
        return {"error": "not_found"}, 404
    
    # Emit Websocket Event with ID
    _emit_product_update({"_id": id}, "DELETED")
    
    return {"status": "deleted"}


@products_bp.route("/search", methods=["GET"])
def search_product_route():
    q = request.args.get("q", "").strip()
    return jsonify(search_products_by_name(q)) if q else []
