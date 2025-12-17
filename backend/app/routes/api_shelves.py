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


@shelves_bp.route("/<id>/restore-location", methods=["PUT"])
@admin_required
def restore_shelf_location_route(id):
    """
    CRITICAL: Restore shelf to its STORAGE location (immutable original position).
    
    Used when:
    - RETURN_SHELF task needs to complete
    - Manual reset needed after error
    - Shelf stuck at drop zone needs recovery
    
    This endpoint bypasses the normal location update and directly restores
    to the shelf's storage_x, storage_y coordinates.
    """
    try:
        from ..services.shelf_location_service import restore_shelf_to_storage_location
        
        success = restore_shelf_to_storage_location(id)
        if not success:
            return jsonify({"error": "shelf_not_found", "shelf_id": id}), 404
        
        from ..services.shelf_location_service import get_shelf_location_info
        location_info = get_shelf_location_info(id)
        
        return jsonify({
            "status": "restored",
            "shelf_id": id,
            "location": location_info
        }), 200
        
    except Exception as e:
        return jsonify({"error": "restoration_failed", "details": str(e)}), 500


@shelves_bp.route("/<id>/location-info", methods=["GET"])
def get_shelf_location_info_route(id):
    """
    Get complete location information for a shelf.
    
    Returns:
    {
        "shelf_id": "...",
        "storage_x": 10.0,        # Immutable original location
        "storage_y": 20.0,
        "storage_yaw": 0.0,
        "current_x": 35.2,        # Current display location
        "current_y": 40.1,
        "current_yaw": 0.5,
        "location_status": "AT_DROP_ZONE",  # Semantic status
        "last_task_id": "...",
        "updated_at": "2025-01-20T10:30:00Z"
    }
    """
    try:
        from ..services.shelf_location_service import get_shelf_location_info
        
        location_info = get_shelf_location_info(id)
        if not location_info:
            return jsonify({"error": "shelf_not_found", "shelf_id": id}), 404
        
        return jsonify(location_info), 200
        
    except Exception as e:
        return jsonify({"error": "failed_to_get_location_info", "details": str(e)}), 500


@shelves_bp.route("/<id>/location-history", methods=["GET"])
def get_shelf_location_history_route(id):
    """
    Get location history/audit trail for a shelf.
    
    Query params:
    - limit: Maximum number of records (default 50)
    
    Returns array of location change records with timestamps.
    """
    try:
        from ..services.shelf_location_service import get_shelf_location_history
        
        limit = request.args.get("limit", 50, type=int)
        history = get_shelf_location_history(id, limit=limit)
        
        return jsonify({
            "shelf_id": id,
            "history": history
        }), 200
        
    except Exception as e:
        return jsonify({"error": "failed_to_get_history", "details": str(e)}), 500


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
