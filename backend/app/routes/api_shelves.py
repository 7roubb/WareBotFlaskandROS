# shelves_bp routes - FIXED VERSION with WebSocket support

from flask import Blueprint, request, jsonify, current_app
from flask_jwt_extended import jwt_required, get_jwt
from pydantic import ValidationError
from datetime import datetime

from ..models import ShelfCreate, ShelfUpdate
from ..services import (
    create_shelf,
    list_shelves,
    get_shelf,
    update_shelf,
    soft_delete_shelf,
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
    return jsonify({
        "error": "validation_error",
        "details": err.errors()
    }), 400


def emit_shelf_update(shelf_data):
    """Emit shelf update to all WebSocket subscribers"""
    try:
        socketio = current_app.extensions.get("socketio")
        if socketio and shelf_data:
            socketio.emit("shelf_update", {
                "id": str(shelf_data.get("_id")),
                "warehouse_id": shelf_data.get("warehouse_id"),
                "current_x": shelf_data.get("current_x"),
                "current_y": shelf_data.get("current_y"),
                "current_yaw": shelf_data.get("current_yaw"),
                "storage_x": shelf_data.get("storage_x"),
                "storage_y": shelf_data.get("storage_y"),
                "storage_yaw": shelf_data.get("storage_yaw"),
                "level": shelf_data.get("level"),
                "status": shelf_data.get("status"),
                "available": shelf_data.get("available"),
                "timestamp": datetime.utcnow().isoformat(),
            }, room="shelves_room")
            print(f"[WebSocket] Emitted shelf_update for {shelf_data.get('_id')}")
    except Exception as e:
        print(f"[WebSocket Error] Failed to emit shelf_update: {e}")


def emit_shelf_location_update(shelf_data):
    """Emit shelf location-specific update"""
    try:
        socketio = current_app.extensions.get("socketio")
        if socketio and shelf_data:
            socketio.emit("shelf_location_update", {
                "id": str(shelf_data.get("_id")),
                "current_x": shelf_data.get("current_x"),
                "current_y": shelf_data.get("current_y"),
                "current_yaw": shelf_data.get("current_yaw"),
                "timestamp": datetime.utcnow().isoformat(),
            }, room="shelves_room")
            print(f"[WebSocket] Emitted shelf_location_update for {shelf_data.get('_id')}")
    except Exception as e:
        print(f"[WebSocket Error] Failed to emit shelf_location_update: {e}")


# =========================================================
# SHELF CRUD
# =========================================================

@shelves_bp.route("", methods=["POST"])
@admin_required
def create_shelf_route():
    try:
        data = ShelfCreate(**(request.json or {})).dict()
    except ValidationError as e:
        return handle_validation_error(e)

    result = create_shelf(data)
    
    # ✓ Emit WebSocket for new shelf
    if result and not isinstance(result, dict) or result.get("error") is None:
        emit_shelf_update(result)
    
    return jsonify(result), 201


@shelves_bp.route("", methods=["GET"])
def list_shelves_route():
    return jsonify(list_shelves()), 200


@shelves_bp.route("/<id>", methods=["GET"])
def get_shelf_route(id):
    shelf = get_shelf(id)
    return jsonify(shelf) if shelf else ({"error": "not_found"}, 404)


@shelves_bp.route("/<id>", methods=["PUT"])
@admin_required
def update_shelf_route(id):
    """Update shelf metadata (warehouse_id, level, available, status)
    
    NOTE: For location updates, use the dedicated /location endpoint
    NOTE: For storage location updates, use the dedicated /storage endpoint (admin only)
    """
    try:
        data = ShelfUpdate(**(request.json or {})).dict(exclude_none=True)
    except ValidationError as e:
        return handle_validation_error(e)

    result = update_shelf(id, data)

    if isinstance(result, dict) and result.get("error") == "forbidden_to_modify_storage":
        return jsonify(result), 403

    if result:
        # ✓ Emit WebSocket for metadata update
        emit_shelf_update(result)
    
    return jsonify(result) if result else ({"error": "not_found"}, 404)


# =========================================================
# REAL-TIME LOCATION UPDATE (CURRENT LOCATION ONLY)
# =========================================================

@shelves_bp.route("/<id>/location", methods=["PUT"])
@admin_required
def update_shelf_location_route(id):
    """
    Update CURRENT shelf location only (used by robots / live updates)
    
    ✓ Used for: Real-time robot position tracking
    ✓ Updates: current_x, current_y, current_yaw
    ✗ Does NOT update: storage_x, storage_y (immutable)

    Body:
    {
        "current_x": float,
        "current_y": float,
        "current_yaw": float (optional)
    }
    """
    try:
        data = request.json or {}

        if "current_x" not in data or "current_y" not in data:
            return jsonify({
                "error": "missing_coordinates",
                "required": ["current_x", "current_y"]
            }), 400

        payload = {
            "current_x": float(data["current_x"]),
            "current_y": float(data["current_y"]),
        }

        if "current_yaw" in data:
            payload["current_yaw"] = float(data["current_yaw"])

        result = update_shelf(id, payload)
        
        if result:
            # ✓ Emit WebSocket for location update
            emit_shelf_location_update(result)
        
        return jsonify(result) if result else ({"error": "not_found"}, 404)

    except (ValueError, TypeError) as e:
        return jsonify({"error": "invalid_coordinates", "details": str(e)}), 400


# =========================================================
# RESTORE TO STORAGE LOCATION
# =========================================================

@shelves_bp.route("/<id>/restore-location", methods=["PUT"])
@admin_required
def restore_shelf_location_route(id):
    """
    Restore shelf to its STORAGE (home) location.
    Used by RETURN_SHELF task completion or manual recovery.
    
    This copies: storage_x → current_x, storage_y → current_y, storage_yaw → current_yaw
    """
    try:
        from ..services.shelf_location_service import (
            restore_shelf_to_storage_location,
            get_shelf_location_info,
        )

        if not restore_shelf_to_storage_location(id):
            return jsonify({"error": "shelf_not_found", "shelf_id": id}), 404

        location_info = get_shelf_location_info(id)

        # ✓ Get the updated shelf document for emission
        result = get_shelf(id)
        if result:
            emit_shelf_location_update(result)

        return jsonify({
            "status": "restored",
            "shelf_id": id,
            "location": location_info,
        }), 200

    except Exception as e:
        return jsonify({
            "error": "restoration_failed",
            "details": str(e)
        }), 500


@shelves_bp.route("/<id>/restore", methods=["POST"])
@admin_required
def restore_shelf_route(id):
    """
    POST alias for restore-location (compatibility).
    """
    try:
        from ..services.shelf_location_service import (
            restore_shelf_to_storage_location,
            get_shelf_location_info,
        )

        if not restore_shelf_to_storage_location(id):
            return jsonify({"error": "shelf_not_found", "shelf_id": id}), 404

        location_info = get_shelf_location_info(id)

        # ✓ Get the updated shelf document for emission
        result = get_shelf(id)
        if result:
            emit_shelf_location_update(result)

        return jsonify({
            "success": True,
            "shelf_id": id,
            "location_status": "STORED",
            "storage": {
                "x": location_info.get("storage_x"),
                "y": location_info.get("storage_y"),
                "yaw": location_info.get("storage_yaw"),
            },
        }), 200

    except Exception as e:
        return jsonify({
            "error": "restoration_failed",
            "details": str(e)
        }), 500


# =========================================================
# ADMIN — SET STORAGE LOCATION
# =========================================================

@shelves_bp.route("/<id>/storage", methods=["PUT"])
@admin_required
def set_shelf_storage_route(id):
    """
    Admin-only: override STORAGE (home) location.
    
    ✓ Used for: Manual warehouse rebalancing
    ✓ Updates: storage_x, storage_y, storage_yaw
    ✗ Does NOT affect: current_x, current_y
    
    Body:
    {
        "storage_x": float,
        "storage_y": float,
        "storage_yaw": float (optional)
    }
    """
    try:
        data = request.json or {}

        if "storage_x" not in data or "storage_y" not in data:
            return jsonify({
                "error": "missing_storage_coordinates",
                "required": ["storage_x", "storage_y"]
            }), 400

        from ..services.shelf_service import set_shelf_storage_location
        from ..services.shelf_location_service import get_shelf_location_info

        success = set_shelf_storage_location(
            id,
            float(data["storage_x"]),
            float(data["storage_y"]),
            float(data.get("storage_yaw", 0.0)),
        )

        if not success:
            return jsonify({"error": "shelf_not_found", "shelf_id": id}), 404

        location_info = get_shelf_location_info(id)

        # ✓ Get the updated shelf document for emission
        result = get_shelf(id)
        if result:
            emit_shelf_update(result)  # Full update since storage changed

        return jsonify({
            "success": True,
            "shelf_id": id,
            "location_status": "STORED",
            "storage": {
                "x": location_info.get("storage_x"),
                "y": location_info.get("storage_y"),
                "yaw": location_info.get("storage_yaw"),
            },
        }), 200

    except (ValueError, TypeError) as e:
        return jsonify({"error": "invalid_input", "details": str(e)}), 400
    except Exception as e:
        return jsonify({"error": "failed_to_set_storage", "details": str(e)}), 500


# =========================================================
# LOCATION INFO & HISTORY
# =========================================================

@shelves_bp.route("/<id>/location-info", methods=["GET"])
def get_shelf_location_info_route(id):
    try:
        from ..services.shelf_location_service import get_shelf_location_info

        info = get_shelf_location_info(id)
        return jsonify(info) if info else ({"error": "shelf_not_found"}, 404)

    except Exception as e:
        return jsonify({
            "error": "failed_to_get_location_info",
            "details": str(e)
        }), 500


@shelves_bp.route("/<id>/location-history", methods=["GET"])
def get_shelf_location_history_route(id):
    try:
        from ..services.shelf_location_service import get_shelf_location_history

        limit = request.args.get("limit", 50, type=int)
        history = get_shelf_location_history(id, limit=limit)

        return jsonify({
            "shelf_id": id,
            "history": history,
        }), 200

    except Exception as e:
        return jsonify({
            "error": "failed_to_get_history",
            "details": str(e)
        }), 500


# =========================================================
# DELETE
# =========================================================

@shelves_bp.route("/<id>", methods=["DELETE"])
@admin_required
def delete_shelf_route(id):
    if not soft_delete_shelf(id):
        return {"error": "not_found"}, 404
    
    # ✓ Could emit deletion event if desired
    try:
        socketio = current_app.extensions.get("socketio")
        if socketio:
            socketio.emit("shelf_deleted", {
                "id": id,
                "timestamp": datetime.utcnow().isoformat(),
            }, room="shelves_room")
    except Exception as e:
        print(f"[WebSocket Error] Failed to emit shelf_deleted: {e}")
    
    return {"status": "deleted"}, 200


# =========================================================
# SHELF → PRODUCTS
# =========================================================

@shelves_bp.route("/<id>/products", methods=["GET"])
def shelf_products_route(id):
    return jsonify(get_products_for_shelf(id)), 200