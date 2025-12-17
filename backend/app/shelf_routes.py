"""
Shelf correction routes for the WareBot backend.
Handles auto-correction suggestions, manual reviews, and correction history.
"""

from flask import Blueprint, request, jsonify
from .services.robot_service import (
    update_shelf_location,
    apply_shelf_correction,
    suggest_shelf_review,
    get_shelf_location,
    get_shelf_correction_history,
    list_shelf_review_suggestions,
    resolve_shelf_review,
)

shelf_bp = Blueprint("shelf", __name__, url_prefix="/api/shelf")


@shelf_bp.route("/location/<shelf_id>", methods=["GET"])
def get_shelf(shelf_id):
    """Get shelf location and metadata."""
    shelf = get_shelf_location(shelf_id)
    if not shelf:
        return jsonify({"error": "Shelf not found"}), 404
    return jsonify(shelf), 200


@shelf_bp.route("/location/<shelf_id>", methods=["PUT"])
def update_shelf(shelf_id):
    """Update shelf location."""
    data = request.get_json()
    
    try:
        x = float(data.get("x"))
        y = float(data.get("y"))
        uncertainty = float(data.get("uncertainty", 0.0))
        observation_count = int(data.get("observation_count", 1))
    except (TypeError, ValueError):
        return jsonify({"error": "Invalid coordinates or metadata"}), 400
    
    success = update_shelf_location(shelf_id, x, y, uncertainty, observation_count)
    
    return jsonify({
        "success": success,
        "shelf_id": shelf_id,
        "x": x,
        "y": y,
    }), 200 if success else 500


@shelf_bp.route("/correction/apply", methods=["POST"])
def apply_correction():
    """Apply a shelf location correction."""
    data = request.get_json()
    
    try:
        shelf_id = data.get("shelf_id")
        old_x = float(data.get("old_x"))
        old_y = float(data.get("old_y"))
        new_x = float(data.get("new_x"))
        new_y = float(data.get("new_y"))
        drift_distance = float(data.get("drift_distance", 0.0))
        action_type = data.get("action_type", "AUTO_CORRECTED")
    except (TypeError, ValueError, KeyError):
        return jsonify({"error": "Invalid correction data"}), 400
    
    success = apply_shelf_correction(
        shelf_id, old_x, old_y, new_x, new_y, drift_distance, action_type
    )
    
    return jsonify({
        "success": success,
        "shelf_id": shelf_id,
        "old_position": {"x": old_x, "y": old_y},
        "new_position": {"x": new_x, "y": new_y},
        "drift_distance": drift_distance,
        "action_type": action_type,
    }), 200 if success else 500


@shelf_bp.route("/review/suggest", methods=["POST"])
def create_review_suggestion():
    """Create a shelf review suggestion for manual verification."""
    data = request.get_json()
    
    try:
        shelf_id = data.get("shelf_id")
        current_x = float(data.get("current_x"))
        current_y = float(data.get("current_y"))
        uncertainty = float(data.get("uncertainty", 0.0))
        observation_count = int(data.get("observation_count", 1))
    except (TypeError, ValueError, KeyError):
        return jsonify({"error": "Invalid review suggestion data"}), 400
    
    suggestion_id = suggest_shelf_review(
        shelf_id, current_x, current_y, uncertainty, observation_count
    )
    
    return jsonify({
        "success": True,
        "suggestion_id": suggestion_id,
        "shelf_id": shelf_id,
        "status": "PENDING",
    }), 201


@shelf_bp.route("/review/suggestions", methods=["GET"])
def get_review_suggestions():
    """Get pending shelf review suggestions."""
    status = request.args.get("status", "PENDING")
    suggestions = list_shelf_review_suggestions(status)
    
    return jsonify({
        "status": status,
        "count": len(suggestions),
        "suggestions": suggestions,
    }), 200


@shelf_bp.route("/review/<suggestion_id>/resolve", methods=["POST"])
def resolve_review(suggestion_id):
    """Resolve a shelf review suggestion."""
    data = request.get_json()
    
    approved = data.get("approved", False)
    notes = data.get("notes", "")
    
    success = resolve_shelf_review(suggestion_id, approved, notes)
    
    return jsonify({
        "success": success,
        "suggestion_id": suggestion_id,
        "status": "APPROVED" if approved else "REJECTED",
    }), 200 if success else 500


@shelf_bp.route("/correction-history/<shelf_id>", methods=["GET"])
def get_correction_history(shelf_id):
    """Get correction history for a shelf."""
    limit = request.args.get("limit", 10, type=int)
    history = get_shelf_correction_history(shelf_id, limit)
    
    return jsonify({
        "shelf_id": shelf_id,
        "correction_count": len(history),
        "corrections": history,
    }), 200
