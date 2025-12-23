"""
Real-time task management API endpoints.
Handles task lifecycle and robot position updates for live map.
"""

from flask import Blueprint, request, jsonify, current_app
from datetime import datetime
from ..services.task_service import (
    update_task_with_robot_position,
    get_task_map_view,
    get_all_tasks_map_view,
    emit_task_update_websocket,
    emit_all_tasks_update_websocket,
)
from .task_websocket import emit_task_status_change, emit_shelf_location_fixed

task_realtime_bp = Blueprint("task_realtime", __name__, url_prefix="/api/tasks/realtime")


@task_realtime_bp.route("/<task_id>/position", methods=["POST"])
def update_robot_position(task_id):
    """
    Update robot's current position for a task.
    Shelf location remains FIXED and unchanged.
    
    Body: {
        "robot_x": float,
        "robot_y": float,
        "status": str (e.g., "MOVING_TO_SHELF", "PICKING", "MOVING_TO_DROP", "DROPPING", "RETURNING")
    }
    """
    data = request.get_json()
    
    try:
        robot_x = float(data.get("robot_x", 0))
        robot_y = float(data.get("robot_y", 0))
        status = data.get("status", "IN_PROGRESS")
    except (TypeError, ValueError):
        return jsonify({"error": "Invalid robot position data"}), 400
    
    # Update task with new robot position
    success = update_task_with_robot_position(task_id, robot_x, robot_y, status)
    
    if success:
        # Emit WebSocket update for real-time map
        socketio = current_app.extensions.get("socketio")
        if socketio:
            emit_task_update_websocket(socketio, task_id)
        
        return jsonify({
            "success": True,
            "task_id": task_id,
            "robot": {"x": robot_x, "y": robot_y},
            "status": status,
            "timestamp": datetime.utcnow().isoformat(),
        }), 200
    else:
        return jsonify({"error": "Task not found or update failed"}), 404


@task_realtime_bp.route("/<task_id>/status", methods=["PUT"])
def update_task_status(task_id):
    """
    Update task status and current target.
    
    Body: {
        "old_status": str,
        "new_status": str,
        "current_target": str ("SHELF" or "DROP_ZONE"),
        "robot_x": float (optional),
        "robot_y": float (optional)
    }
    """
    data = request.get_json()
    
    old_status = data.get("old_status")
    new_status = data.get("new_status")
    current_target = data.get("current_target")
    robot_x = data.get("robot_x")
    robot_y = data.get("robot_y")
    
    # Update task
    from ..services.task_service import update_task_status
    success = update_task_status(task_id, new_status)
    
    if success:
        # Emit WebSocket update
        socketio = current_app.extensions.get("socketio")
        # Enrich and emit status-specific events
        try:
            map_data = get_task_map_view(task_id)
        except Exception:
            map_data = None

        if socketio:
            emit_task_update_websocket(socketio, task_id)
            # emit generic task_status_change for subscribers
            emit_task_status_change(socketio, task_id, old_status, new_status, current_target, robot_x, robot_y)

            # If this was a RETURN_SHELF completion, also notify that shelf was restored
            if map_data and map_data.get("type") == "RETURN_SHELF" and new_status == "COMPLETED":
                shelf = map_data.get("shelf", {})
                storage = shelf.get("storage", {})
                emit_shelf_location_fixed(socketio, task_id, shelf.get("id"), storage.get("x"), storage.get("y"))
        
        return jsonify({
            "success": True,
            "task_id": task_id,
            "status_change": {
                "from": old_status,
                "to": new_status,
            },
            "current_target": current_target,
            "timestamp": datetime.utcnow().isoformat(),
        }), 200
    else:
        return jsonify({"error": "Task not found or status update failed"}), 404


@task_realtime_bp.route("/<task_id>", methods=["GET"])
def get_task_for_map(task_id):
    """Get task data formatted for map display."""
    map_data = get_task_map_view(task_id)
    
    if not map_data:
        return jsonify({"error": "Task not found"}), 404
    
    return jsonify({
        "task": map_data,
        "timestamp": datetime.utcnow().isoformat(),
    }), 200


@task_realtime_bp.route("/map/all", methods=["GET"])
def get_all_tasks_for_map():
    """Get all active tasks formatted for map display."""
    tasks = get_all_tasks_map_view()
    
    return jsonify({
        "tasks": tasks,
        "count": len(tasks),
        "timestamp": datetime.utcnow().isoformat(),
    }), 200


@task_realtime_bp.route("/map/robot/<robot_id>", methods=["GET"])
def get_robot_tasks_for_map(robot_id):
    """Get all active tasks for a specific robot."""
    from ..extensions import get_db
    db = get_db()
    
    tasks = list(db.tasks.find({
        "robot_id": robot_id,
        "status": {"$in": ["ACTIVE", "PENDING", "IN_PROGRESS"]}
    }))
    
    map_data = []
    for task in tasks:
        # enrich with shelf storage/current if available
        from ..services.shelf_location_service import get_shelf_location_info
        shelf_info = None
        try:
            shelf_info = get_shelf_location_info(task.get("shelf_id"))
        except Exception:
            shelf_info = None

        if shelf_info:
            storage = {"x": shelf_info.get("storage_x"), "y": shelf_info.get("storage_y")}
            current = {"x": shelf_info.get("current_x"), "y": shelf_info.get("current_y")}
        else:
            storage = {"x": float(task.get("origin_storage_x", task.get("pickup_x", 0))), "y": float(task.get("origin_storage_y", task.get("pickup_y", 0)))}
            current = {"x": float(task.get("current_robot_x", task.get("pickup_x", 0))), "y": float(task.get("current_robot_y", task.get("pickup_y", 0)))}

        map_data.append({
            "task_id": str(task.get("_id")),
            "robot_id": task.get("robot_id"),
            "status": task.get("status"),
            "robot": {
                "x": float(task.get("current_robot_x", 0)),
                "y": float(task.get("current_robot_y", 0)),
            },
            "shelf": {
                "id": task.get("shelf_id"),
                "storage": storage,
                "current": current,
            },
            "drop_zone": {
                "id": task.get("drop_zone_id", task.get("zone_id")),
                "x": float(task.get("drop_x", task.get("zone_x", 0))),
                "y": float(task.get("drop_y", task.get("zone_y", 0))),
            },
        })
    
    return jsonify({
        "robot_id": robot_id,
        "tasks": map_data,
        "count": len(map_data),
        "timestamp": datetime.utcnow().isoformat(),
    }), 200


@task_realtime_bp.route("/broadcast-map-update", methods=["POST"])
def broadcast_map_update():
    """
    Trigger a broadcast of all active tasks map data to all connected clients.
    Useful for syncing map state after batch updates.
    """
    socketio = current_app.extensions.get("socketio")
    
    if socketio:
        emit_all_tasks_update_websocket(socketio)
        return jsonify({
            "success": True,
            "message": "Map update broadcast to all clients",
            "timestamp": datetime.utcnow().isoformat(),
        }), 200
    else:
        return jsonify({"error": "WebSocket not available"}), 500
