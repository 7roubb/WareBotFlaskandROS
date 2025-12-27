"""
Real-time task management API endpoints.
Handles task lifecycle and robot position updates for live map.
Properly integrates shelf restoration with WebSocket updates.
"""

from flask import Blueprint, request, jsonify, current_app
from datetime import datetime
from ..services.task_service import (
    update_task_with_robot_position,
    get_task_map_view,
    get_all_tasks_map_view,
    emit_task_update_websocket,
    emit_all_tasks_update_websocket,
    update_task_status,
)

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
def update_task_status_route(task_id):
    """
    Update task status and trigger associated actions (shelf restoration, etc).
    
    Body: {
        "old_status": str,
        "new_status": str,
        "current_target": str ("SHELF" or "DROP_ZONE"),
        "robot_x": float (optional),
        "robot_y": float (optional)
    }
    
    This endpoint:
    1. Updates task status (which triggers shelf restoration if needed)
    2. Emits WebSocket updates to all subscribers
    3. Handles RETURN_SHELF completion with shelf location notification
    """
    data = request.get_json()
    
    old_status = data.get("old_status")
    new_status = data.get("new_status")
    current_target = data.get("current_target")
    robot_x = data.get("robot_x")
    robot_y = data.get("robot_y")
    
    # Update task status (this triggers shelf restoration if needed via update_task_status())
    success = update_task_status(task_id, new_status)
    
    if success:
        socketio = current_app.extensions.get("socketio")
        
        # Fetch updated task data for WebSocket notifications
        try:
            map_data = get_task_map_view(task_id)
        except Exception as e:
            current_app.logger.error(f"Failed to get task map view: {e}")
            map_data = None

        if socketio:
            from ..routes.task_websocket import (
                emit_task_status_change,
                emit_shelf_location_fixed
            )
            
            # Emit task update
            emit_task_update_websocket(socketio, task_id)
            
            # Emit status change event
            emit_task_status_change(
                socketio,
                task_id,
                old_status,
                new_status,
                current_target,
                robot_x,
                robot_y
            )

            # ✓ CRITICAL: If this was RETURN_SHELF completion, notify of restoration
            if map_data and map_data.get("type") == "RETURN_SHELF" and new_status == "COMPLETED":
                shelf = map_data.get("shelf", {})
                storage = shelf.get("storage", {})
                
                # Emit shelf restoration confirmation
                emit_shelf_location_fixed(
                    socketio,
                    task_id,
                    shelf.get("id"),
                    storage.get("x"),
                    storage.get("y")
                )
                
                # ✓ Also emit shelf location update to shelf subscribers
                try:
                    from ..routes.api_shelves import emit_shelf_location_update
                    emit_shelf_location_update({
                        "id": shelf.get("id"),
                        "current_x": storage.get("x"),
                        "current_y": storage.get("y"),
                        "current_yaw": storage.get("yaw"),
                    })
                except Exception as e:
                    current_app.logger.warning(f"Failed to emit shelf location update: {e}")
            
            # Broadcast all tasks update to map subscribers
            emit_all_tasks_update_websocket(socketio)
        
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


@task_realtime_bp.route("/<task_id>/restore-shelf", methods=["POST"])
def restore_shelf_via_task(task_id):
    """
    Restore shelf to storage via task API.
    
    Useful for:
    - Manual restoration if task gets stuck
    - Administrative recovery operations
    - Testing shelf restoration workflow
    
    This endpoint:
    1. Fetches the task and shelf_id
    2. Calls shelf restoration service
    3. Emits WebSocket updates
    4. Returns confirmation with location data
    """
    from bson import ObjectId
    from ..extensions import get_db
    from ..services.shelf_location_service import (
        restore_shelf_to_storage_location,
        get_shelf_location_info
    )
    
    db = get_db()
    
    try:
        oid = ObjectId(task_id)
    except Exception:
        return jsonify({"error": "invalid_task_id"}), 400
    
    task = db.tasks.find_one({"_id": oid})
    if not task:
        return jsonify({"error": "task_not_found"}), 404
    
    shelf_id = task.get("shelf_id")
    if not shelf_id:
        return jsonify({"error": "task_has_no_shelf"}), 400
    
    try:
        # Restore shelf to storage
        success = restore_shelf_to_storage_location(shelf_id, task_id=task_id)
        
        if not success:
            return jsonify({
                "error": "restoration_failed",
                "message": f"Failed to restore shelf {shelf_id}"
            }), 500
        
        # Get updated location info
        location_info = get_shelf_location_info(shelf_id)
        
        # Emit WebSocket updates
        socketio = current_app.extensions.get("socketio")
        if socketio:
            # Emit shelf location update
            try:
                from ..routes.api_shelves import emit_shelf_location_update
                emit_shelf_location_update({
                    "id": shelf_id,
                    "current_x": location_info.get("current_x"),
                    "current_y": location_info.get("current_y"),
                    "current_yaw": location_info.get("current_yaw"),
                })
            except Exception as e:
                current_app.logger.warning(f"Failed to emit shelf location update: {e}")
            
            # Emit task update
            emit_task_update_websocket(socketio, task_id)
            
            # Emit all tasks update
            emit_all_tasks_update_websocket(socketio)
        
        return jsonify({
            "success": True,
            "task_id": task_id,
            "shelf_id": shelf_id,
            "restored_to": {
                "x": location_info.get("storage_x"),
                "y": location_info.get("storage_y"),
                "yaw": location_info.get("storage_yaw"),
            },
            "current_location": {
                "x": location_info.get("current_x"),
                "y": location_info.get("current_y"),
                "yaw": location_info.get("current_yaw"),
            },
            "message": f"Shelf {shelf_id} successfully restored to storage",
            "timestamp": datetime.utcnow().isoformat(),
        }), 200
    
    except Exception as e:
        current_app.logger.error(
            f"Failed to restore shelf via task API: {str(e)}",
            exc_info=True
        )
        return jsonify({
            "error": "restoration_error",
            "details": str(e)
        }), 500


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