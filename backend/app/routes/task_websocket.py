"""
Real-time task and map WebSocket handlers for live updates.
Emits task status changes, robot positions, and shelf locations in real-time.
"""
from datetime import datetime
from flask import request
from flask_socketio import emit, disconnect, join_room, leave_room, rooms
from ..services.task_service import get_task_map_view, get_all_tasks_map_view


def register_task_websocket_handlers(socketio):
    """Register WebSocket handlers for real-time task updates."""
    
    @socketio.on("connect")
    def on_connect():
        """Client connects to WebSocket."""
        print(f"Client connected: {request.sid}")
        emit("connection_response", {
            "status": "connected",
            "timestamp": datetime.utcnow().isoformat(),
        })
    
    @socketio.on("disconnect")
    def on_disconnect():
        """Client disconnects from WebSocket."""
        print(f"Client disconnected: {request.sid}")
    
    @socketio.on("subscribe_task")
    def on_subscribe_task(data):
        """Subscribe to updates for a specific task."""
        task_id = data.get("task_id")
        if task_id:
            join_room(f"task_{task_id}")
            emit("subscribed", {
                "task_id": task_id,
                "timestamp": datetime.utcnow().isoformat(),
            })
    
    @socketio.on("unsubscribe_task")
    def on_unsubscribe_task(data):
        """Unsubscribe from task updates."""
        task_id = data.get("task_id")
        if task_id:
            leave_room(f"task_{task_id}")
            emit("unsubscribed", {
                "task_id": task_id,
                "timestamp": datetime.utcnow().isoformat(),
            })
    
    @socketio.on("subscribe_map")
    def on_subscribe_map():
        """Subscribe to all tasks map updates."""
        join_room("map_updates")
        emit("map_subscribed", {
            "status": "subscribed to all map updates",
            "timestamp": datetime.utcnow().isoformat(),
        })
    
    @socketio.on("request_task_data")
    def on_request_task_data(data):
        """Request current task data on demand."""
        task_id = data.get("task_id")
        if task_id:
            task_map = get_task_map_view(task_id)
            if task_map:
                emit("task_data", {
                    "task": task_map,
                    "timestamp": datetime.utcnow().isoformat(),
                })
    
    @socketio.on("request_map_data")
    def on_request_map_data():
        """Request all active tasks map data on demand."""
        tasks = get_all_tasks_map_view()
        emit("map_data", {
            "tasks": tasks,
            "timestamp": datetime.utcnow().isoformat(),
        })


def emit_task_robot_position_update(socketio, task_id: str, robot_x: float, robot_y: float, status: str):
    """
    Emit robot position update for a specific task.
    This updates ONLY the robot position, NOT shelf location.
    """
    map_data = get_task_map_view(task_id)
    if map_data:
        socketio.emit("robot_position_update", {
            "task_id": task_id,
            "robot": {
                "x": robot_x,
                "y": robot_y,
            },
            "status": status,
            "timestamp": datetime.utcnow().isoformat(),
        }, room=f"task_{task_id}")


def emit_task_status_change(socketio, task_id: str, old_status: str, new_status: str, 
                           current_target: str = None, robot_x: float = None, robot_y: float = None):
    """
    Emit task status change event.
    Status can be: PENDING -> ASSIGNED -> MOVING_TO_SHELF -> PICKING -> 
                   MOVING_TO_DROP -> DROPPING -> RETURNING -> COMPLETED
    """
    map_data = get_task_map_view(task_id)
    if map_data:
        socketio.emit("task_status_change", {
            "task_id": task_id,
            "old_status": old_status,
            "new_status": new_status,
            "current_target": current_target,
            "robot": {
                "x": robot_x if robot_x is not None else map_data["robot"]["x"],
                "y": robot_y if robot_y is not None else map_data["robot"]["y"],
            },
            "shelf": map_data["shelf"],  # FIXED - never changes
            "drop_zone": map_data["drop_zone"],  # FIXED - never changes
            "timestamp": datetime.utcnow().isoformat(),
        }, room=f"task_{task_id}")
        
        # Also broadcast to map subscribers
        socketio.emit("task_status_change", {
            "task_id": task_id,
            "old_status": old_status,
            "new_status": new_status,
            "current_target": current_target,
            "robot": {
                "x": robot_x if robot_x is not None else map_data["robot"]["x"],
                "y": robot_y if robot_y is not None else map_data["robot"]["y"],
            },
            "timestamp": datetime.utcnow().isoformat(),
        }, room="map_updates")


def emit_shelf_location_fixed(socketio, task_id: str, shelf_id: str, shelf_x: float, shelf_y: float):
    """
    Emit notification that shelf location is FIXED and won't change.
    This is informational for the frontend.
    """
    socketio.emit("shelf_location_fixed", {
        "task_id": task_id,
        "shelf_id": shelf_id,
        "x": shelf_x,
        "y": shelf_y,
        "note": "Shelf location is FIXED during task execution. Only robot position changes.",
        "timestamp": datetime.utcnow().isoformat(),
    }, room=f"task_{task_id}")


def emit_all_tasks_map_update(socketio):
    """Broadcast all active tasks map data to all connected clients."""
    tasks = get_all_tasks_map_view()
    socketio.emit("all_tasks_map_update", {
        "tasks": tasks,
        "timestamp": datetime.utcnow().isoformat(),
    }, room="map_updates")
