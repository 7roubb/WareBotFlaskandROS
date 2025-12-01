"""
WebSocket handlers for real-time live updates (tasks, robots, shelves)
Clients connect and receive streaming updates about system state changes
"""

from flask import current_app
from flask_socketio import emit, join_room, leave_room, disconnect
from datetime import datetime
from ..extensions import get_db


# =========================================================
# WEBSOCKET EVENTS - LIVE DASHBOARD
# =========================================================

def register_websocket_handlers(socketio):
    """Register all WebSocket event handlers for live updates"""

    @socketio.on("connect")
    def on_connect():
        """Handle client connection"""
        current_app.logger.info(f"Client connected for live updates")
        emit("connection_response", {
            "data": "Connected to live update server",
            "timestamp": datetime.utcnow().isoformat()
        })

    @socketio.on("disconnect")
    def on_disconnect():
        """Handle client disconnection"""
        current_app.logger.info("Client disconnected from live updates")

    @socketio.on("subscribe_tasks")
    def on_subscribe_tasks():
        """Subscribe to real-time task updates"""
        join_room("tasks_room")
        emit("subscription_response", {
            "channel": "tasks",
            "status": "subscribed",
            "timestamp": datetime.utcnow().isoformat()
        })
        current_app.logger.debug("Client subscribed to task updates")

    @socketio.on("subscribe_robots")
    def on_subscribe_robots():
        """Subscribe to real-time robot updates"""
        join_room("robots_room")
        emit("subscription_response", {
            "channel": "robots",
            "status": "subscribed",
            "timestamp": datetime.utcnow().isoformat()
        })
        current_app.logger.debug("Client subscribed to robot updates")

    @socketio.on("subscribe_shelves")
    def on_subscribe_shelves():
        """Subscribe to real-time shelf updates"""
        join_room("shelves_room")
        emit("subscription_response", {
            "channel": "shelves",
            "status": "subscribed",
            "timestamp": datetime.utcnow().isoformat()
        })
        current_app.logger.debug("Client subscribed to shelf updates")

    @socketio.on("subscribe_system")
    def on_subscribe_system():
        """Subscribe to overall system health updates"""
        join_room("system_room")
        emit("subscription_response", {
            "channel": "system",
            "status": "subscribed",
            "timestamp": datetime.utcnow().isoformat()
        })
        current_app.logger.debug("Client subscribed to system updates")

    @socketio.on("request_task_details")
    def on_request_task_details(data):
        """On-demand request for specific task details"""
        try:
            task_id = data.get("task_id")
            if not task_id:
                emit("error", {"message": "Missing task_id"})
                return

            from bson import ObjectId
            db = get_db()
            
            try:
                oid = ObjectId(task_id)
                task = db.tasks.find_one({"_id": oid})
            except:
                task = db.tasks.find_one({"task_id": task_id})

            if task:
                from ..services.utils_service import serialize
                emit("task_details", serialize(task))
            else:
                emit("error", {"message": "Task not found"})
        except Exception as e:
            current_app.logger.error(f"Error fetching task details: {e}")
            emit("error", {"message": str(e)})

    @socketio.on("request_robot_details")
    def on_request_robot_details(data):
        """On-demand request for specific robot details"""
        try:
            robot_id = data.get("robot_id")
            if not robot_id:
                emit("error", {"message": "Missing robot_id"})
                return

            from bson import ObjectId
            db = get_db()
            
            try:
                oid = ObjectId(robot_id)
                robot = db.robots.find_one({"_id": oid, "deleted": False})
            except:
                robot = db.robots.find_one({"robot_id": robot_id, "deleted": False})

            if robot:
                from ..services.utils_service import serialize
                emit("robot_details", serialize(robot))
            else:
                emit("error", {"message": "Robot not found"})
        except Exception as e:
            current_app.logger.error(f"Error fetching robot details: {e}")
            emit("error", {"message": str(e)})


# =========================================================
# HELPER FUNCTIONS FOR EMITTING LIVE UPDATES
# =========================================================

def emit_task_update(socketio, task_data):
    """Emit a task update to all subscribed clients"""
    try:
        socketio.emit("task_update", task_data, room="tasks_room")
    except Exception as e:
        current_app.logger.error(f"Error emitting task update: {e}")


def emit_robot_update(socketio, robot_data):
    """Emit a robot update to all subscribed clients"""
    try:
        socketio.emit("robot_update", robot_data, room="robots_room")
    except Exception as e:
        current_app.logger.error(f"Error emitting robot update: {e}")


def emit_shelf_update(socketio, shelf_data):
    """Emit a shelf location update to all subscribed clients"""
    try:
        socketio.emit("shelf_update", shelf_data, room="shelves_room")
    except Exception as e:
        current_app.logger.error(f"Error emitting shelf update: {e}")


def emit_system_update(socketio, system_data):
    """Emit a system health update to all subscribed clients"""
    try:
        socketio.emit("system_update", system_data, room="system_room")
    except Exception as e:
        current_app.logger.error(f"Error emitting system update: {e}")
