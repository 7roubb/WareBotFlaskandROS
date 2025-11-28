"""
Socket.IO event emitters for real-time updates
"""
from flask import current_app
from typing import Dict, Any


def get_socketio():
    """Get Socket.IO instance"""
    try:
        return current_app.extensions.get("socketio")
    except:
        return None


def emit_robot_telemetry(data: Dict[str, Any]) -> None:
    """
    Emit robot telemetry update
    Event: 'telemetry'
    """
    try:
        socketio = get_socketio()
        if not socketio:
            return
        
        current_app.logger.debug(f"[WS EMIT] telemetry: {data}")
        socketio.emit("telemetry", data)
    except Exception as e:
        current_app.logger.error(f"[WS EMIT ERROR] telemetry: {e}")


def emit_map_update(data: Dict[str, Any]) -> None:
    """
    Emit map update
    Event: 'map_update'
    """
    try:
        socketio = get_socketio()
        if not socketio:
            return
        
        current_app.logger.debug(f"[WS EMIT] map_update received")
        socketio.emit("map_update", data)
    except Exception as e:
        current_app.logger.error(f"[WS EMIT ERROR] map_update: {e}")


def emit_task_status(data: Dict[str, Any]) -> None:
    """
    Emit task status update
    Event: 'task_status'
    """
    try:
        socketio = get_socketio()
        if not socketio:
            return
        
        current_app.logger.debug(f"[WS EMIT] task_status: {data}")
        socketio.emit("task_status", data)
    except Exception as e:
        current_app.logger.error(f"[WS EMIT ERROR] task_status: {e}")


def emit_robot_status(data: Dict[str, Any]) -> None:
    """
    Emit robot status change
    Event: 'robot_status'
    """
    try:
        socketio = get_socketio()
        if not socketio:
            return
        
        current_app.logger.debug(f"[WS EMIT] robot_status: {data}")
        socketio.emit("robot_status", data)
    except Exception as e:
        current_app.logger.error(f"[WS EMIT ERROR] robot_status: {e}")


def emit_error(message: str) -> None:
    """
    Emit error event
    Event: 'error'
    """
    try:
        socketio = get_socketio()
        if not socketio:
            return
        
        socketio.emit("error", {"message": message})
    except Exception as e:
        current_app.logger.error(f"[WS EMIT ERROR] error: {e}")
