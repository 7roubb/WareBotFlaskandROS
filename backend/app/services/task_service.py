from datetime import datetime
from typing import Optional, Dict, Any, List
from bson import ObjectId
from flask import current_app as app
from ..extensions import get_db
from .utils_service import serialize
from .shelf_location_service import (
    update_shelf_current_location,
    restore_shelf_to_storage_location,
    capture_shelf_location_snapshot,
    get_shelf_location_info
)


def _to_str_id(oid):
    """Convert ObjectId to string, or return as-is if already string."""
    try:
        return str(oid)
    except Exception:
        return oid


def _resolve_shelf_coords(shelf_id: str) -> tuple:
    """
    Dynamically resolve shelf coordinates at runtime.
    Returns (x, y, yaw) tuple.
    Raises ValueError if shelf not found.
    """
    db = get_db()
    try:
        shelf_oid = ObjectId(shelf_id)
        shelf = db.shelves.find_one({"_id": shelf_oid, "deleted": False})
    except Exception:
        shelf = db.shelves.find_one({"shelf_id": shelf_id, "deleted": False})

    if not shelf:
        raise ValueError(f"shelf_not_found: {shelf_id}")

    x = float(shelf.get("x_coord", shelf.get("x", 0.0)))
    y = float(shelf.get("y_coord", shelf.get("y", 0.0)))
    yaw = float(shelf.get("yaw", 0.0))
    return x, y, yaw


def _resolve_zone_coords(zone_id: str) -> tuple:
    """
    Dynamically resolve zone coordinates at runtime.
    Returns (x, y, yaw) tuple or (None, None, None) if not found.
    """
    if not zone_id:
        return None, None, None

    db = get_db()
    try:
        zone_oid = ObjectId(zone_id)
        zone = db.zones.find_one({"_id": zone_oid, "deleted": False})
    except Exception:
        zone = db.zones.find_one({"zone_id": zone_id, "deleted": False})

    if not zone:
        return None, None, None

    x = float(zone.get("x", 0.0))
    y = float(zone.get("y", 0.0))
    yaw = float(zone.get("yaw", 0.0))
    return x, y, yaw


def create_task_and_assign(
    shelf_id: str,
    priority: int,
    desc: Optional[str] = None,
    zone_id: Optional[str] = None,
    task_type: str = "PICKUP_AND_DELIVER",
    target_shelf_id: Optional[str] = None,
    target_zone_id: Optional[str] = None,
) -> Dict[str, Any]:
    """
    Create a task for the shelf and pick the best available robot.
    Tasks now store references (shelf_id, zone_id) and resolve coordinates dynamically.
    
    task_type options:
    - PICKUP_AND_DELIVER: Pick shelf, deliver to zone (or same location)
    - MOVE_SHELF: Move shelf from current location to target_shelf_id location
    - RETURN_SHELF: Return shelf from drop zone to home/original location
    - REPOSITION: Reposition shelf to target_zone_id
    
    Raises ValueError on invalid input or missing resources.
    """
    db = get_db()

    # Validate and resolve shelf location
    try:
        pickup_x, pickup_y, pickup_yaw = _resolve_shelf_coords(shelf_id)
    except ValueError:
        raise ValueError("shelf_not_found")

    # Find available robots
    robots = list(db.robots.find({"deleted": False, "available": True}))
    if not robots:
        raise ValueError("no_available_robots")

    # Choose best robot (distance + battery heuristic)
    best = None
    best_cost = float("inf")
    for r in robots:
        rx = r.get("x") if r.get("x") is not None else r.get("x_coord")
        ry = r.get("y") if r.get("y") is not None else r.get("y_coord")
        battery = float(r.get("battery_level", 100))
        if rx is None or ry is None:
            continue
        try:
            dist = ((float(rx) - pickup_x) ** 2 + (float(ry) - pickup_y) ** 2) ** 0.5
        except Exception:
            continue
        cost = dist * 0.7 + (100.0 - battery) * 0.3
        if cost < best_cost:
            best_cost = cost
            best = r

    if not best:
        raise ValueError("no_suitable_robot")

    now = datetime.utcnow()
    assigned_robot_name = best.get("robot_id") or best.get("name") or ""
    assigned_robot_id = _to_str_id(best.get("_id"))

    # Resolve drop location based on task_type
    drop_x, drop_y, drop_yaw = pickup_x, pickup_y, pickup_yaw  # default: drop at pickup
    drop_zone_id = zone_id

    if task_type == "PICKUP_AND_DELIVER" and zone_id:
        # Try to resolve zone coordinates
        zx, zy, zyaw = _resolve_zone_coords(zone_id)
        if zx is not None:
            drop_x, drop_y, drop_yaw = zx, zy, zyaw
    elif task_type == "MOVE_SHELF" and target_shelf_id:
        # Destination is another shelf's location
        try:
            drop_x, drop_y, drop_yaw = _resolve_shelf_coords(target_shelf_id)
        except ValueError:
            # If target shelf not found, use same location
            pass
    elif task_type == "REPOSITION" and target_zone_id:
        # Destination is a zone
        zx, zy, zyaw = _resolve_zone_coords(target_zone_id)
        if zx is not None:
            drop_x, drop_y, drop_yaw = zx, zy, zyaw
            drop_zone_id = target_zone_id

    # Build task document (stores references, not coordinates)
    doc = {
        "shelf_id": shelf_id,
        "priority": int(priority),
        "description": desc,
        "task_type": task_type,
        "assigned_robot_name": assigned_robot_name,
        "assigned_robot_id": assigned_robot_id,
        # Pickup = shelf (current coordinates stored at assignment time for reference)
        "pickup_x": pickup_x,
        "pickup_y": pickup_y,
        "pickup_yaw": pickup_yaw,
        # Legacy target_* names for backward compatibility
        "target_x": pickup_x,
        "target_y": pickup_y,
        "target_yaw": pickup_yaw,
        # Drop location (current coordinates at assignment time)
        "drop_x": drop_x,
        "drop_y": drop_y,
        "drop_yaw": drop_yaw,
        "drop_zone_id": drop_zone_id,
        # Task-specific references for future dynamic resolution
        "target_shelf_id": target_shelf_id,
        "target_zone_id": target_zone_id,
        # Status tracking
        "status": "ASSIGNED",
        "created_at": now,
        "updated_at": now,
    }

    # Insert and return serialized doc
    res = db.tasks.insert_one(doc)
    created = db.tasks.find_one({"_id": res.inserted_id})
    return serialize(created)


def resolve_task_coordinates(task_id: str) -> Dict[str, float]:
    """
    Dynamically resolve current pickup and drop coordinates for a task
    based on its referenced shelf/zone IDs.
    Returns dict with pickup_x, pickup_y, pickup_yaw, drop_x, drop_y, drop_yaw.
    Used when publishing MQTT to get latest coordinates.
    """
    db = get_db()
    try:
        task_oid = ObjectId(task_id)
        task = db.tasks.find_one({"_id": task_oid})
    except Exception:
        task = None

    if not task:
        raise ValueError(f"task_not_found: {task_id}")

    # Resolve pickup location (from shelf_id)
    try:
        px, py, pyaw = _resolve_shelf_coords(task["shelf_id"])
    except ValueError:
        # Fallback to stored values
        px = task.get("pickup_x", 0.0)
        py = task.get("pickup_y", 0.0)
        pyaw = task.get("pickup_yaw", 0.0)

    # Resolve drop location
    task_type = task.get("task_type", "PICKUP_AND_DELIVER")
    dx, dy, dyaw = px, py, pyaw  # default to pickup

    if task_type == "PICKUP_AND_DELIVER" and task.get("drop_zone_id"):
        zx, zy, zyaw = _resolve_zone_coords(task["drop_zone_id"])
        if zx is not None:
            dx, dy, dyaw = zx, zy, zyaw
    elif task_type == "MOVE_SHELF" and task.get("target_shelf_id"):
        try:
            dx, dy, dyaw = _resolve_shelf_coords(task["target_shelf_id"])
        except ValueError:
            pass
    elif task_type == "REPOSITION" and task.get("target_zone_id"):
        zx, zy, zyaw = _resolve_zone_coords(task["target_zone_id"])
        if zx is not None:
            dx, dy, dyaw = zx, zy, zyaw

    return {
        "pickup_x": px,
        "pickup_y": py,
        "pickup_yaw": pyaw,
        "drop_x": dx,
        "drop_y": dy,
        "drop_yaw": dyaw,
    }


def list_tasks() -> List[Dict[str, Any]]:
    """Return a list of serialized tasks (all)."""
    db = get_db()
    return [serialize(t) for t in db.tasks.find({})]


def update_task_status(task_id: str, status: str) -> bool:
    """
    Update a task status. Returns True if successful.
    
    CRITICAL: Handles task-type-specific logic:
    - RETURN_SHELF completion: Restore shelf to storage location
    - PICKUP_AND_DELIVER completion: Leave shelf at current (drop zone) location
    - MOVE_SHELF completion: Keep shelf at target location
    """
    db = get_db()
    try:
        oid = ObjectId(task_id)
    except Exception:
        return False

    # Fetch current task to check its type and shelf
    current_task = db.tasks.find_one({"_id": oid})
    if not current_task:
        return False
    
    task_type = current_task.get("task_type", "PICKUP_AND_DELIVER")
    shelf_id = current_task.get("shelf_id")

    upd = {"$set": {
        "status": status,
        "updated_at": datetime.utcnow()
    }}
    
    # CRITICAL: Handle completion-specific logic
    if status == "COMPLETED":
        completed_at = datetime.utcnow()
        upd["$set"]["completed_at"] = completed_at
        
        # Track duration for analytics
        created_at = current_task.get("created_at", completed_at)
        duration = (completed_at - created_at).total_seconds()
        upd["$set"]["duration_seconds"] = duration
        
        # CRITICAL: Task type-specific location handling
        if task_type == "RETURN_SHELF" and shelf_id:
            # Restore shelf to its STORAGE location (original warehouse position)
            # This is the CORE FIX for the drop zone overwrite bug
            restore_shelf_to_storage_location(shelf_id, task_id=task_id)
            app.logger.info(f"[TASK COMPLETION] RETURN_SHELF task {task_id}: Restored shelf {shelf_id} to storage location")
            upd["$set"]["completion_action"] = "RESTORED_TO_STORAGE"
            
        elif task_type == "PICKUP_AND_DELIVER" and shelf_id:
            # Shelf stays at drop zone (current location)
            # Mark it as delivered (at drop zone)
            location_info = get_shelf_location_info(shelf_id)
            if location_info:
                update_shelf_current_location(
                    shelf_id,
                    location_info["current_x"],
                    location_info["current_y"],
                    location_info["current_yaw"],
                    location_status="DELIVERED_AT_DROP_ZONE",
                    task_id=task_id
                )
            upd["$set"]["completion_action"] = "DELIVERED_TO_DROP_ZONE"
            app.logger.info(f"[TASK COMPLETION] PICKUP_AND_DELIVER task {task_id}: Shelf {shelf_id} delivered to drop zone")
            
        elif task_type == "MOVE_SHELF" and shelf_id:
            # Shelf stays at target location
            location_info = get_shelf_location_info(shelf_id)
            if location_info:
                update_shelf_current_location(
                    shelf_id,
                    location_info["current_x"],
                    location_info["current_y"],
                    location_info["current_yaw"],
                    location_status="REPOSITIONED",
                    task_id=task_id
                )
            upd["$set"]["completion_action"] = "MOVED_TO_TARGET"
            app.logger.info(f"[TASK COMPLETION] MOVE_SHELF task {task_id}: Shelf {shelf_id} moved to target location")
            
        elif task_type == "REPOSITION" and shelf_id:
            # Shelf stays at target zone
            location_info = get_shelf_location_info(shelf_id)
            if location_info:
                update_shelf_current_location(
                    shelf_id,
                    location_info["current_x"],
                    location_info["current_y"],
                    location_info["current_yaw"],
                    location_status="REPOSITIONED",
                    task_id=task_id
                )
            upd["$set"]["completion_action"] = "REPOSITIONED_AT_ZONE"
            app.logger.info(f"[TASK COMPLETION] REPOSITION task {task_id}: Shelf {shelf_id} repositioned at zone")
        
        # Mark robot as available again
        robot_id = current_task.get("assigned_robot_id")
        if robot_id:
            try:
                oid_robot = ObjectId(robot_id)
                db.robots.update_one(
                    {"_id": oid_robot},
                    {"$set": {"available": True, "current_shelf_id": None}}
                )
            except Exception:
                db.robots.update_one(
                    {"robot_id": robot_id},
                    {"$set": {"available": True, "current_shelf_id": None}}
                )
    
    # Track error states
    if status == "ERROR":
        upd["$set"]["error_at"] = datetime.utcnow()
        
        # On error, try to restore shelf to storage location for safety
        if shelf_id:
            try:
                restore_shelf_to_storage_location(shelf_id, task_id=task_id)
                app.logger.warning(f"[TASK ERROR] Task {task_id} failed - restored shelf {shelf_id} to storage for safety")
            except Exception as e:
                app.logger.error(f"[TASK ERROR] Failed to restore shelf on error: {e}")
        
        # Mark robot as available again on error
        robot_id = current_task.get("assigned_robot_id")
        if robot_id:
            try:
                oid_robot = ObjectId(robot_id)
                db.robots.update_one(
                    {"_id": oid_robot},
                    {"$set": {"available": True, "current_shelf_id": None}}
                )
            except Exception:
                db.robots.update_one(
                    {"robot_id": robot_id},
                    {"$set": {"available": True, "current_shelf_id": None}}
                )

    result = db.tasks.update_one({"_id": oid}, upd)
    return result.modified_count > 0


def record_task_state_transition(task_id: str, from_state: str, to_state: str, metadata: Optional[Dict] = None) -> bool:
    """
    Record task state transitions for audit trail and analytics.
    Helps track task lifecycle in detail.
    """
    db = get_db()
    try:
        oid = ObjectId(task_id)
    except Exception:
        return False

    transition_doc = {
        "task_id": task_id,
        "from_state": from_state,
        "to_state": to_state,
        "timestamp": datetime.utcnow(),
        "metadata": metadata or {}
    }
    
    try:
        db.task_transitions.insert_one(transition_doc)
    except Exception:
        pass  # If collection doesn't exist yet, just skip
    
    return True
