"""
Task service with proper shelf location management.
Handles task lifecycle and shelf restoration on completion.
"""

from typing import Optional, Dict, Any, List
from datetime import datetime
from bson import ObjectId
from flask import current_app as app

from ..extensions import get_db
from .utils_service import serialize


def _to_str_id(oid):
    """Convert ObjectId to string, or return as-is if already string."""
    try:
        return str(oid)
    except Exception:
        return oid


def _resolve_shelf_coords(shelf_id: str) -> tuple:
    """
    Dynamically resolve shelf CURRENT coordinates at runtime.
    Returns (x, y, yaw) tuple for pickup location (uses current_* not storage).
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

    # Use current_* fields (live location) for pickup, fallback to legacy x/x_coord
    x = float(shelf.get("current_x", shelf.get("x_coord", shelf.get("x", 0.0))))
    y = float(shelf.get("current_y", shelf.get("y_coord", shelf.get("y", 0.0))))
    yaw = float(shelf.get("current_yaw", shelf.get("yaw", 0.0)))
    return x, y, yaw


def _resolve_shelf_storage_coords(shelf_id: str) -> tuple:
    """
    Dynamically resolve shelf STORAGE (home) coordinates at runtime.
    Returns (x, y, yaw) tuple for storage/home location (immutable).
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

    # Use storage_* fields (immutable home location)
    x = float(shelf.get("storage_x", 0.0))
    y = float(shelf.get("storage_y", 0.0))
    yaw = float(shelf.get("storage_yaw", 0.0))
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
    
    Robot selection criteria:
    - Status must be IDLE (not BUSY, ERROR, or OFFLINE)
    - Must have valid position (current_x/current_y or x/y)
    - Best robot selected based on distance + battery level heuristic
    
    task_type options:
    - PICKUP_AND_DELIVER: Pick shelf, deliver to zone (or same location)
    - MOVE_SHELF: Move shelf from current location to target_shelf_id location
    - RETURN_SHELF: Return shelf from drop zone to home/original location
    - REPOSITION: Reposition shelf to target_zone_id
    
    Raises ValueError on invalid input or missing resources.
    """
    db = get_db()

    # For RETURN_SHELF tasks, pickup location is the CURRENT location, 
    # and drop location is the STORAGE location
    if task_type == "RETURN_SHELF":
        # Start from current location
        try:
            pickup_x, pickup_y, pickup_yaw = _resolve_shelf_coords(shelf_id)
        except ValueError:
            raise ValueError("shelf_not_found")
        
        # Return to storage location
        try:
            drop_x, drop_y, drop_yaw = _resolve_shelf_storage_coords(shelf_id)
        except ValueError:
            raise ValueError("shelf_not_found")
        
        drop_zone_id = None
    else:
        # For all other tasks, validate and resolve shelf location
        try:
            pickup_x, pickup_y, pickup_yaw = _resolve_shelf_coords(shelf_id)
        except ValueError:
            raise ValueError("shelf_not_found")

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

    # Find IDLE robots (not BUSY, ERROR, or OFFLINE)
    # Status must be IDLE, and must have valid position (x/y)
    robots = list(db.robots.find({"deleted": False, "status": "IDLE"}))
    if not robots:
        raise ValueError("no_available_robots")

    # Filter robots with valid positions
    valid_robots = []
    for r in robots:
        # Try current_x/current_y first (preferred), fallback to legacy x/y
        rx = r.get("current_x") if r.get("current_x") is not None else r.get("x")
        ry = r.get("current_y") if r.get("current_y") is not None else r.get("y")
        if rx is None or ry is None:
            continue
        valid_robots.append(r)
    
    if not valid_robots:
        raise ValueError("no_suitable_robot")

    # Choose best robot (distance + battery heuristic)
    best = None
    best_cost = float("inf")
    for r in valid_robots:
        # Get position (current_x/current_y preferred, fallback to x/y)
        rx = r.get("current_x") if r.get("current_x") is not None else r.get("x")
        ry = r.get("current_y") if r.get("current_y") is not None else r.get("y")
        battery = float(r.get("battery_level", 100))
        
        try:
            dist = ((float(rx) - pickup_x) ** 2 + (float(ry) - pickup_y) ** 2) ** 0.5
        except Exception:
            continue
        
        # Cost function: distance (70%) + battery depletion (30%)
        cost = dist * 0.7 + (100.0 - battery) * 0.3
        if cost < best_cost:
            best_cost = cost
            best = r

    if not best:
        raise ValueError("no_suitable_robot")

    now = datetime.utcnow()
    assigned_robot_name = best.get("robot_id") or best.get("name") or ""
    assigned_robot_id = _to_str_id(best.get("_id"))
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

    # Capture and persist storage snapshot (immutable) for this task
    try:
        from .shelf_location_service import capture_shelf_location_snapshot
        snapshot = capture_shelf_location_snapshot(shelf_id)
        if snapshot:
            doc["origin_storage_x"] = snapshot.get("storage_x")
            doc["origin_storage_y"] = snapshot.get("storage_y")
            doc["origin_storage_yaw"] = snapshot.get("storage_yaw")
            # Also capture pickup (current) location at assignment time
            doc["origin_pickup_x"] = snapshot.get("pickup_x")
            doc["origin_pickup_y"] = snapshot.get("pickup_y")
            doc["origin_pickup_yaw"] = snapshot.get("pickup_yaw")
    except Exception as e:
        app.logger.warning(f"[TASK] Failed to capture shelf snapshot: {e}")
        # never fail task creation because of snapshot

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
    
    CRITICAL: For RETURN_SHELF tasks:
    - pickup: Current shelf location
    - drop: Storage (home) location
    """
    db = get_db()
    try:
        task_oid = ObjectId(task_id)
        task = db.tasks.find_one({"_id": task_oid})
    except Exception:
        task = None

    if not task:
        raise ValueError(f"task_not_found: {task_id}")

    task_type = task.get("task_type", "PICKUP_AND_DELIVER")
    
    # Resolve pickup location (from shelf_id) - always current location
    try:
        px, py, pyaw = _resolve_shelf_coords(task["shelf_id"])
    except ValueError:
        # Fallback to stored values
        px = task.get("pickup_x", 0.0)
        py = task.get("pickup_y", 0.0)
        pyaw = task.get("pickup_yaw", 0.0)

    # Resolve drop location based on task type
    dx, dy, dyaw = px, py, pyaw  # default to pickup

    if task_type == "RETURN_SHELF":
        # Return shelf to STORAGE location
        try:
            dx, dy, dyaw = _resolve_shelf_storage_coords(task["shelf_id"])
        except ValueError:
            # Fallback to stored drop coordinates
            dx = task.get("drop_x", px)
            dy = task.get("drop_y", py)
            dyaw = task.get("drop_yaw", pyaw)
    elif task_type == "PICKUP_AND_DELIVER" and task.get("drop_zone_id"):
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
    Update task status with proper task-type-specific shelf location handling.
    
    CRITICAL: Handles task-type-specific logic:
    - RETURN_SHELF completion: Restore shelf to STORAGE location
    - PICKUP_AND_DELIVER completion: Shelf stays at DROP ZONE
    - MOVE_SHELF completion: Shelf stays at TARGET location
    - REPOSITION completion: Shelf stays at TARGET ZONE
    
    Returns: bool - True if update succeeded, False otherwise
    """
    db = get_db()
    try:
        oid = ObjectId(task_id)
    except Exception:
        app.logger.error(f"[Task] Invalid task ID: {task_id}")
        return False

    # Fetch current task to check its type and shelf
    current_task = db.tasks.find_one({"_id": oid})
    if not current_task:
        app.logger.error(f"[Task] Task not found: {task_id}")
        return False
    
    task_type = current_task.get("task_type", "PICKUP_AND_DELIVER")
    shelf_id = current_task.get("shelf_id")

    upd = {"$set": {
        "status": status,
        "updated_at": datetime.utcnow()
    }}
    
    # =========================================================
    # CRITICAL: Handle robot-shelf attachment
    # =========================================================
    if status == "ATTACHED" and shelf_id:
        robot_id = current_task.get("assigned_robot_id")
        if robot_id:
            try:
                try:
                    oid_robot = ObjectId(robot_id)
                    db.robots.update_one(
                        {"_id": oid_robot},
                        {"$set": {"current_shelf_id": shelf_id}}
                    )
                except Exception:
                    db.robots.update_one(
                        {"robot_id": robot_id},
                        {"$set": {"current_shelf_id": shelf_id}}
                    )
                app.logger.info(f"[TASK UPDATE] Linked shelf {shelf_id} to robot {robot_id} (ATTACHED)")
            except Exception as e:
                app.logger.error(f"[TASK UPDATE] Failed to link shelf to robot: {str(e)}")

    # =========================================================
    # CRITICAL: Handle completion-specific logic
    # =========================================================
    if status == "COMPLETED":
        completed_at = datetime.utcnow()
        upd["$set"]["completed_at"] = completed_at
        
        # Track duration for analytics
        created_at = current_task.get("created_at", completed_at)
        duration = (completed_at - created_at).total_seconds()
        upd["$set"]["duration_seconds"] = duration
        
        # =========================================================
        # TASK-TYPE-SPECIFIC SHELF LOCATION HANDLING
        # =========================================================
        
        if task_type == "RETURN_SHELF" and shelf_id:
            """
            RETURN_SHELF: Restore shelf from drop zone back to STORAGE location.
            """
            try:
                from .shelf_location_service import (
                    restore_shelf_to_storage_location,
                    get_shelf_location_info
                )
                
                success = restore_shelf_to_storage_location(shelf_id, task_id=task_id)
                
                if success:
                    location_info = get_shelf_location_info(shelf_id)
                    app.logger.info(
                        f"[TASK COMPLETION] RETURN_SHELF task {task_id}: "
                        f"Restored shelf {shelf_id} to storage location "
                        f"({location_info.get('storage_x')}, {location_info.get('storage_y')})"
                    )
                    upd["$set"]["completion_action"] = "RESTORED_TO_STORAGE"
                    upd["$set"]["restoration_confirmed"] = True
                else:
                    app.logger.warning(
                        f"[TASK COMPLETION] RETURN_SHELF task {task_id}: "
                        f"Shelf restoration returned False for {shelf_id}"
                    )
                    upd["$set"]["completion_action"] = "RESTORATION_FAILED"
                    upd["$set"]["restoration_confirmed"] = False
                    
            except Exception as e:
                app.logger.error(
                    f"[TASK COMPLETION] RETURN_SHELF task {task_id}: "
                    f"Failed to restore shelf {shelf_id}: {str(e)}",
                    exc_info=True
                )
                upd["$set"]["completion_action"] = "RESTORATION_ERROR"
                upd["$set"]["restoration_confirmed"] = False
                upd["$set"]["restoration_error"] = str(e)
        
        elif task_type == "PICKUP_AND_DELIVER" and shelf_id:
            """
            PICKUP_AND_DELIVER: Shelf stays at drop zone.
            Update shelf current location to the drop zone coordinates.
            CRITICAL: Use robot's ACTUAL position if available to prevent snapping.
            """
            try:
                from .shelf_location_service import update_shelf_current_location
                
                # Default to theoretical drop coordinates
                final_x = float(current_task.get("drop_x", 0.0))
                final_y = float(current_task.get("drop_y", 0.0))
                final_yaw = float(current_task.get("drop_yaw", 0.0))
                
                # Try to get robot's actual position
                robot_id = current_task.get("assigned_robot_id")
                if robot_id:
                    try:
                        oid_robot = ObjectId(robot_id)
                        robot = db.robots.find_one({"_id": oid_robot})
                    except Exception:
                        robot = db.robots.find_one({"robot_id": robot_id})
                    
                    if robot and robot.get("current_x") is not None:
                        final_x = float(robot.get("current_x"))
                        final_y = float(robot.get("current_y"))
                        final_yaw = float(robot.get("current_yaw", 0.0))
                        app.logger.info(f"[TASK COMPLETION] Using robot position for shelf drop: ({final_x}, {final_y})")

                update_shelf_current_location(
                    shelf_id,
                    final_x,
                    final_y,
                    final_yaw,
                    location_status="DELIVERED_AT_DROP_ZONE",
                    task_id=task_id
                )
                app.logger.info(
                    f"[TASK COMPLETION] PICKUP_AND_DELIVER task {task_id}: "
                    f"Shelf {shelf_id} delivered at drop zone ({final_x}, {final_y})"
                )
                upd["$set"]["completion_action"] = "DELIVERED_TO_DROP_ZONE"
            except Exception as e:
                app.logger.error(
                    f"[TASK COMPLETION] PICKUP_AND_DELIVER task {task_id}: "
                    f"Failed to update shelf location: {str(e)}",
                    exc_info=True
                )
                upd["$set"]["completion_action"] = "DELIVERY_LOCATION_ERROR"
        
        elif task_type == "MOVE_SHELF" and shelf_id:
            """
            MOVE_SHELF: Shelf stays at target location.
            Update shelf current location.
            CRITICAL: Use robot's ACTUAL position if available to prevent snapping.
            """
            try:
                from .shelf_location_service import update_shelf_current_location
                
                # Default logic (theoretical target)
                final_x = 0.0
                final_y = 0.0
                final_yaw = 0.0
                
                # 1. Try robot position first (preferred)
                robot_pos_found = False
                robot_id = current_task.get("assigned_robot_id")
                if robot_id:
                    try:
                        oid_robot = ObjectId(robot_id)
                        robot = db.robots.find_one({"_id": oid_robot})
                    except Exception:
                        robot = db.robots.find_one({"robot_id": robot_id})
                    
                    if robot and robot.get("current_x") is not None:
                        final_x = float(robot.get("current_x"))
                        final_y = float(robot.get("current_y"))
                        final_yaw = float(robot.get("current_yaw", 0.0))
                        robot_pos_found = True
                        app.logger.info(f"[TASK COMPLETION] Using robot position for move completion: ({final_x}, {final_y})")

                # 2. Fallback to target shelf location if robot pos not found
                if not robot_pos_found:
                    target_shelf_id = current_task.get("target_shelf_id")
                    if target_shelf_id:
                        from .shelf_service import get_shelf
                        target_shelf = get_shelf(target_shelf_id)
                        if target_shelf:
                            final_x = target_shelf.get("current_x", 0.0)
                            final_y = target_shelf.get("current_y", 0.0)
                            final_yaw = target_shelf.get("current_yaw", 0.0)
                
                update_shelf_current_location(
                    shelf_id,
                    final_x,
                    final_y,
                    final_yaw,
                    location_status="REPOSITIONED",
                    task_id=task_id
                )
                app.logger.info(
                    f"[TASK COMPLETION] MOVE_SHELF task {task_id}: "
                    f"Shelf {shelf_id} moved to target location ({final_x}, {final_y})"
                )
                upd["$set"]["completion_action"] = "MOVED_TO_TARGET"
            except Exception as e:
                app.logger.error(
                    f"[TASK COMPLETION] MOVE_SHELF task {task_id}: "
                    f"Failed to update shelf location: {str(e)}",
                    exc_info=True
                )
                upd["$set"]["completion_action"] = "MOVE_LOCATION_ERROR"
        
        elif task_type == "REPOSITION" and shelf_id:
            """
            REPOSITION: Shelf stays at target zone.
            Update shelf current location.
            CRITICAL: Use robot's ACTUAL position if available to prevent snapping.
            """
            try:
                from .shelf_location_service import update_shelf_current_location
                
                # Default to theoretical drop coordinates
                final_x = float(current_task.get("drop_x", 0.0))
                final_y = float(current_task.get("drop_y", 0.0))
                final_yaw = float(current_task.get("drop_yaw", 0.0))
                
                # Try to get robot's actual position
                robot_id = current_task.get("assigned_robot_id")
                if robot_id:
                    try:
                        oid_robot = ObjectId(robot_id)
                        robot = db.robots.find_one({"_id": oid_robot})
                    except Exception:
                        robot = db.robots.find_one({"robot_id": robot_id})
                    
                    if robot and robot.get("current_x") is not None:
                        final_x = float(robot.get("current_x"))
                        final_y = float(robot.get("current_y"))
                        final_yaw = float(robot.get("current_yaw", 0.0))
                        app.logger.info(f"[TASK COMPLETION] Using robot position for reposition: ({final_x}, {final_y})")
                
                update_shelf_current_location(
                    shelf_id,
                    final_x,
                    final_y,
                    final_yaw,
                    location_status="REPOSITIONED_AT_ZONE",
                    task_id=task_id
                )
                app.logger.info(
                    f"[TASK COMPLETION] REPOSITION task {task_id}: "
                    f"Shelf {shelf_id} repositioned at zone ({final_x}, {final_y})"
                )
                upd["$set"]["completion_action"] = "REPOSITIONED_AT_ZONE"
            except Exception as e:
                app.logger.error(
                    f"[TASK COMPLETION] REPOSITION task {task_id}: "
                    f"Failed to update shelf location: {str(e)}",
                    exc_info=True
                )
                upd["$set"]["completion_action"] = "REPOSITION_ERROR"
        
        # Release robot back to IDLE status
        robot_id = current_task.get("assigned_robot_id")
        if robot_id:
            try:
                try:
                    oid_robot = ObjectId(robot_id)
                    result = db.robots.update_one(
                        {"_id": oid_robot},
                        {"$set": {"status": "IDLE", "current_shelf_id": None}}
                    )
                except Exception:
                    result = db.robots.update_one(
                        {"robot_id": robot_id},
                        {"$set": {"status": "IDLE", "current_shelf_id": None}}
                    )
                
                if result.modified_count > 0:
                    app.logger.info(f"[TASK COMPLETION] Released robot {robot_id} to IDLE status")
            except Exception as e:
                app.logger.error(f"[TASK COMPLETION] Failed to release robot: {str(e)}")

    # Handle drop events
    elif status in ("DROPPING", "DROPPED") and shelf_id:
        try:
            from .shelf_location_service import update_shelf_current_location
            
            dx = float(current_task.get("drop_x", current_task.get("pickup_x", 0.0)))
            dy = float(current_task.get("drop_y", current_task.get("pickup_y", 0.0)))
            dyaw = float(current_task.get("drop_yaw", current_task.get("pickup_yaw", 0.0)))
            
            update_shelf_current_location(
                shelf_id,
                dx,
                dy,
                dyaw,
                location_status="AT_DROP_ZONE",
                task_id=task_id
            )
            app.logger.info(
                f"[TASK DROP] Task {task_id}: "
                f"Shelf {shelf_id} placed at drop zone ({dx}, {dy})"
            )
        except Exception as e:
            app.logger.error(
                f"[TASK DROP] Failed to record drop location for shelf {shelf_id}: {str(e)}",
                exc_info=True
            )

    # Handle cancellation/abort
    elif status in ("CANCELLED", "ABORTED") and shelf_id:
        try:
            from .shelf_location_service import restore_shelf_to_storage_location
            
            success = restore_shelf_to_storage_location(shelf_id, task_id=task_id)
            if success:
                app.logger.warning(
                    f"[TASK CANCEL] Task {task_id}: "
                    f"Restored shelf {shelf_id} to storage due to cancellation"
                )
            else:
                app.logger.error(
                    f"[TASK CANCEL] Task {task_id}: "
                    f"Failed to restore shelf {shelf_id} on cancellation"
                )
        except Exception as e:
            app.logger.error(
                f"[TASK CANCEL] Failed to restore shelf on cancel: {str(e)}",
                exc_info=True
            )

    # Handle error states
    if status == "ERROR":
        upd["$set"]["error_at"] = datetime.utcnow()
        
        if shelf_id:
            try:
                from .shelf_location_service import restore_shelf_to_storage_location
                
                success = restore_shelf_to_storage_location(shelf_id, task_id=task_id)
                if success:
                    app.logger.warning(
                        f"[TASK ERROR] Task {task_id} failed - "
                        f"restored shelf {shelf_id} to storage for safety"
                    )
                else:
                    app.logger.error(
                        f"[TASK ERROR] Task {task_id} failed and "
                        f"shelf restoration also failed for {shelf_id}"
                    )
            except Exception as e:
                app.logger.error(
                    f"[TASK ERROR] Failed to restore shelf on error: {str(e)}",
                    exc_info=True
                )
        
        # Release robot on error (set status to IDLE)
        robot_id = current_task.get("assigned_robot_id")
        if robot_id:
            try:
                try:
                    oid_robot = ObjectId(robot_id)
                    db.robots.update_one(
                        {"_id": oid_robot},
                        {"$set": {"status": "IDLE", "current_shelf_id": None}}
                    )
                except Exception:
                    db.robots.update_one(
                        {"robot_id": robot_id},
                        {"$set": {"status": "IDLE", "current_shelf_id": None}}
                    )
                app.logger.info(f"[TASK ERROR] Released robot {robot_id} to IDLE status on error")
            except Exception as e:
                app.logger.error(f"[TASK ERROR] Failed to release robot: {str(e)}")

    # Perform the update
    try:
        result = db.tasks.update_one({"_id": oid}, upd)
        if result.modified_count > 0:
            app.logger.info(
                f"[TASK UPDATE] Task {task_id} status changed to {status}"
            )
            return True
        else:
            app.logger.warning(f"[TASK UPDATE] Task {task_id} update returned 0 modified")
            return False
    except Exception as e:
        app.logger.error(
            f"[TASK UPDATE] Failed to update task {task_id}: {str(e)}",
            exc_info=True
        )
        return False


def record_task_state_transition(task_id: str, from_state: str, to_state: str, metadata: Optional[Dict] = None) -> bool:
    """
    Record task state transitions for audit trail and analytics.
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
        pass
    
    return True


def _build_shelf_info(task: dict) -> Dict[str, Any]:
    """
    Helper: extract shelf storage/current info from task or shelf_location_service.
    """
    shelf_info = None
    try:
        from .shelf_location_service import get_shelf_location_info
        shelf_info = get_shelf_location_info(task.get("shelf_id"))
    except Exception:
        shelf_info = None

    if shelf_info:
        return {
            "storage": {
                "x": float(shelf_info.get("storage_x", 0)),
                "y": float(shelf_info.get("storage_y", 0)),
                "yaw": float(shelf_info.get("storage_yaw", 0)),
            },
            "current": {
                "x": float(shelf_info.get("current_x", 0)),
                "y": float(shelf_info.get("current_y", 0)),
                "yaw": float(shelf_info.get("current_yaw", 0)),
            }
        }
    else:
        # Fallback to task-stored snapshots
        return {
            "storage": {
                "x": float(task.get("origin_storage_x", task.get("pickup_x", 0))),
                "y": float(task.get("origin_storage_y", task.get("pickup_y", 0))),
                "yaw": float(task.get("origin_storage_yaw", task.get("pickup_yaw", 0))),
            },
            "current": {
                "x": float(task.get("current_robot_x", task.get("pickup_x", 0))),
                "y": float(task.get("current_robot_y", task.get("pickup_y", 0))),
                "yaw": float(task.get("pickup_yaw", 0)),
            }
        }


# =========================================================
# REAL-TIME TASK TRACKING FOR MAP UPDATES
# =========================================================

def update_task_with_robot_position(task_id: str, robot_x: float, robot_y: float, status: str):
    """
    Update task with robot's current position and status.
    Shelf location is FIXED and NEVER changes during task execution.
    """
    db = get_db()
    now = datetime.utcnow()
    
    try:
        oid = ObjectId(task_id)
    except Exception:
        return False
    
    task = db.tasks.find_one({"_id": oid})
    if not task:
        return False
    
    update_doc = {
        "current_robot_x": robot_x,
        "current_robot_y": robot_y,
        "status": status,
        "last_position_update": now,
    }
    
    position_entry = {
        "timestamp": now,
        "robot_x": robot_x,
        "robot_y": robot_y,
        "status": status,
    }
    
    result = db.tasks.update_one(
        {"_id": oid},
        {
            "$set": update_doc,
            "$push": {"position_history": position_entry}
        }
    )
    
    return result.modified_count > 0


def get_task_map_view(task_id: str):
    """
    Get task data for real-time map display.
    Shelf location is FIXED. Only robot position changes.
    """
    db = get_db()
    
    try:
        oid = ObjectId(task_id)
    except Exception:
        return None
    
    task = db.tasks.find_one({"_id": oid})
    if not task:
        return None
    
    shelf_info_dict = _build_shelf_info(task)

    return {
        "task_id": str(task.get("_id")),
        "robot_id": task.get("robot_id"),
        "status": task.get("status"),
        "type": task.get("task_type", task.get("type")),

        "robot": {
            "x": float(task.get("current_robot_x", 0)),
            "y": float(task.get("current_robot_y", 0)),
        },

        "shelf": {
            "id": task.get("shelf_id"),
            "storage": shelf_info_dict["storage"],
            "current": shelf_info_dict["current"],
        },

        "drop_zone": {
            "id": task.get("drop_zone_id", task.get("zone_id")),
            "x": float(task.get("drop_x", task.get("zone_x", 0))),
            "y": float(task.get("drop_y", task.get("zone_y", 0))),
        },

        "phase": task.get("phase"),
        "current_target": task.get("current_target"),

        "created_at": task.get("created_at"),
        "started_at": task.get("started_at"),
        "last_updated": task.get("last_position_update"),
    }


def get_all_tasks_map_view():
    """Get all active tasks formatted for map display."""
    db = get_db()
    tasks = list(db.tasks.find({"status": {"$in": ["ACTIVE", "PENDING", "IN_PROGRESS", "ASSIGNED", "MOVING_TO_SHELF", "MOVING_TO_DROP"]}}))

    map_view = []
    for task in tasks:
        shelf_info_dict = _build_shelf_info(task)

        map_view.append({
            "task_id": str(task.get("_id")),
            "robot_id": task.get("robot_id"),
            "status": task.get("status"),
            "type": task.get("task_type", task.get("type")),
            "robot": {
                "x": float(task.get("current_robot_x", 0)),
                "y": float(task.get("current_robot_y", 0)),
            },
            "shelf": {
                "id": task.get("shelf_id"),
                "storage": shelf_info_dict["storage"],
                "current": shelf_info_dict["current"],
            },
            "drop_zone": {
                "id": task.get("drop_zone_id", task.get("zone_id")),
                "x": float(task.get("drop_x", task.get("zone_x", 0))),
                "y": float(task.get("drop_y", task.get("zone_y", 0))),
            },
            "phase": task.get("phase"),
            "current_target": task.get("current_target"),
        })

    return map_view


def emit_task_update_websocket(socketio, task_id: str):
    """Emit task update to WebSocket clients for live map updates."""
    try:
        map_data = get_task_map_view(task_id)
        if map_data:
            socketio.emit("task_update", {
                "task": map_data,
                "timestamp": datetime.utcnow().isoformat(),
            }, broadcast=True)
    except Exception as e:
        app.logger.error(f"Failed to emit task update: {e}")


def emit_all_tasks_update_websocket(socketio):
    """Emit all tasks update to WebSocket clients for live map."""
    try:
        map_data = get_all_tasks_map_view()
        socketio.emit("tasks_update", {
            "tasks": map_data,
            "timestamp": datetime.utcnow().isoformat(),
        }, broadcast=True)
    except Exception as e:
        app.logger.error(f"Failed to emit all tasks update: {e}")