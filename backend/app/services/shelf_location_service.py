"""
CRITICAL: Shelf Location Lifecycle Management Service

This service ensures proper handling of shelf locations across task lifecycle:
- Storage location: Immutable original location (warehouse home)
- Current location: Where shelf is now (for map display)
- Location status: Semantic meaning of current location (STORED, IN_TRANSIT, AT_DROP_ZONE, etc.)

Drop zones are TEMPORARY ONLY - they must NOT overwrite storage locations.
"""

from datetime import datetime
from typing import Optional, Dict, Any, Tuple
from bson import ObjectId
from ..extensions import get_db


def initialize_shelf_storage_location(shelf_id: str, x: float, y: float, yaw: float = 0.0) -> bool:
    """
    Initialize a shelf's storage location (called at shelf creation).
    Storage location is IMMUTABLE and represents the warehouse home position.
    
    Args:
        shelf_id: Shelf ObjectId or shelf_id string
        x, y, yaw: Storage location coordinates
    
    Returns:
        True if successful
    """
    db = get_db()
    
    try:
        oid = ObjectId(shelf_id)
        result = db.shelves.update_one(
            {"_id": oid, "deleted": False},
            {
                "$set": {
                    "storage_x": float(x),
                    "storage_y": float(y),
                    "storage_yaw": float(yaw),
                    "current_x": float(x),
                    "current_y": float(y),
                    "current_yaw": float(yaw),
                    "location_status": "STORED",
                    "updated_at": datetime.utcnow()
                }
            }
        )
        return result.modified_count > 0
    except Exception as e:
        # Try by shelf_id field
        try:
            result = db.shelves.update_one(
                {"shelf_id": shelf_id, "deleted": False},
                {
                    "$set": {
                        "storage_x": float(x),
                        "storage_y": float(y),
                        "storage_yaw": float(yaw),
                        "current_x": float(x),
                        "current_y": float(y),
                        "current_yaw": float(yaw),
                        "location_status": "STORED",
                        "updated_at": datetime.utcnow()
                    }
                }
            )
            return result.modified_count > 0
        except Exception:
            return False


def update_shelf_current_location(
    shelf_id: str,
    x: float,
    y: float,
    yaw: float = 0.0,
    location_status: str = "IN_TRANSIT",
    task_id: Optional[str] = None
) -> bool:
    """
    Update a shelf's CURRENT location for map display and navigation.
    
    CRITICAL: This NEVER touches storage_x, storage_y, storage_yaw (immutable).
    This is called when robot moves shelf or delivers to drop zone.
    
    Args:
        shelf_id: Shelf ObjectId or shelf_id string
        x, y, yaw: Current position coordinates
        location_status: "IN_TRANSIT", "AT_DROP_ZONE", "STORED", etc.
        task_id: Optional task context for audit trail
    
    Returns:
        True if successful
    """
    db = get_db()
    
    update_doc = {
        "current_x": float(x),
        "current_y": float(y),
        "current_yaw": float(yaw),
        "location_status": location_status,
        "updated_at": datetime.utcnow()
    }
    
    # Include task context for audit trail
    if task_id:
        update_doc["last_task_id"] = task_id
    
    try:
        oid = ObjectId(shelf_id)
        result = db.shelves.update_one(
            {"_id": oid, "deleted": False},
            {"$set": update_doc}
        )
        
        if result.modified_count > 0:
            # Record location change in history
            _record_location_change(shelf_id, x, y, yaw, location_status, task_id)
            return True
        return False
    except Exception as e:
        # Try by shelf_id field
        try:
            result = db.shelves.update_one(
                {"shelf_id": shelf_id, "deleted": False},
                {"$set": update_doc}
            )
            
            if result.modified_count > 0:
                _record_location_change(shelf_id, x, y, yaw, location_status, task_id)
                return True
            return False
        except Exception:
            return False


def restore_shelf_to_storage_location(shelf_id: str, task_id: Optional[str] = None) -> bool:
    """
    Restore shelf to its STORAGE location (immutable original position).
    
    This is called when:
    - RETURN_SHELF task completes
    - Manual reset/restore needed
    - Error recovery
    
    Args:
        shelf_id: Shelf ObjectId or shelf_id string
        task_id: Optional task context
    
    Returns:
        True if successful
    """
    db = get_db()
    
    try:
        oid = ObjectId(shelf_id)
        shelf = db.shelves.find_one({"_id": oid, "deleted": False})
    except Exception:
        shelf = db.shelves.find_one({"shelf_id": shelf_id, "deleted": False})
    
    if not shelf:
        return False
    
    # Get storage location (should always exist)
    storage_x = shelf.get("storage_x", 0.0)
    storage_y = shelf.get("storage_y", 0.0)
    storage_yaw = shelf.get("storage_yaw", 0.0)
    
    # Update current location to storage location
    return update_shelf_current_location(
        shelf_id,
        storage_x,
        storage_y,
        storage_yaw,
        location_status="STORED",
        task_id=task_id
    )


def get_shelf_location_info(shelf_id: str) -> Optional[Dict[str, Any]]:
    """
    Get complete location information for a shelf including both storage and current.
    
    Returns:
        Dict with keys:
        - storage_x, storage_y, storage_yaw: Immutable original location
        - current_x, current_y, current_yaw: Current display location
        - location_status: Current semantic status
        - last_task_id: Last task that modified location
        Or None if shelf not found
    """
    db = get_db()
    
    try:
        oid = ObjectId(shelf_id)
        shelf = db.shelves.find_one({"_id": oid, "deleted": False})
    except Exception:
        shelf = db.shelves.find_one({"shelf_id": shelf_id, "deleted": False})
    
    if not shelf:
        return None
    
    return {
        "shelf_id": str(shelf.get("_id", shelf.get("shelf_id"))),
        "storage_x": float(shelf.get("storage_x", 0.0)),
        "storage_y": float(shelf.get("storage_y", 0.0)),
        "storage_yaw": float(shelf.get("storage_yaw", 0.0)),
        "current_x": float(shelf.get("current_x", 0.0)),
        "current_y": float(shelf.get("current_y", 0.0)),
        "current_yaw": float(shelf.get("current_yaw", 0.0)),
        "location_status": shelf.get("location_status", "STORED"),
        "last_task_id": shelf.get("last_task_id"),
        "updated_at": shelf.get("updated_at")
    }


def _record_location_change(
    shelf_id: str,
    x: float,
    y: float,
    yaw: float,
    location_status: str,
    task_id: Optional[str] = None
) -> bool:
    """
    Record shelf location change in history for audit trail.
    
    Args:
        shelf_id: Shelf identifier
        x, y, yaw: New coordinates
        location_status: Status of new location
        task_id: Optional task context
    
    Returns:
        True if recorded successfully
    """
    db = get_db()
    
    history_doc = {
        "shelf_id": str(shelf_id),
        "x": float(x),
        "y": float(y),
        "yaw": float(yaw),
        "location_status": location_status,
        "task_id": task_id,
        "timestamp": datetime.utcnow()
    }
    
    try:
        db.shelf_location_history.insert_one(history_doc)
        return True
    except Exception:
        # If collection doesn't exist yet, just skip
        return False


def get_shelf_location_history(shelf_id: str, limit: int = 50) -> list:
    """
    Get location history for a shelf for audit trail.
    
    Args:
        shelf_id: Shelf identifier
        limit: Maximum number of records to return
    
    Returns:
        List of location history documents
    """
    db = get_db()
    
    try:
        history = list(
            db.shelf_location_history
            .find({"shelf_id": str(shelf_id)})
            .sort("timestamp", -1)
            .limit(limit)
        )
        
        # Serialize datetime objects
        for doc in history:
            if "_id" in doc:
                doc["_id"] = str(doc["_id"])
            if "timestamp" in doc:
                doc["timestamp"] = doc["timestamp"].isoformat()
        
        return history
    except Exception:
        return []


def capture_shelf_location_snapshot(shelf_id: str) -> Optional[Dict[str, Any]]:
    """
    Capture a snapshot of shelf's current location at a point in time.
    Used when starting a task to preserve the pickup location.
    
    Args:
        shelf_id: Shelf identifier
    
    Returns:
        Dict with current location data or None if not found
    """
    db = get_db()
    
    try:
        oid = ObjectId(shelf_id)
        shelf = db.shelves.find_one({"_id": oid, "deleted": False})
    except Exception:
        shelf = db.shelves.find_one({"shelf_id": shelf_id, "deleted": False})
    
    if not shelf:
        return None
    
    return {
        "shelf_id": str(shelf.get("_id", shelf.get("shelf_id"))),
        "pickup_x": float(shelf.get("current_x", 0.0)),
        "pickup_y": float(shelf.get("current_y", 0.0)),
        "pickup_yaw": float(shelf.get("current_yaw", 0.0)),
        "storage_x": float(shelf.get("storage_x", 0.0)),
        "storage_y": float(shelf.get("storage_y", 0.0)),
        "storage_yaw": float(shelf.get("storage_yaw", 0.0)),
        "location_status": shelf.get("location_status", "STORED"),
        "timestamp": datetime.utcnow().isoformat()
    }
