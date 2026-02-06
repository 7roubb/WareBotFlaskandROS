from flask import Blueprint, request, jsonify, current_app
from datetime import datetime
from flask_jwt_extended import jwt_required, get_jwt
from pydantic import ValidationError
import json
from bson import ObjectId

from ..models import TaskCreate
from ..services.task_service import (
    create_task_and_assign,
    resolve_task_coordinates,
    list_tasks,
)
from ..services.product_service import get_product
from ..extensions import get_db

tasks_bp = Blueprint("tasks", __name__)


# ---------------------------------------------------------
# Helpers
# ---------------------------------------------------------
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
    return jsonify({"error": "validation_error", "details": err.errors()}), 400


# ---------------------------------------------------------
# List tasks
# ---------------------------------------------------------
@tasks_bp.route("", methods=["GET"])
def list_tasks_route():
    return jsonify(list_tasks()), 200


# ---------------------------------------------------------
# Get task by ID with state history
# ---------------------------------------------------------
@tasks_bp.route("/<id>", methods=["GET"])
def get_task_route(id):
    db = get_db()
    try:
        oid = ObjectId(id)
    except Exception:
        return jsonify({"error": "invalid_id"}), 400

    task = db.tasks.find_one({"_id": oid})
    if not task:
        return jsonify({"error": "not_found"}), 404

    # Include state transition history
    try:
        transitions = list(db.task_transitions.find({"task_id": id}).sort("timestamp", 1))
        task["state_history"] = [
            {
                "from": t.get("from_state"),
                "to": t.get("to_state"),
                "timestamp": t.get("timestamp"),
                "metadata": t.get("metadata", {})
            }
            for t in transitions
        ]
    except Exception:
        task["state_history"] = []

    # Serialize ObjectIds
    from ..services.utils_service import serialize
    return jsonify(serialize(task)), 200


# ---------------------------------------------------------
# Create + Assign Task
# ---------------------------------------------------------
@tasks_bp.route("/assign", methods=["POST"])
@admin_required
def create_task_route():
    try:
        data = TaskCreate(**request.json).dict()
    except ValidationError as e:
        return handle_validation_error(e)

    try:
        task = create_task_and_assign(
            shelf_id=data["shelf_id"],
            priority=data["priority"],
            desc=data.get("description"),
            zone_id=data.get("zone_id"),
            task_type=data.get("task_type", "PICKUP_AND_DELIVER"),
            target_shelf_id=data.get("target_shelf_id"),
            target_zone_id=data.get("target_zone_id"),
        )
    except ValueError as e:
        # map known errors to HTTP statuses
        msg = str(e)
        if msg == "shelf_not_found":
            return jsonify({"error": "shelf_not_found"}), 400
        if msg == "no_available_robots":
            return jsonify({"error": "no_available_robots"}), 409
        if msg == "no_suitable_robot":
            return jsonify({"error": "no_suitable_robot"}), 409
        return jsonify({"error": "invalid_request", "details": msg}), 400
    except Exception:
        current_app.logger.exception("Unexpected error creating task")
        return jsonify({"error": "internal_error"}), 500

    # dispatch mqtt
    _dispatch_task_mqtt(task)

    # return created task
    return jsonify(task), 201


# ---------------------------------------------------------
# Create Task by Product (Shelf Restoration)
# ---------------------------------------------------------
@tasks_bp.route("/restore-shelf-by-product", methods=["POST"])
@admin_required
def restore_shelf_by_product_route():
    data = request.json
    if not data or "product_id" not in data or "target_zone_id" not in data:
        return jsonify({"error": "validation_error", "details": "product_id and target_zone_id are required"}), 400

    product_id = data["product_id"]
    target_zone_id = data["target_zone_id"]
    priority = data.get("priority", 2) # Default higher priority for manual restoration

    # 1. Find Product
    product = get_product(product_id)
    if not product:
        return jsonify({"error": "product_not_found"}), 404

    # 2. Get Shelf ID
    shelf_id = product.get("shelf_id")
    if not shelf_id:
        return jsonify({"error": "product_not_on_shelf", "message": "Product is not assigned to any shelf."}), 400

    # 3. Create Task
    try:
        task = create_task_and_assign(
            shelf_id=shelf_id,
            priority=priority,
            desc=f"Restore shelf for product: {product.get('name', 'Unknown')}",
            zone_id=target_zone_id,
            task_type="PICKUP_AND_DELIVER"
        )
    except ValueError as e:
        msg = str(e)
        if msg == "shelf_not_found":
            return jsonify({"error": "shelf_not_found"}), 400
        if msg == "no_available_robots":
            return jsonify({"error": "no_available_robots"}), 409
        if msg == "no_suitable_robot":
            return jsonify({"error": "no_suitable_robot"}), 409
        return jsonify({"error": "invalid_request", "details": msg}), 400
    except Exception:
        current_app.logger.exception("Unexpected error creating restoration task")
        return jsonify({"error": "internal_error"}), 500
        
    # See create_task_route for MQTT logic - we should probably refactor that into a service method
    # For now, to ensure the task is dispatched, we need to replicate the MQTT dispatch or trust the scheduler?
    # The existing create_task_route creates the task AND dispatches MQTT.
    # create_task_and_assign (service) determines assignment and saves to DB. 
    # It DOES NOT seem to publish MQTT in the service layer based on `create_task_route` code above (lines 118-262).
    # So we MUST replicate the MQTT dispatch logic here or refactor.
    
    # Let's encapsulate the MQTT dispatch into a helper function inside this file to avoid code duplication 
    # and use it in both routes.
    
    _dispatch_task_mqtt(task)

    return jsonify(task), 201

"""
FIXED _dispatch_task_mqtt() function for tasks.py

Replace the existing _dispatch_task_mqtt() function (lines ~150-262) with this version.
This simplifies robot ID resolution and ensures compatibility with task runner.
"""

def _dispatch_task_mqtt(task):
    """
    Dispatch task to robot via MQTT - FIXED VERSION
    
    Key improvements:
    1. Simplified robot ID resolution (no underscore variants)
    2. Better error handling and logging
    3. Tracks MQTT delivery status in database
    4. Uses exact robot_id match for ROS2 namespace compatibility
    """
    if not task:
        current_app.logger.warning("[MQTT] Cannot dispatch: task is None")
        return
    
    db = get_db()
    mqtt_client = current_app.mqtt
    
    if not mqtt_client:
        current_app.logger.error("[MQTT] MQTT client not available")
        return
    
    # =========================================================
    # STEP 1: Get robot_id (must match ROS2 namespace exactly)
    # =========================================================
    robot_id = None
    
    # Option A: Use assigned_robot_name (direct match to ROS2 namespace)
    if task.get("assigned_robot_name"):
        robot_id = task["assigned_robot_name"]
        current_app.logger.debug(f"[MQTT] Using assigned_robot_name: {robot_id}")
    
    # Option B: Lookup by MongoDB _id
    elif task.get("assigned_robot_id"):
        try:
            robot_doc = db.robots.find_one({
                "_id": ObjectId(task["assigned_robot_id"]),
                "deleted": False
            })
            if robot_doc:
                robot_id = robot_doc.get("robot_id")
                current_app.logger.debug(f"[MQTT] Resolved robot_id from DB: {robot_id}")
            else:
                current_app.logger.error(
                    f"[MQTT] Robot with _id={task['assigned_robot_id']} not found"
                )
        except Exception as e:
            current_app.logger.error(f"[MQTT] Failed to lookup robot: {e}")
    
    if not robot_id:
        error_msg = f"Cannot dispatch task {task.get('id')}: no robot_id found"
        current_app.logger.error(f"[MQTT] {error_msg}")
        
        # Update task with error
        if task.get("_id"):
            db.tasks.update_one(
                {"_id": task["_id"]},
                {"$set": {
                    "mqtt_delivery_status": "FAILED",
                    "mqtt_delivery_errors": [error_msg],
                    "mqtt_delivery_attempted_at": datetime.utcnow()
                }}
            )
        return
    
    # Validate robot_id format (should be robot0, robot1, etc.)
    if not robot_id.startswith("robot"):
        current_app.logger.warning(
            f"[MQTT] Unusual robot_id format: {robot_id} "
            f"(expected robot0, robot1, etc.)"
        )
    
    # =========================================================
    # STEP 2: Resolve task coordinates
    # =========================================================
    try:
        task_id_str = str(task["_id"]) if "_id" in task else task.get("id")
        coords = resolve_task_coordinates(task_id_str)
        current_app.logger.debug(f"[MQTT] Resolved coordinates for task {task_id_str}")
    except ValueError as e:
        current_app.logger.warning(
            f"[MQTT] Could not resolve coordinates via service, using task fields: {e}"
        )
        coords = {
            "pickup_x": task.get("pickup_x") or task.get("origin_pickup_x") or 0.0,
            "pickup_y": task.get("pickup_y") or task.get("origin_pickup_y") or 0.0,
            "pickup_yaw": task.get("pickup_yaw") or task.get("origin_pickup_yaw") or 0.0,
            "drop_x": task.get("drop_x") or task.get("zone_x") or 0.0,
            "drop_y": task.get("drop_y") or task.get("zone_y") or 0.0,
            "drop_yaw": task.get("drop_yaw") or 0.0,
        }
    except Exception as e:
        current_app.logger.error(f"[MQTT] Unexpected error resolving coordinates: {e}")
        coords = {
            "pickup_x": 0.0, "pickup_y": 0.0, "pickup_yaw": 0.0,
            "drop_x": 0.0, "drop_y": 0.0, "drop_yaw": 0.0,
        }
    
    # =========================================================
    # STEP 3: Build MQTT payload
    # =========================================================
    payload = {
        # Task identification
        "task_id": task.get("id"),
        "task_type": task.get("task_type", "PICKUP_AND_DELIVER"),
        "shelf_id": task.get("shelf_id"),
        "priority": task.get("priority", 1),
        
        # Pickup coordinates
        "pickup_x": float(coords["pickup_x"]),
        "pickup_y": float(coords["pickup_y"]),
        "pickup_yaw": float(coords["pickup_yaw"]),
        
        # Drop-off coordinates
        "drop_x": float(coords["drop_x"]),
        "drop_y": float(coords["drop_y"]),
        "drop_yaw": float(coords["drop_yaw"]),
        "drop_zone_id": task.get("drop_zone_id"),
    }
    
    # =========================================================
    # STEP 4: Publish to MQTT with QoS=1
    # =========================================================
    topic = f"robot/{robot_id}/task/assignment"
    
    try:
        payload_json = json.dumps(payload)
        
        # Publish with QoS=1 for guaranteed delivery (at least once)
        result = mqtt_client.publish(topic, payload_json, qos=1)
        
        if result.rc == 0:  # MQTT_ERR_SUCCESS
            current_app.logger.info(
                f"[MQTT] ✅ Task {payload['task_id']} published successfully\n"
                f"       Topic: {topic}\n"
                f"       Pickup: ({payload['pickup_x']:.2f}, {payload['pickup_y']:.2f})\n"
                f"       Drop: ({payload['drop_x']:.2f}, {payload['drop_y']:.2f})\n"
                f"       Priority: {payload['priority']}, QoS: 1"
            )
            
            # Update task delivery status
            if task.get("_id"):
                db.tasks.update_one(
                    {"_id": task["_id"]},
                    {"$set": {
                        "mqtt_delivery_status": "SUCCESS",
                        "mqtt_delivery_attempted_at": datetime.utcnow(),
                        "mqtt_topic": topic
                    }}
                )
        else:
            raise Exception(f"MQTT publish failed with return code {result.rc}")
            
    except Exception as e:
        error_msg = str(e)
        current_app.logger.error(
            f"[MQTT] ❌ Failed to publish task {payload['task_id']} to {topic}\n"
            f"       Error: {error_msg}"
        )
        
        # Update task with failure status
        if task.get("_id"):
            db.tasks.update_one(
                {"_id": task["_id"]},
                {"$set": {
                    "mqtt_delivery_status": "FAILED",
                    "mqtt_delivery_errors": [error_msg],
                    "mqtt_delivery_attempted_at": datetime.utcnow(),
                    "mqtt_topic": topic
                }}
            )

# ---------------------------------------------------------
# Delete task
# ---------------------------------------------------------
@tasks_bp.route("/<id>", methods=["DELETE"])
@admin_required
def delete_task_route(id):
    db = get_db()
    try:
        oid = ObjectId(id)
    except Exception:
        return {"error": "invalid_id"}, 400

    res = db.tasks.delete_one({"_id": oid})
    if res.deleted_count == 0:
        return {"error": "not_found"}, 404

    return {"result": "deleted"}, 200


# ---------------------------------------------------------
# Get live task statistics
# ---------------------------------------------------------
@tasks_bp.route("/stats/live", methods=["GET"])
def get_live_stats():
    """Return real-time task and robot statistics for dashboard"""
    db = get_db()
    
    try:
        # Task statistics
        total_tasks = db.tasks.count_documents({})
        assigned_tasks = db.tasks.count_documents({"status": "ASSIGNED"})
        in_progress = db.tasks.count_documents({"status": {"$in": ["MOVING_TO_PICKUP", "ATTACHED", "MOVING_TO_DROP"]}})
        completed_tasks = db.tasks.count_documents({"status": "COMPLETED"})
        failed_tasks = db.tasks.count_documents({"status": "ERROR"})
        
        # Robot statistics
        total_robots = db.robots.count_documents({"deleted": False})
        available_robots = db.robots.count_documents({"deleted": False, "available": True})
        busy_robots = db.robots.count_documents({"deleted": False, "available": False})
        offline_robots = db.robots.count_documents({"deleted": False, "status": "OFFLINE"})
        
        # Average task duration
        completed = list(db.tasks.find({"status": "COMPLETED", "duration_seconds": {"$exists": True}}))
        avg_duration = sum(t.get("duration_seconds", 0) for t in completed) / max(len(completed), 1)
        
        return jsonify({
            "tasks": {
                "total": total_tasks,
                "assigned": assigned_tasks,
                "in_progress": in_progress,
                "completed": completed_tasks,
                "failed": failed_tasks,
                "average_duration_seconds": round(avg_duration, 2)
            },
            "robots": {
                "total": total_robots,
                "available": available_robots,
                "busy": busy_robots,
                "offline": offline_robots
            },
            "timestamp": datetime.utcnow().isoformat()
        }), 200
    except Exception as e:
        return jsonify({"error": str(e)}), 500
