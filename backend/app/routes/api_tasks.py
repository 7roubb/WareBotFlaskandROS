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

    # determine canonical robot id for topic publishing
    db = get_db()
    robot_id_for_topic = None

    assigned_db_id = task.get("assigned_robot_id")
    if assigned_db_id:
        try:
            oid = ObjectId(assigned_db_id)
        except Exception:
            oid = assigned_db_id
        robot_doc = db.robots.find_one({"_id": oid, "deleted": False})
        if robot_doc:
            robot_id_for_topic = robot_doc.get("robot_id")

    if not robot_id_for_topic:
        candidate = task.get("assigned_robot_name")
        if candidate:
            robot_doc = db.robots.find_one({"robot_id": candidate, "deleted": False})
            if robot_doc:
                robot_id_for_topic = robot_doc.get("robot_id")
            else:
                robot_doc = db.robots.find_one({"name": candidate, "deleted": False})
                if robot_doc:
                    robot_id_for_topic = robot_doc.get("robot_id")

    if not robot_id_for_topic:
        robot_id_for_topic = task.get("assigned_robot_name")

    # Resolve current coordinates from shelf/zone at publish time (not at creation)
    try:
        # Extract task ObjectId string to resolve coordinates
        task_oid = str(task.get("_id")) if hasattr(task.get("_id"), '__str__') else task.get("_id")
        coords = resolve_task_coordinates(task_oid)
    except ValueError:
        # Fallback to stored coordinates if resolution fails. Prefer origin snapshots (storage/pickup)
        coords = {
            "pickup_x": task.get("origin_pickup_x") or task.get("pickup_x") or task.get("target_x"),
            "pickup_y": task.get("origin_pickup_y") or task.get("pickup_y") or task.get("target_y"),
            "pickup_yaw": task.get("origin_pickup_yaw") or task.get("pickup_yaw") or task.get("target_yaw") or 0.0,
            "drop_x": task.get("drop_x", task.get("zone_x", 0.0)),
            "drop_y": task.get("drop_y", task.get("zone_y", 0.0)),
            "drop_yaw": task.get("drop_yaw", 0.0),
        }

    # Build MQTT payload with dynamic coordinates
    payload_dict = {
        "task_id": task.get("id"),
        "task_type": task.get("task_type", "PICKUP_AND_DELIVER"),
        "shelf_id": task.get("shelf_id"),
        "priority": task.get("priority"),
        # Current coordinates resolved from shelf/zone
        "pickup_x": coords["pickup_x"],
        "pickup_y": coords["pickup_y"],
        "pickup_yaw": coords["pickup_yaw"],
        # Legacy keys
        "target_x": coords["pickup_x"],
        "target_y": coords["pickup_y"],
        "target_yaw": coords["pickup_yaw"],
        # Drop coordinates
        "drop_x": coords["drop_x"],
        "drop_y": coords["drop_y"],
        "drop_yaw": coords["drop_yaw"],
        "drop_zone_id": task.get("drop_zone_id"),
    }

    payload = json.dumps(payload_dict)
    mqtt_client = current_app.mqtt

    if mqtt_client:
        candidate_ids = []
        if robot_id_for_topic:
            candidate_ids.append(str(robot_id_for_topic))
        if task.get("assigned_robot_name"):
            candidate_ids.append(str(task.get("assigned_robot_name")))
        if task.get("assigned_robot_id"):
            candidate_ids.append(str(task.get("assigned_robot_id")))

        # unique preserve order
        seen = set()
        candidate_ids = [x for x in candidate_ids if not (x in seen or seen.add(x))]

        # add variants (no underscore) to increase chance of match
        alt_variants = []
        for cid in candidate_ids:
            if cid:
                no_us = cid.replace("_", "")
                if no_us and no_us != cid:
                    alt_variants.append(no_us)
        for v in alt_variants:
            if v not in candidate_ids:
                candidate_ids.append(v)

        # legacy topic (if robot_id_for_topic known)
        try:
            if robot_id_for_topic:
                legacy_topic = f"robots/mp400/{robot_id_for_topic}/task"
                mqtt_client.publish(legacy_topic, payload)
                current_app.logger.info(f"[MQTT] Sent task → {legacy_topic}: {payload}")
        except Exception:
            current_app.logger.exception("[MQTT] Failed to publish to legacy topic")

        # publish to robot/<id>/task/assignment for all candidate ids
        for cid in candidate_ids:
            try:
                t = f"robot/{cid}/task/assignment"
                mqtt_client.publish(t, payload)
                current_app.logger.info(f"[MQTT] Sent task → {t}: {payload}")
            except Exception:
                current_app.logger.exception(f"[MQTT] Failed to publish to {t}")

        # broadcast assignment
        try:
            br = "robot/all/task/assignment"
            mqtt_client.publish(br, payload)
            current_app.logger.info(f"[MQTT] Sent task → {br}: {payload}")
        except Exception:
            current_app.logger.exception("[MQTT] Failed to publish to broadcast assignment")

        # publish reference_point/update for drop zone (if present)
        try:
            drop_x = payload_dict.get("drop_x")
            drop_y = payload_dict.get("drop_y")
            drop_yaw = payload_dict.get("drop_yaw")
            if drop_x is not None and drop_y is not None:
                ref_payload = json.dumps({
                    "x": float(drop_x),
                    "y": float(drop_y),
                    "yaw": float(drop_yaw or 0.0),
                })
                for cid in candidate_ids:
                    try:
                        rt = f"robot/{cid}/reference_point/update"
                        mqtt_client.publish(rt, ref_payload)
                        current_app.logger.info(f"[MQTT] Sent ref → {rt}: {ref_payload}")
                    except Exception:
                        current_app.logger.exception(f"[MQTT] Failed to publish ref to {rt}")
                # broadcast ref
                try:
                    brt = "robot/all/reference_point/update"
                    mqtt_client.publish(brt, ref_payload)
                    current_app.logger.info(f"[MQTT] Sent ref → {brt}: {ref_payload}")
                except Exception:
                    current_app.logger.exception("[MQTT] Failed to publish broadcast ref")
        except Exception:
            current_app.logger.exception("[MQTT] Failed to publish reference_point update")

    # return created task
    return jsonify(task), 201


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
