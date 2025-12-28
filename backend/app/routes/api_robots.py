from datetime import datetime
from flask import Blueprint, request, jsonify, current_app
from flask_jwt_extended import jwt_required, get_jwt
from pydantic import ValidationError
from bson import ObjectId

from ..models import RobotCreate, RobotUpdate
from ..extensions import get_influx, get_db
from ..services import (
    create_robot, list_robots, get_robot,
    update_robot, soft_delete_robot,
)

robots_bp = Blueprint("robots", __name__)


# =========================================================
# HELPERS
# =========================================================
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


# =========================================================
# ROBOT CRUD
# =========================================================
@robots_bp.route("", methods=["POST"])
@admin_required
def create_robot_route():
    """
    Create a robot using only:
    {
        "name": "Robot A",
        "robot_id": "robot1",
        "status": "IDLE"  # Optional, defaults to IDLE
    }
    Backend generates:
        topic = robots/mp400/robot1/status
    """
    try:
        data = RobotCreate(**request.json).dict()
    except ValidationError as e:
        return handle_validation_error(e)

    robot = create_robot(data)

    # Auto-subscribe MQTT
    topic = robot["topic"]
    mqtt_client = current_app.mqtt

    if mqtt_client:
        mqtt_client.subscribe(topic)
        current_app.logger.info(f"[MQTT] Subscribed to {topic}")

    return jsonify(robot), 201


@robots_bp.route("", methods=["GET"])
def list_robots_route():
    return jsonify(list_robots())


@robots_bp.route("/<id>", methods=["GET"])
def get_robot_route(id):
    r = get_robot(id)
    return jsonify(r) if r else ({"error": "not_found"}, 404)


@robots_bp.route("/<id>", methods=["PUT"])
@admin_required
def update_robot_route(id):
    try:
        data = RobotUpdate(**request.json).dict(exclude_none=True)
    except ValidationError as e:
        return handle_validation_error(e)

    updated = update_robot(id, data)

    # Robot ID changed → resubscribe new MQTT topic
    if updated and "robot_id" in data:
        new_topic = updated["topic"]
        mqtt_client = current_app.mqtt
        if mqtt_client:
            mqtt_client.subscribe(new_topic)
            current_app.logger.info(f"[MQTT] Resubscribed to {new_topic}")

    return jsonify(updated) if updated else ({"error": "not_found"}, 404)


@robots_bp.route("/<id>", methods=["DELETE"])
@admin_required
def delete_robot_route(id):
    if not soft_delete_robot(id):
        return {"error": "not_found"}, 404
    return {"status": "deleted"}


# =========================================================
# ROBOT STATUS MANAGEMENT
# =========================================================
@robots_bp.route("/<robot_id>/status", methods=["PATCH"])
def update_robot_status(robot_id: str):
    """
    Update robot status (IDLE, BUSY, ERROR, OFFLINE).
    
    Body: {"status": "IDLE"}
    """
    try:
        data = request.get_json()
        new_status = data.get("status", "").upper()
        
        # Validate status
        valid_statuses = {"IDLE", "BUSY", "ERROR", "OFFLINE"}
        if new_status not in valid_statuses:
            return jsonify({
                "error": f"Invalid status. Must be one of: {', '.join(valid_statuses)}"
            }), 400
        
        db = get_db()
        
        # Try by MongoDB _id first
        try:
            oid = ObjectId(robot_id)
            result = db.robots.update_one(
                {"_id": oid},
                {"$set": {
                    "status": new_status,
                    "updated_at": datetime.utcnow()
                }}
            )
        except:
            # Try by robot_id string
            result = db.robots.update_one(
                {"robot_id": robot_id},
                {"$set": {
                    "status": new_status,
                    "updated_at": datetime.utcnow()
                }}
            )
        
        if result.modified_count > 0:
            current_app.logger.info(f"[ROBOT STATUS] Updated robot {robot_id} status to {new_status}")
            return jsonify({
                "message": f"Robot status updated to {new_status}",
                "status": new_status
            }), 200
        else:
            return jsonify({"error": "Robot not found"}), 404
            
    except Exception as e:
        current_app.logger.error(f"[ROBOT STATUS] Error: {e}", exc_info=True)
        return jsonify({"error": str(e)}), 500


@robots_bp.route("/reset-all-idle", methods=["POST"])
@admin_required
def reset_all_robots_to_idle():
    """
    Reset ALL robots to IDLE status (emergency use only).
    Use when system gets into inconsistent state.
    """
    try:
        db = get_db()
        result = db.robots.update_many(
            {"deleted": False},
            {"$set": {
                "status": "IDLE",
                "current_shelf_id": None,
                "updated_at": datetime.utcnow()
            }}
        )
        
        current_app.logger.warning(f"[ROBOT RESET] Reset {result.modified_count} robots to IDLE")
        return jsonify({
            "message": f"Reset {result.modified_count} robots to IDLE",
            "count": result.modified_count
        }), 200
        
    except Exception as e:
        current_app.logger.error(f"[ROBOT RESET] Error: {e}", exc_info=True)
        return jsonify({"error": str(e)}), 500


@robots_bp.route("/<robot_id>/set-idle", methods=["POST"])
def set_robot_idle(robot_id: str):
    """
    Set a specific robot to IDLE status.
    Use when robot is stuck in BUSY/ERROR state.
    """
    try:
        db = get_db()
        
        try:
            oid = ObjectId(robot_id)
            result = db.robots.update_one(
                {"_id": oid},
                {"$set": {
                    "status": "IDLE",
                    "current_shelf_id": None,
                    "updated_at": datetime.utcnow()
                }}
            )
        except:
            result = db.robots.update_one(
                {"robot_id": robot_id},
                {"$set": {
                    "status": "IDLE",
                    "current_shelf_id": None,
                    "updated_at": datetime.utcnow()
                }}
            )
        
        if result.modified_count > 0:
            current_app.logger.info(f"[ROBOT RESET] Set robot {robot_id} to IDLE")
            return jsonify({
                "message": f"Robot {robot_id} is now IDLE",
                "status": "IDLE"
            }), 200
        else:
            return jsonify({"error": "Robot not found"}), 404
            
    except Exception as e:
        current_app.logger.error(f"[ROBOT RESET] Error: {e}", exc_info=True)
        return jsonify({"error": str(e)}), 500


@robots_bp.route("/<robot_id>/set-busy", methods=["POST"])
def set_robot_busy(robot_id: str):
    """Set robot to BUSY status (used by task assignment)"""
    try:
        db = get_db()
        
        try:
            oid = ObjectId(robot_id)
            result = db.robots.update_one(
                {"_id": oid},
                {"$set": {
                    "status": "BUSY",
                    "updated_at": datetime.utcnow()
                }}
            )
        except:
            result = db.robots.update_one(
                {"robot_id": robot_id},
                {"$set": {
                    "status": "BUSY",
                    "updated_at": datetime.utcnow()
                }}
            )
        
        if result.modified_count > 0:
            current_app.logger.info(f"[ROBOT STATUS] Set robot {robot_id} to BUSY")
            return jsonify({
                "message": f"Robot {robot_id} is now BUSY",
                "status": "BUSY"
            }), 200
        else:
            return jsonify({"error": "Robot not found"}), 404
            
    except Exception as e:
        current_app.logger.error(f"[ROBOT STATUS] Error: {e}", exc_info=True)
        return jsonify({"error": str(e)}), 500


# =========================================================
# ROBOT TELEMETRY (InfluxDB)
# =========================================================
@robots_bp.route("/<robot_id>/telemetry/latest", methods=["GET"])
def api_get_latest_influx(robot_id):
    influx_client, _ = get_influx()
    query_api = influx_client.query_api()
    bucket = current_app.config["INFLUX_BUCKET"]

    # Flux Query
    query = f'''
    from(bucket: "{bucket}")
        |> range(start: -24h)
        |> filter(fn: (r) => r["robot"] == "{robot_id}")
        |> filter(fn: (r) => r["_measurement"] == "robot_telemetry")
        |> last()
    '''

    tables = query_api.query(query)
    latest = {}

    for table in tables:
        for record in table.records:
            latest[record.get_field()] = record.get_value()
            latest["time"] = record.get_time().isoformat()

    if not latest:
        return {"error": "not_found"}, 404

    return latest