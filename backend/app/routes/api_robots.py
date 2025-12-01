from flask import Blueprint, request, jsonify, current_app
from flask_jwt_extended import jwt_required, get_jwt
from pydantic import ValidationError

from ..models import RobotCreate, RobotUpdate
from ..extensions import get_influx
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
        "robot_id": "robot1"
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
