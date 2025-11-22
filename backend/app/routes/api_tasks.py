from flask import Blueprint, request, jsonify, current_app
from flask_jwt_extended import jwt_required, get_jwt
from pydantic import ValidationError
import json

from ..models import TaskCreate
from ..services import (
    create_task_and_assign,
    list_tasks,
)

tasks_bp = Blueprint("tasks", __name__)


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
# LIST ALL TASKS
# =========================================================
@tasks_bp.route("", methods=["GET"])
def list_tasks_route():
    return jsonify(list_tasks())


# =========================================================
# CREATE + ASSIGN TASK
# =========================================================
@tasks_bp.route("/assign", methods=["POST"])
@admin_required
def create_task_route():
    try:
        data = TaskCreate(**request.json).dict()
    except ValidationError as e:
        return handle_validation_error(e)

    task = create_task_and_assign(
        shelf_id=data["shelf_id"],
        priority=data["priority"],
        desc=data.get("description")
    )

    # MQTT publish to assigned robot
    robot_name = task["assigned_robot_name"]
    topic = f"robots/mp400/{robot_name}/task"

    payload = json.dumps({
        "task_id": task["id"],
        "task": "goto_shelf",
        "shelf_id": task["shelf_id"]
    })

    mqtt_client = current_app.mqtt
    if mqtt_client:
        mqtt_client.publish(topic, payload)
        current_app.logger.info(f"[MQTT] Sent task → {topic}: {payload}")

    return jsonify(task), 201
