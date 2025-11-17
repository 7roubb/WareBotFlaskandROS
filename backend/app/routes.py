from functools import wraps
from flask import Blueprint, request, jsonify, current_app
from pydantic import ValidationError
from flask_jwt_extended import jwt_required, get_jwt
import json

# Validation Models
from .models import (
    ProductCreate,
    ProductUpdate,
    ShelfCreate,
    ShelfUpdate,
    RobotCreate,
    RobotUpdate,
    TaskCreate
)

# Services
from .services import (
    create_product, list_products, get_product,
    update_product, soft_delete_product, search_products_by_name,
    get_shelf_for_product,
    create_shelf, list_shelves, get_shelf,
    update_shelf, soft_delete_shelf, get_products_for_shelf,
    create_robot, list_robots, get_robot,
    update_robot, soft_delete_robot,
    serialize_doc,
    create_task_and_assign,
    list_tasks
)

# MinIO + Mongo
from .extensions import minio_client, get_db

# Blueprint
api_bp = Blueprint("api", __name__)


# ---------------------------------------------------------
# Helpers
# ---------------------------------------------------------
def handle_pydantic_error(err: ValidationError):
    return jsonify({"error": "validation_error", "details": err.errors()}), 400


def admin_required(fn):
    """Allow only Admins with JWT"""
    @wraps(fn)
    @jwt_required()
    def wrapper(*args, **kwargs):
        claims = get_jwt()
        if claims.get("role") != "ADMIN":
            return jsonify({"error": "forbidden", "msg": "Admins only"}), 403
        return fn(*args, **kwargs)
    return wrapper


# ---------------------------------------------------------
# PRODUCT ROUTES
# ---------------------------------------------------------
@api_bp.route("/products", methods=["POST"])
@admin_required
def create_product_route():
    try:
        data = ProductCreate(**request.json).dict()
    except ValidationError as e:
        return handle_pydantic_error(e)
    return jsonify(create_product(data)), 201


@api_bp.route("/products", methods=["GET"])
def list_products_route():
    return jsonify(list_products())


@api_bp.route("/products/<product_id>", methods=["GET"])
def get_product_route(product_id):
    product = get_product(product_id)
    if not product:
        return jsonify({"error": "not_found"}), 404
    return jsonify(product)


@api_bp.route("/products/<product_id>", methods=["PUT"])
@admin_required
def update_product_route(product_id):
    try:
        data = ProductUpdate(**request.json).dict(exclude_none=True)
    except ValidationError as e:
        return handle_pydantic_error(e)
    updated = update_product(product_id, data)
    if not updated:
        return jsonify({"error": "not_found"}), 404
    return jsonify(updated)


@api_bp.route("/products/<product_id>", methods=["DELETE"])
@admin_required
def delete_product_route(product_id):
    ok = soft_delete_product(product_id)
    if not ok:
        return jsonify({"error": "not_found"}), 404
    return jsonify({"status": "deleted"})


@api_bp.route("/products/search", methods=["GET"])
def search_products_route():
    q = request.args.get("q", "").strip()
    if not q:
        return jsonify([])
    return jsonify(search_products_by_name(q))


@api_bp.route("/products/<product_id>/shelf", methods=["GET"])
def get_product_shelf_route(product_id):
    shelf = get_shelf_for_product(product_id)
    if not shelf:
        return jsonify({"error": "not_found"}), 404
    return jsonify(shelf)


# -------------------- MINIO PRESIGNED UPLOAD URL --------------------
@api_bp.route("/products/upload-url", methods=["GET"])
@jwt_required()
def get_presigned_url():
    filename = request.args.get("filename")
    if not filename:
        return {"error": "filename_required"}, 400

    bucket = current_app.config["MINIO_BUCKET"]

    upload_url = minio_client.presigned_put_object(bucket, filename)
    public_url = f"http://localhost:9000/{bucket}/{filename}"

    return {"upload_url": upload_url, "public_url": public_url}


# ---------------------- FILTER PRODUCTS -----------------------------
@api_bp.route("/products/filter", methods=["GET"])
def filter_products():
    db = get_db()
    query = {"deleted": False}

    brand = request.args.get("brand")
    category = request.args.get("category")
    min_price = request.args.get("min_price")
    max_price = request.args.get("max_price")

    if brand:
        query["brand"] = brand
    if category:
        query["category"] = category
    if min_price:
        query["price"] = {"$gte": float(min_price)}
    if max_price:
        query.setdefault("price", {})
        query["price"]["$lte"] = float(max_price)

    docs = db.products.find(query)
    return jsonify([serialize_doc(doc) for doc in docs])


# ---------------------------------------------------------
# SHELF ROUTES
# ---------------------------------------------------------
@api_bp.route("/shelves", methods=["POST"])
@admin_required
def create_shelf_route():
    try:
        data = ShelfCreate(**request.json).dict()
    except ValidationError as e:
        return handle_pydantic_error(e)
    return jsonify(create_shelf(data)), 201


@api_bp.route("/shelves", methods=["GET"])
def list_shelves_route():
    return jsonify(list_shelves())


@api_bp.route("/shelves/<shelf_id>", methods=["GET"])
def get_shelf_route(shelf_id):
    shelf = get_shelf(shelf_id)
    if not shelf:
        return jsonify({"error": "not_found"}), 404
    return jsonify(shelf)


@api_bp.route("/shelves/<shelf_id>", methods=["PUT"])
@admin_required
def update_shelf_route(shelf_id):
    try:
        data = ShelfUpdate(**request.json).dict(exclude_none=True)
    except ValidationError as e:
        return handle_pydantic_error(e)
    updated = update_shelf(shelf_id, data)
    if not updated:
        return jsonify({"error": "not_found"}), 404
    return jsonify(updated)


@api_bp.route("/shelves/<shelf_id>", methods=["DELETE"])
@admin_required
def delete_shelf_route(shelf_id):
    ok = soft_delete_shelf(shelf_id)
    if not ok:
        return jsonify({"error": "not_found"}), 404
    return jsonify({"status": "deleted"})


@api_bp.route("/shelves/<shelf_id>/products", methods=["GET"])
def get_shelf_products_route(shelf_id):
    return jsonify(get_products_for_shelf(shelf_id))


# ---------------------------------------------------------
# ROBOT ROUTES
# ---------------------------------------------------------
@api_bp.route("/robots", methods=["POST"])
@admin_required
def create_robot_route():
    try:
        data = RobotCreate(**request.json).dict()
    except ValidationError as e:
        return handle_pydantic_error(e)

    return jsonify(create_robot(data)), 201


@api_bp.route("/robots", methods=["GET"])
def list_robots_route():
    return jsonify(list_robots())


@api_bp.route("/robots/<robot_id>", methods=["GET"])
def get_robot_route(robot_id):
    robot = get_robot(robot_id)
    if not robot:
        return jsonify({"error": "not_found"}), 404
    return jsonify(robot)


@api_bp.route("/robots/<robot_id>", methods=["PUT"])
@admin_required
def update_robot_route(robot_id):
    try:
        data = RobotUpdate(**request.json).dict(exclude_none=True)
    except ValidationError as e:
        return handle_pydantic_error(e)

    updated = update_robot(robot_id, data)
    if not updated:
        return jsonify({"error": "not_found"}), 404
    return jsonify(updated)


@api_bp.route("/robots/<robot_id>", methods=["DELETE"])
@admin_required
def delete_robot_route(robot_id):
    ok = soft_delete_robot(robot_id)
    if not ok:
        return jsonify({"error": "not_found"}), 404
    return jsonify({"status": "deleted"})


# ---------------------------------------------------------
# TASK SYSTEM (Assign Task → Best Robot → Send via MQTT)
# ---------------------------------------------------------

@api_bp.route("/tasks", methods=["GET"])
def list_tasks_route():
    return jsonify(list_tasks())


@api_bp.route("/tasks/assign", methods=["POST"])
@admin_required
def assign_task_route():
    """
    Assign task to the best robot (distance + battery + availability)
    """
    try:
        data = TaskCreate(**request.json).dict()
    except ValidationError as e:
        return handle_pydantic_error(e)

    shelf_id = data["shelf_id"]
    priority = data["priority"]
    description = data.get("description")

    # Create & assign task
    try:
        task = create_task_and_assign(shelf_id, priority, description)
    except ValueError as e:
        return jsonify({"error": str(e)}), 400

    # Send MQTT command to robot
    topic = f"robots/mp400/{task['assigned_robot_name']}/task"
    payload = {
        "task_id": task["id"],
        "task": "goto_shelf",
        "shelf_id": task["shelf_id"]
    }

    current_app.mqtt.publish(topic, json.dumps(payload))

    return jsonify(task), 201
