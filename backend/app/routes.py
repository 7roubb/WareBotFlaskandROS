from functools import wraps
from flask import Blueprint, request, jsonify, current_app
from pydantic import ValidationError
from flask_jwt_extended import jwt_required, get_jwt
import json

# Validation Models
from .models import (
    ProductCreate, ProductUpdate,
    ShelfCreate, ShelfUpdate,
    RobotCreate, RobotUpdate,
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
    list_tasks,
    subtract_from_product_stock,
    dashboard_daily_movements,
    dashboard_shelf_summary,
    dashboard_top_moving_products,
    return_product_stock,
    record_product_transaction,
    adjust_product_stock
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
# PRODUCT STOCK + TRANSACTIONS
# ---------------------------------------------------------
@api_bp.route("/products/<product_id>/pick", methods=["POST"])
@admin_required
def pick_product_route(product_id):
    body = request.json
    qty = body.get("quantity")
    description = body.get("description")

    if qty is None or qty <= 0:
        return {"error": "invalid_quantity"}, 400

    try:
        new_qty = subtract_from_product_stock(product_id, qty, description)
    except ValueError as e:
        return {"error": str(e)}, 400

    return jsonify({"status": "ok", "new_quantity": new_qty}), 200


@api_bp.route("/products/return", methods=["POST"])
@admin_required
def return_stock_route():
    from .models import StockReturn
    try:
        data = StockReturn(**request.json).dict()
    except ValidationError as e:
        return handle_pydantic_error(e)

    try:
        updated = return_product_stock(
            data["product_id"],
            data["quantity"],
            data.get("description")
        )
    except ValueError as ex:
        return {"error": str(ex)}, 404

    return jsonify(updated), 200


@api_bp.route("/products/adjust", methods=["POST"])
@admin_required
def adjust_stock_route():
    from .models import StockAdjust
    try:
        data = StockAdjust(**request.json).dict()
    except ValidationError as e:
        return handle_pydantic_error(e)

    try:
        updated = adjust_product_stock(
            data["product_id"],
            data["new_quantity"],
            data.get("reason")
        )
    except ValueError as ex:
        return {"error": str(ex)}, 404

    return jsonify(updated), 200


@api_bp.route("/products/<product_id>/transactions", methods=["GET"])
def product_transactions_route(product_id):
    db = get_db()
    docs = db.product_transactions.find({"product_id": product_id}).sort("timestamp", -1)

    result = []
    for d in docs:
        d["id"] = str(d["_id"])
        d.pop("_id", None)
        result.append(d)

    return jsonify(result)


# ---------------------------------------------------------
# MAP ROUTES
# ---------------------------------------------------------
@api_bp.route("/maps/merged", methods=["GET"])
def get_merged_map():
    db = get_db()
    doc = db.maps.find_one({"name": "merged_map"})

    if not doc:
        return {"error": "map_not_found"}, 404

    doc["id"] = str(doc["_id"])
    doc.pop("_id", None)
    return doc


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


@api_bp.route("/shelves/<shelf_id>/contents", methods=["GET"])
def get_shelf_contents_route(shelf_id):
    shelf = get_shelf(shelf_id)
    if not shelf:
        return jsonify({"error": "not_found"}), 404

    products = get_products_for_shelf(shelf_id)

    return jsonify({
        "shelf": shelf,
        "products": products
    })


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

    # Store robot with topic
    robot = create_robot(data)

    # Subscribe to robot topic via MQTT
    topic = data.get("topic")
    mqtt_client = current_app.mqtt
    if mqtt_client and topic:
        mqtt_client.subscribe(topic)
        current_app.logger.info(f"[MQTT] Subscribed to robot topic: {topic}")

    return jsonify(robot), 201


# ---------------------------------------------------------
# MONITOR ROBOT BY TOPIC
# ---------------------------------------------------------
@api_bp.route("/robots/monitor", methods=["POST"])
@admin_required
def monitor_robot_topic_route():
    """
    Register a robot topic to monitor and subscribe to it via MQTT.
    Body: { "topic": "robots/mp400/robot1/custom_topic" }
    """
    topic = request.json.get("topic")
    if not topic or not isinstance(topic, str):
        return jsonify({"error": "invalid_topic"}), 400

    # Subscribe to the topic using the MQTT client
    mqtt_client = current_app.mqtt
    if mqtt_client:
        mqtt_client.subscribe(topic)
        current_app.logger.info(f"[MQTT] Subscribed to new robot topic: {topic}")
        return jsonify({"status": "subscribed", "topic": topic}), 200
    else:
        return jsonify({"error": "mqtt_not_initialized"}), 500


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
# TASK ROUTES (Assign Task → Best Robot → MQTT)
# ---------------------------------------------------------
@api_bp.route("/tasks", methods=["GET"])
def list_tasks_route():
    return jsonify(list_tasks())


@api_bp.route("/tasks/assign", methods=["POST"])
@admin_required
def assign_task_route():
    try:
        data = TaskCreate(**request.json).dict()
    except ValidationError as e:
        return handle_pydantic_error(e)

    shelf_id = data["shelf_id"]
    priority = data["priority"]
    description = data.get("description")

    try:
        task = create_task_and_assign(shelf_id, priority, description)
    except ValueError as e:
        return jsonify({"error": str(e)}), 400

    topic = f"robots/mp400/{task['assigned_robot_name']}/task"
    payload = {
        "task_id": task["id"],
        "task": "goto_shelf",
        "shelf_id": task["shelf_id"]
    }

    current_app.mqtt.publish(topic, json.dumps(payload))

    return jsonify(task), 201


# ---------------------------------------------------------
# DASHBOARD ROUTES
# ---------------------------------------------------------
@api_bp.route("/dashboard/top-moving", methods=["GET"])
def dashboard_top_moving_route():
    return jsonify(dashboard_top_moving_products())


@api_bp.route("/dashboard/shelves", methods=["GET"])
def dashboard_shelves_route():
    return jsonify(dashboard_shelf_summary())


@api_bp.route("/dashboard/daily", methods=["GET"])
def dashboard_daily_route():
    return jsonify(dashboard_daily_movements())
