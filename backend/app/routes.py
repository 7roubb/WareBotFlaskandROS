from functools import wraps
from flask import Blueprint, request, jsonify, current_app
from pydantic import ValidationError
from flask_jwt_extended import jwt_required, get_jwt
from flask_cors import cross_origin
import json

from .models import (
    ProductCreate, ProductUpdate,
    ShelfCreate, ShelfUpdate,
    RobotCreate, RobotUpdate,
    TaskCreate,
    StockReturn, StockAdjust
)

from .services import (
    create_product, list_products, get_product,
    update_product, soft_delete_product, search_products_by_name,
    subtract_from_product_stock, return_product_stock, adjust_product_stock,
    create_shelf, list_shelves, get_shelf, update_shelf,
    soft_delete_shelf, get_products_for_shelf,
    create_robot, list_robots, get_robot,
    update_robot, soft_delete_robot,
    create_task_and_assign, list_tasks,
    dashboard_top_moving_products, dashboard_shelf_summary, dashboard_daily_movements,
    upload_image_to_minio, delete_image_from_minio, update_product_images
)

from .extensions import get_db
from .extensions import get_influx

api_bp = Blueprint("api", __name__)


# =========================================================
# HELPERS
# =========================================================
def handle_validation_error(err: ValidationError):
    return jsonify({"error": "validation_error", "details": err.errors()}), 400


def admin_required(fn):
    @wraps(fn)
    @jwt_required()
    def wrapper(*args, **kwargs):
        claims = get_jwt()
        if claims.get("role") != "ADMIN":
            return jsonify({"error": "forbidden"}), 403
        return fn(*args, **kwargs)
    return wrapper


# =========================================================
# PRODUCT ROUTES
# =========================================================
@api_bp.route("/products", methods=["POST"])
@admin_required
def create_product_route():
    try:
        data = ProductCreate(**request.json).dict()
    except ValidationError as e:
        return handle_validation_error(e)
    return jsonify(create_product(data)), 201


@api_bp.route("/products", methods=["GET"])
def list_products_route():
    return jsonify(list_products())


@api_bp.route("/products/<id>", methods=["GET"])
def get_product_route(id):
    p = get_product(id)
    return jsonify(p) if p else ({"error": "not_found"}, 404)


@api_bp.route("/products/<id>", methods=["PUT"])
@admin_required
def update_product_route(id):
    try:
        data = ProductUpdate(**request.json).dict(exclude_none=True)
    except ValidationError as e:
        return handle_validation_error(e)
    result = update_product(id, data)
    return jsonify(result) if result else ({"error": "not_found"}, 404)


@api_bp.route("/products/<id>", methods=["DELETE"])
@admin_required
def delete_product_route(id):
    if not soft_delete_product(id):
        return {"error": "not_found"}, 404
    return {"status": "deleted"}


@api_bp.route("/products/search", methods=["GET"])
def search_product_route():
    q = request.args.get("q", "").strip()
    return jsonify(search_products_by_name(q)) if q else []


# =========================================================
# PRODUCT IMAGES
# =========================================================
@api_bp.route("/products/<product_id>/images", methods=["POST", "OPTIONS"])
@cross_origin()
@admin_required
def upload_image_route(product_id):

    # Handle CORS preflight
    if request.method == "OPTIONS":
        return jsonify({}), 200

    if "image" not in request.files:
        return {"error": "no_image_uploaded"}, 400

    file = request.files["image"]
    if not file.filename:
        return {"error": "empty_filename"}, 400

    content = file.read()
    url = upload_image_to_minio(
        file_content=content,
        filename=file.filename,
        product_id=product_id,
        content_type=file.mimetype
    )

    product = get_product(product_id)
    if not product:
        return {"error": "product_not_found"}, 404

    # Add to product images
    images = product.get("image_urls", [])
    images.append(url)
    updated = update_product_images(product_id, image_urls=images)

    return jsonify({
        "status": "uploaded",
        "image_url": url,
        "product": updated
    })


@api_bp.route("/products/<product_id>/images/<int:index>", methods=["DELETE"])
@admin_required
def delete_image_route(product_id, index):
    product = get_product(product_id)
    if not product:
        return {"error": "product_not_found"}, 404

    images = product.get("image_urls", [])

    if index < 0 or index >= len(images):
        return {"error": "invalid_index"}, 400

    delete_image_from_minio(images[index])

    images.pop(index)
    updated = update_product_images(product_id, image_urls=images)

    return jsonify({"status": "deleted", "product": updated})


@api_bp.route("/products/<product_id>/images/set-main", methods=["PUT"])
@admin_required
def set_main_image_route(product_id):
    url = request.json.get("image_url")
    if not url:
        return {"error": "image_url_required"}, 400
    updated = update_product_images(product_id, main_image_url=url)
    return jsonify(updated)


@api_bp.route("/products/<product_id>/images", methods=["GET"])
def get_product_images_route(product_id):
    p = get_product(product_id)
    if not p:
        return {"error": "not_found"}, 404
    return {
        "product_id": product_id,
        "main_image_url": p.get("main_image_url"),
        "image_urls": p.get("image_urls", [])
    }


# =========================================================
# PRODUCT STOCK
# =========================================================
@api_bp.route("/products/<product_id>/pick", methods=["POST"])
@admin_required
def pick_stock_route(product_id):
    qty = request.json.get("quantity")
    if not qty or qty <= 0:
        return {"error": "invalid_quantity"}, 400

    try:
        new_qty = subtract_from_product_stock(product_id, qty, request.json.get("description"))
        return {"status": "ok", "new_quantity": new_qty}
    except ValueError as e:
        return {"error": str(e)}, 400


@api_bp.route("/products/return", methods=["POST"])
@admin_required
def return_stock_route():
    try:
        data = StockReturn(**request.json).dict()
    except ValidationError as e:
        return handle_validation_error(e)

    try:
        return jsonify(return_product_stock(data["product_id"], data["quantity"], data.get("description")))
    except ValueError as e:
        return {"error": str(e)}, 400


@api_bp.route("/products/adjust", methods=["POST"])
@admin_required
def adjust_stock_route():
    try:
        data = StockAdjust(**request.json).dict()
    except ValidationError as e:
        return handle_validation_error(e)

    try:
        return jsonify(adjust_product_stock(data["product_id"], data["new_quantity"], data.get("reason")))
    except ValueError as e:
        return {"error": str(e)}, 400


@api_bp.route("/products/<product_id>/transactions", methods=["GET"])
def get_product_transactions_route(product_id):
    db = get_db()
    docs = db.product_transactions.find({"product_id": product_id}).sort("timestamp", -1)
    return jsonify([{**d, "id": str(d["_id"])} for d in docs])


# =========================================================
# SHELVES
# =========================================================
@api_bp.route("/shelves", methods=["POST"])
@admin_required
def create_shelf_route():
    try:
        data = ShelfCreate(**request.json).dict()
    except ValidationError as e:
        return handle_validation_error(e)
    return jsonify(create_shelf(data)), 201


@api_bp.route("/shelves", methods=["GET"])
def list_shelves_route():
    return jsonify(list_shelves())


@api_bp.route("/shelves/<id>", methods=["GET"])
def get_shelf_route(id):
    s = get_shelf(id)
    return jsonify(s) if s else ({"error": "not_found"}, 404)


@api_bp.route("/shelves/<id>", methods=["PUT"])
@admin_required
def update_shelf_route(id):
    try:
        data = ShelfUpdate(**request.json).dict(exclude_none=True)
    except ValidationError as e:
        return handle_validation_error(e)
    result = update_shelf(id, data)
    return jsonify(result) if result else ({"error": "not_found"}, 404)


@api_bp.route("/shelves/<id>", methods=["DELETE"])
@admin_required
def delete_shelf_route(id):
    if not soft_delete_shelf(id):
        return {"error": "not_found"}, 404
    return {"status": "deleted"}


@api_bp.route("/shelves/<id>/products", methods=["GET"])
def shelf_products_route(id):
    return jsonify(get_products_for_shelf(id))


# =========================================================
# ROBOTS
# =========================================================
@api_bp.route("/robots", methods=["POST"])
@admin_required
def create_robot_route():
    """
    Create a robot using only:
    {
        "name": "Robot A",
        "robot_id": "robot1"
    }
    The backend generates:
        robots/mp400/robot1/status
    """
    try:
        data = RobotCreate(**request.json).dict()
    except ValidationError as e:
        return handle_validation_error(e)

    robot = create_robot(data)

    # Auto subscribe to MQTT topic
    topic = robot["topic"]     # robots/mp400/<id>/status
    mqtt_client = current_app.mqtt

    if mqtt_client:
        mqtt_client.subscribe(topic)
        current_app.logger.info(f"[MQTT] Subscribed to {topic}")

    return jsonify(robot), 201


@api_bp.route("/robots", methods=["GET"])
def list_robots_route():
    return jsonify(list_robots())


@api_bp.route("/robots/<id>", methods=["GET"])
def get_robot_route(id):
    r = get_robot(id)
    return jsonify(r) if r else ({"error": "not_found"}, 404)


@api_bp.route("/robots/<id>", methods=["PUT"])
@admin_required
def update_robot_route(id):
    try:
        data = RobotUpdate(**request.json).dict(exclude_none=True)
    except ValidationError as e:
        return handle_validation_error(e)

    updated = update_robot(id, data)

    # If robot_id is changed → resubscribe
    if "robot_id" in data:
        new_topic = updated["topic"]
        mqtt_client = current_app.mqtt
        if mqtt_client:
            mqtt_client.subscribe(new_topic)
            current_app.logger.info(f"[MQTT] Resubscribed to {new_topic}")

    return jsonify(updated) if updated else ({"error": "not_found"}, 404)


@api_bp.route("/robots/<id>", methods=["DELETE"])
@admin_required
def delete_robot_route(id):
    if not soft_delete_robot(id):
        return {"error": "not_found"}, 404
    return {"status": "deleted"}


@api_bp.route("/robots/<robot_id>/telemetry/latest", methods=["GET"])
def api_get_latest_influx(robot_id):
    influx_client, write_api = get_influx()

    # Get query API from client
    query_api = influx_client.query_api()
    bucket = current_app.config["INFLUX_BUCKET"]

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


# =========================================================
# TASK SYSTEM
# =========================================================
@api_bp.route("/tasks", methods=["GET"])
def list_tasks_route():
    return jsonify(list_tasks())


@api_bp.route("/tasks/assign", methods=["POST"])
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

    # Publish task to robot (MQTT)
    topic = f"robots/mp400/{task['assigned_robot_name']}/task"
    payload = json.dumps({
        "task_id": task["id"],
        "task": "goto_shelf",
        "shelf_id": task["shelf_id"]
    })
    current_app.mqtt.publish(topic, payload)

    return jsonify(task), 201


# =========================================================
# DASHBOARD
# =========================================================
@api_bp.route("/dashboard/top-moving", methods=["GET"])
def dashboard_top_route():
    return jsonify(dashboard_top_moving_products())


@api_bp.route("/dashboard/shelves", methods=["GET"])
def dashboard_shelves_route():
    return jsonify(dashboard_shelf_summary())


@api_bp.route("/dashboard/daily", methods=["GET"])
def dashboard_daily_route():
    return jsonify(dashboard_daily_movements())


# =========================================================
# MAP ROUTES
# =========================================================
@api_bp.route("/maps/merged", methods=["GET"])
def get_merged_map():
    db = get_db()
    doc = db.maps.find_one({"name": "merged_map"})
    if not doc:
        return {"error": "not_found"}, 404

    doc["id"] = str(doc["_id"])
    doc.pop("_id", None)
    return doc
