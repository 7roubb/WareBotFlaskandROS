from flask import Blueprint, request, jsonify
from flask_jwt_extended import jwt_required, get_jwt
from pydantic import ValidationError

from ..models import ProductCreate, ProductUpdate
from ..services import (
    create_product, list_products, get_product,
    update_product, soft_delete_product, search_products_by_name
)


products_bp = Blueprint("products", __name__)


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
# PRODUCT CRUD
# =========================================================
@products_bp.route("", methods=["POST"])
@admin_required
def create_product_route():
    try:
        data = ProductCreate(**request.json).dict()
    except ValidationError as e:
        return handle_validation_error(e)
    return jsonify(create_product(data)), 201


@products_bp.route("", methods=["GET"])
def list_products_route():
    return jsonify(list_products())


@products_bp.route("/<id>", methods=["GET"])
def get_product_route(id):
    p = get_product(id)
    return jsonify(p) if p else ({"error": "not_found"}, 404)


@products_bp.route("/<id>", methods=["PUT"])
@admin_required
def update_product_route(id):
    try:
        data = ProductUpdate(**request.json).dict(exclude_none=True)
    except ValidationError as e:
        return handle_validation_error(e)
    result = update_product(id, data)
    return jsonify(result) if result else ({"error": "not_found"}, 404)


@products_bp.route("/<id>", methods=["DELETE"])
@admin_required
def delete_product_route(id):
    if not soft_delete_product(id):
        return {"error": "not_found"}, 404
    return {"status": "deleted"}


@products_bp.route("/search", methods=["GET"])
def search_product_route():
    q = request.args.get("q", "").strip()
    return jsonify(search_products_by_name(q)) if q else []
