from flask import Blueprint, request, jsonify
from flask_jwt_extended import jwt_required, get_jwt
from pydantic import ValidationError

from ..models import StockReturn, StockAdjust
from ..services import (
    subtract_from_product_stock,
    return_product_stock,
    adjust_product_stock,
)
from ..extensions import get_db

stock_bp = Blueprint("stock", __name__)


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
# PICK PRODUCT FROM STOCK
# =========================================================
@stock_bp.route("/<product_id>/pick", methods=["POST"])
@admin_required
def pick_stock_route(product_id):
    qty = request.json.get("quantity")

    if not qty or qty <= 0:
        return {"error": "invalid_quantity"}, 400

    try:
        new_qty = subtract_from_product_stock(
            product_id,
            qty,
            request.json.get("description")
        )
        return {"status": "ok", "new_quantity": new_qty}

    except ValueError as e:
        return {"error": str(e)}, 400


# =========================================================
# RETURN TO STOCK
# =========================================================
@stock_bp.route("/return", methods=["POST"])
@admin_required
def return_stock_route():
    try:
        data = StockReturn(**request.json).dict()
    except ValidationError as e:
        return handle_validation_error(e)

    try:
        return jsonify(
            return_product_stock(
                data["product_id"],
                data["quantity"],
                data.get("description")
            )
        )

    except ValueError as e:
        return {"error": str(e)}, 400


# =========================================================
# ADJUST STOCK
# =========================================================
@stock_bp.route("/adjust", methods=["POST"])
@admin_required
def adjust_stock_route():
    try:
        data = StockAdjust(**request.json).dict()
    except ValidationError as e:
        return handle_validation_error(e)

    try:
        return jsonify(
            adjust_product_stock(
                data["product_id"],
                data["new_quantity"],
                data.get("reason")
            )
        )
    except ValueError as e:
        return {"error": str(e)}, 400


# =========================================================
# GET PRODUCT TRANSACTIONS
# =========================================================
@stock_bp.route("/<product_id>/transactions", methods=["GET"])
def get_product_transactions_route(product_id):
    db = get_db()
    docs = (
        db.product_transactions
        .find({"product_id": product_id})
        .sort("timestamp", -1)
    )

    return jsonify([
        {**d, "id": str(d["_id"])} for d in docs
    ])
