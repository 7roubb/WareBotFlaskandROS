from flask import Blueprint, jsonify

from ..services import (
    dashboard_top_moving_products,
    dashboard_shelf_summary,
    dashboard_daily_movements
)

dashboard_bp = Blueprint("dashboard", __name__)


# =========================================================
# TOP MOVING PRODUCTS
# =========================================================
@dashboard_bp.route("/top-moving", methods=["GET"])
def dashboard_top_route():
    """
    Returns top picked products based on product_transactions
    """
    return jsonify(dashboard_top_moving_products())


# =========================================================
# SHELF SUMMARY
# =========================================================
@dashboard_bp.route("/shelves", methods=["GET"])
def dashboard_shelves_route():
    """
    Returns shelves summary:
    - shelf coords
    - total items
    - number of products
    """
    return jsonify(dashboard_shelf_summary())


# =========================================================
# DAILY MOVEMENTS
# =========================================================
@dashboard_bp.route("/daily", methods=["GET"])
def dashboard_daily_route():
    """
    Returns PICK, RETURN, ADJUST aggregated for the current day
    """
    return jsonify(dashboard_daily_movements())
