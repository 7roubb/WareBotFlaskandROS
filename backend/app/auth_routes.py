from flask import Blueprint, request, jsonify
from pydantic import ValidationError
from flask_jwt_extended import create_access_token

from .models import AdminCreate, AdminLogin
from .services import create_admin, get_admin_by_username, verify_admin_password

auth_bp = Blueprint("auth", __name__)


# ---------------------------------------------
# Helper: return unified validation error
# ---------------------------------------------
def handle_pydantic_error(err: ValidationError):
    return jsonify({
        "error": "validation_error",
        "details": err.errors()
    }), 400


# ---------------------------------------------
# Register Admin (Optional)
# ---------------------------------------------
@auth_bp.route("/register-admin", methods=["POST"])
def register_admin():
    """
    Create a new admin (should be disabled in production)
    """
    try:
        data = AdminCreate(**request.json).dict()
    except ValidationError as e:
        return handle_pydantic_error(e)

    # Check if username exists
    if get_admin_by_username(data["username"]):
        return jsonify({"error": "username_taken"}), 400

    admin = create_admin(data["username"], data["password"])
    admin.pop("password_hash", None)

    return jsonify(admin), 201


# ---------------------------------------------
# Admin Login → returns JWT token
# ---------------------------------------------
@auth_bp.route("/login", methods=["POST"])
def admin_login():
    """
    Admin login returns JWT token
    """
    try:
        data = AdminLogin(**request.json).dict()
    except ValidationError as e:
        return handle_pydantic_error(e)

    if not verify_admin_password(data["username"], data["password"]):
        return jsonify({"error": "invalid_credentials"}), 401

    token = create_access_token(
        identity=data["username"],
        additional_claims={"role": "ADMIN"}
    )

    return jsonify({"access_token": token}), 200
