from flask import Blueprint, request, jsonify
from pydantic import ValidationError
from flask_jwt_extended import create_access_token
from .models import AdminCreate, AdminLogin
from .services import create_admin, get_admin_by_username, verify_admin_password

auth_bp = Blueprint("auth", __name__)


def handle_pydantic_error(err: ValidationError):
    return jsonify({"error": "validation_error", "details": err.errors()}), 400


@auth_bp.route("/register-admin", methods=["POST"])
def register_admin():
    """
    Register a new admin.
    NOTE: In production, you probably want to disable this endpoint
    and create admins manually/migration.
    ---
    tags:
      - Auth
    """
    try:
        data = AdminCreate(**request.json).dict()
    except ValidationError as e:
        return handle_pydantic_error(e)

    existing = get_admin_by_username(data["username"])
    if existing:
        return jsonify({"error": "username_taken"}), 400

    admin = create_admin(data["username"], data["password"])
    admin.pop("password_hash", None)
    return jsonify(admin), 201


@auth_bp.route("/login", methods=["POST"])
def admin_login():
    """
    Admin login to get JWT access token.
    ---
    tags:
      - Auth
    """
    try:
        data = AdminLogin(**request.json).dict()
    except ValidationError as e:
        return handle_pydantic_error(e)

    if not verify_admin_password(data["username"], data["password"]):
        return jsonify({"error": "invalid_credentials"}), 401

    additional_claims = {"role": "ADMIN"}
    access_token = create_access_token(
        identity=data["username"], additional_claims=additional_claims
    )
    return jsonify({"access_token": access_token})
