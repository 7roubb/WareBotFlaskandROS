from flask import Blueprint, request, jsonify
from flask_jwt_extended import jwt_required, get_jwt
from flask_cors import cross_origin
from pydantic import ValidationError

from ..services import (
    upload_image_to_minio,
    delete_image_from_minio,
    update_product_images,
    get_product,
)

images_bp = Blueprint("images", __name__)


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


# =========================================================
# UPLOAD PRODUCT IMAGE
# =========================================================
@images_bp.route("/<product_id>/images", methods=["POST", "OPTIONS"])
@cross_origin()
@admin_required
def upload_product_image_route(product_id):

    # Handle CORS preflight
    if request.method == "OPTIONS":
        return jsonify({}), 200

    if "image" not in request.files:
        return {"error": "no_image_uploaded"}, 400

    file = request.files["image"]
    if not file.filename:
        return {"error": "empty_filename"}, 400

    content = file.read()

    # Upload to MinIO
    url = upload_image_to_minio(
        file_content=content,
        filename=file.filename,
        product_id=product_id,
        content_type=file.mimetype
    )

    # Update product images in DB
    product = get_product(product_id)
    if not product:
        return {"error": "product_not_found"}, 404

    images = product.get("image_urls", [])
    images.append(url)

    updated = update_product_images(product_id, image_urls=images)

    return jsonify({
        "status": "uploaded",
        "image_url": url,
        "product": updated
    })


# =========================================================
# DELETE IMAGE BY INDEX
# =========================================================
@images_bp.route("/<product_id>/images/<int:index>", methods=["DELETE"])
@admin_required
def delete_product_image_route(product_id, index):
    product = get_product(product_id)
    if not product:
        return {"error": "product_not_found"}, 404

    images = product.get("image_urls", [])
    if index < 0 or index >= len(images):
        return {"error": "invalid_index"}, 400

    # Remove from MinIO
    delete_image_from_minio(images[index])

    # Remove from DB
    images.pop(index)
    updated = update_product_images(product_id, image_urls=images)

    return jsonify({
        "status": "deleted",
        "product": updated
    })


# =========================================================
# SET MAIN IMAGE
# =========================================================
@images_bp.route("/<product_id>/images/set-main", methods=["PUT"])
@admin_required
def set_main_image_route(product_id):
    url = request.json.get("image_url")
    if not url:
        return {"error": "image_url_required"}, 400

    updated = update_product_images(product_id, main_image_url=url)
    return jsonify(updated)


# =========================================================
# GET ALL IMAGES FOR A PRODUCT
# =========================================================
@images_bp.route("/<product_id>/images", methods=["GET"])
def get_product_images_route(product_id):
    product = get_product(product_id)
    if not product:
        return {"error": "not_found"}, 404

    return {
        "product_id": product_id,
        "main_image_url": product.get("main_image_url"),
        "image_urls": product.get("image_urls", [])
    }
