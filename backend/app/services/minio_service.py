import io
import os
from typing import List
from flask import current_app
from bson import ObjectId
from ..extensions import get_db, get_minio
from .utils_service import serialize
from datetime import datetime

def upload_image_to_minio(file_content: bytes, filename: str, product_id: str, content_type: str):
    client = get_minio()
    if not client:
        raise Exception("MinIO not initialized")

    bucket = current_app.config["MINIO_BUCKET"]

    filename = filename.strip().replace(" ", "_")
    name, ext = os.path.splitext(filename)
    if not ext:
        ext = ".jpg"

    object_name = f"{product_id}/{name}{ext}"

    client.put_object(
        bucket_name=bucket,
        object_name=object_name,
        data=io.BytesIO(file_content),
        length=len(file_content),
        content_type=content_type,
    )

    public_domain = current_app.config.get("PUBLIC_MINIO", "localhost:9000")
    return f"http://{public_domain}/{bucket}/{object_name}"


def delete_image_from_minio(url: str) -> bool:
    client = get_minio()
    if not client:
        return False

    bucket = current_app.config["MINIO_BUCKET"]
    prefix = f"http://{current_app.config['MINIO_ENDPOINT']}/{bucket}/"

    if not url.startswith(prefix):
        return False

    object_name = url.replace(prefix, "")
    client.remove_object(bucket, object_name)
    return True


def update_product_images(product_id: str, main_image_url: str = None, image_urls: List[str] = None):
    db = get_db()
    try:
        oid = ObjectId(product_id)
    except:
        return None

    update_data = {}
    if main_image_url is not None:
        update_data["main_image_url"] = main_image_url
    if image_urls is not None:
        update_data["image_urls"] = image_urls

    update_data["updated_at"] = datetime.utcnow()

    db.products.update_one({"_id": oid, "deleted": False}, {"$set": update_data})
    return serialize(db.products.find_one({"_id": oid, "deleted": False}))
