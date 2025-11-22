import io
import cv2
from minio import Minio
from moms_apriltag import TagGenerator2
from datetime import datetime

MAX_TAG_IDS = 587


def generate_apriltag(tag_id: int, size: int = 500) -> bytes:
    gen = TagGenerator2("tag36h11")
    tag = gen.generate(tag_id)
    tag_resized = cv2.resize(tag, (size, size), interpolation=cv2.INTER_NEAREST)
    ok, png = cv2.imencode(".png", tag_resized)
    if not ok:
        raise RuntimeError("Failed to encode AprilTag")
    return png.tobytes()


def upload_apriltag(tag_bytes: bytes, shelf_id: str):
    client = Minio("minio:9000", access_key="admin", secret_key="admin123", secure=False)
    bucket = "shelves"

    if not client.bucket_exists(bucket):
        client.make_bucket(bucket)

    fname = f"tag_{shelf_id}.png"

    client.put_object(
        bucket,
        fname,
        io.BytesIO(tag_bytes),
        length=len(tag_bytes),
        content_type="image/png",
    )

    return f"http://localhost:9000/{bucket}/{fname}"
