from flask import current_app
from pymongo import MongoClient
from flask_jwt_extended import JWTManager
from minio import Minio


mongo_client = None
mongo_db = None
jwt = JWTManager()
minio_client = None


def init_extensions(app):
    global mongo_client, mongo_db
    mongo_client = MongoClient(app.config["MONGO_URI"])
    mongo_db = mongo_client[app.config["MONGO_DB_NAME"]]

    # MinIO
    minio_client = Minio(
        app.config["MINIO_ENDPOINT"],
        access_key=app.config["MINIO_ACCESS_KEY"],
        secret_key=app.config["MINIO_SECRET_KEY"],
        secure=app.config["MINIO_SECURE"]
    )

    # Bucket check
    if not minio_client.bucket_exists(app.config["MINIO_BUCKET"]):
        minio_client.make_bucket(app.config["MINIO_BUCKET"])
    # Init JWT
    jwt.init_app(app)


def get_db():
    """
    Helper to access Mongo DB instance.
    """
    global mongo_db
    if mongo_db is None:
        mongo_db = MongoClient(current_app.config["MONGO_URI"])[
            current_app.config["MONGO_DB_NAME"]
        ]
    return mongo_db
