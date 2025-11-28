from flask import current_app
from pymongo import MongoClient
from flask_jwt_extended import JWTManager
from minio import Minio
from influxdb_client import InfluxDBClient
from influxdb_client.client.write_api import SYNCHRONOUS
from flask_socketio import SocketIO
from bson import ObjectId
from datetime import datetime
from flask.json.provider import DefaultJSONProvider

mongo_client = None
mongo_db = None
jwt = JWTManager()
minio_client = None
influx_client = None
influx_write_api = None
socketio = SocketIO()


class CustomJSONProvider(DefaultJSONProvider):
    """Custom JSON provider for MongoDB ObjectId and datetime"""
    def default(self, o):
        if isinstance(o, ObjectId):
            return str(o)
        if isinstance(o, datetime):
            return o.isoformat()
        return super().default(o)


def init_extensions(app):
    global mongo_client, mongo_db
    global minio_client, influx_client, influx_write_api, socketio

    # Set custom JSON provider for Flask 3.0+
    app.json_provider_class = CustomJSONProvider
    app.json = CustomJSONProvider(app)

    # ============================
    # MongoDB
    # ============================
    mongo_client = MongoClient(app.config["MONGO_URI"])
    mongo_db = mongo_client[app.config["MONGO_DB_NAME"]]

    # ============================
    # JWT
    # ============================
    jwt.init_app(app)

    # ============================
    # MinIO
    # ============================
    print(">>> Initializing MinIO...")

    minio_client = Minio(
        endpoint=app.config["MINIO_ENDPOINT"],
        access_key=app.config["MINIO_ACCESS_KEY"],
        secret_key=app.config["MINIO_SECRET_KEY"],
        secure=app.config["MINIO_SECURE"],
    )

    bucket = app.config["MINIO_BUCKET"]

    if not minio_client.bucket_exists(bucket):
        minio_client.make_bucket(bucket)

    print(f">>> MinIO initialized: bucket={bucket}")

    # ============================
    # InfluxDB
    # ============================
    print(">>> Initializing InfluxDB...")

    influx_client_local = InfluxDBClient(
        url=app.config["INFLUX_URL"],
        token=app.config["INFLUX_TOKEN"],
        org=app.config["INFLUX_ORG"],
    )

    influx_write_api_local = influx_client_local.write_api(write_options=SYNCHRONOUS)

    influx_client = influx_client_local
    influx_write_api = influx_write_api_local

    print(">>> InfluxDB initialized.")

    try:
        socketio.init_app(app, cors_allowed_origins="*")
        try:
            app.extensions["socketio"] = socketio
        except Exception:
            app.extensions.update({"socketio": socketio})
        print(">>> SocketIO initialized.")
    except Exception as e:
        print(f">>> SocketIO init failed: {e}")


def get_db():
    global mongo_db
    return mongo_db


def get_influx():
    global influx_client, influx_write_api
    return influx_client, influx_write_api


def get_minio():
    """Return current MinIO client instance"""
    global minio_client
    return minio_client
