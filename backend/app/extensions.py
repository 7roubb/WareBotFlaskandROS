from flask import current_app
from pymongo import MongoClient
from flask_jwt_extended import JWTManager
from minio import Minio
from influxdb_client import InfluxDBClient, WriteOptions

# Global references
mongo_client = None
mongo_db = None
jwt = JWTManager()
minio_client = None
influx_client = None
influx_write_api = None


# =========================================================
# INIT ALL EXTENSIONS
# =========================================================
def init_extensions(app):
    global mongo_client, mongo_db, minio_client
    global influx_client, influx_write_api

    # -----------------------------------------------------
    # MongoDB
    # -----------------------------------------------------
    mongo_client = MongoClient(app.config["MONGO_URI"])
    mongo_db = mongo_client[app.config["MONGO_DB_NAME"]]

    # -----------------------------------------------------
    # JWT
    # -----------------------------------------------------
    jwt.init_app(app)

    # -----------------------------------------------------
    # MinIO
    # -----------------------------------------------------
    minio_client = Minio(
        app.config["MINIO_ENDPOINT"],
        access_key=app.config["MINIO_ACCESS_KEY"],
        secret_key=app.config["MINIO_SECRET_KEY"],
        secure=app.config["MINIO_SECURE"],
    )

    bucket = app.config["MINIO_BUCKET"]
    if not minio_client.bucket_exists(bucket):
        minio_client.make_bucket(bucket)

    # -----------------------------------------------------
    # InfluxDB (Time-Series)
    # -----------------------------------------------------
    influx_client = InfluxDBClient(
        url=app.config["INFLUX_URL"],
        token=app.config["INFLUX_TOKEN"],
        org=app.config["INFLUX_ORG"],
    )

    influx_write_api = influx_client.write_api(
        write_options=WriteOptions(
            batch_size=1,     # write immediately
            flush_interval=0
        )
    )


# =========================================================
# HELPERS
# =========================================================

def get_db():
    """Return MongoDB database instance"""
    global mongo_db
    if mongo_db is None:
        client = MongoClient(current_app.config["MONGO_URI"])
        mongo_db = client[current_app.config["MONGO_DB_NAME"]]
    return mongo_db


def get_influx():
    """Return InfluxDB write API"""
    global influx_client, influx_write_api
    return influx_client, influx_write_api
