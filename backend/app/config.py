import os
from dotenv import load_dotenv

load_dotenv()  # Load .env file


class Config:
    # -----------------------------
    # MongoDB
    # -----------------------------
    MONGO_URI = os.getenv("MONGO_URI", "mongodb://mongo:27017/warebot_db")
    MONGO_DB_NAME = os.getenv("MONGO_DB_NAME", "warebot_db")

    # -----------------------------
    # JWT
    # -----------------------------
    JWT_SECRET_KEY = os.getenv("JWT_SECRET_KEY", "SUPER_SECRET_KEY")

    # -----------------------------
    # MinIO
    # -----------------------------
    MINIO_ENDPOINT = os.getenv("MINIO_ENDPOINT", "minio:9000")
    MINIO_ACCESS_KEY = os.getenv("MINIO_ACCESS_KEY", "admin")
    MINIO_SECRET_KEY = os.getenv("MINIO_SECRET_KEY", "admin123")
    MINIO_BUCKET = os.getenv("MINIO_BUCKET", "products")
    MINIO_SECURE = os.getenv("MINIO_SECURE", "False").lower() == "true"

    # -----------------------------
    # MQTT
    # -----------------------------
    MQTT_HOST = os.getenv("MQTT_HOST", "hivemq")
    MQTT_PORT = int(os.getenv("MQTT_PORT", 1883))
    MQTT_USERNAME = os.getenv("MQTT_USERNAME")
    MQTT_PASSWORD = os.getenv("MQTT_PASSWORD")

    # -----------------------------
    # InfluxDB (Time-Series)
    # -----------------------------
    INFLUX_URL = os.getenv("INFLUX_URL", "http://influxdb:8086")
    INFLUX_TOKEN = os.getenv("INFLUX_TOKEN", "influx-token")
    INFLUX_ORG = os.getenv("INFLUX_ORG", "warebot")
    INFLUX_BUCKET = os.getenv("INFLUX_BUCKET", "telemetry")

    # -----------------------------
    # Flask debug
    # -----------------------------
    DEBUG = os.getenv("FLASK_DEBUG", "0") == "1"
