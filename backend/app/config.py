import os

class Config:
    # -----------------------------
    # MongoDB
    # -----------------------------
    MONGO_URI = os.environ.get("MONGO_URI", "mongodb://mongo:27017/warebot_db")
    MONGO_DB_NAME = os.environ.get("MONGO_DB_NAME", "warebot_db")

    # -----------------------------
    # JWT
    # -----------------------------
    JWT_SECRET_KEY = os.environ.get("JWT_SECRET_KEY", "SUPER_SECRET_KEY")

    # -----------------------------
    # MinIO
    # -----------------------------
    MINIO_ENDPOINT = os.environ.get("MINIO_ENDPOINT", "minio:9000")
    MINIO_ACCESS_KEY = os.environ.get("MINIO_ACCESS_KEY", "admin")
    MINIO_SECRET_KEY = os.environ.get("MINIO_SECRET_KEY", "admin123")
    MINIO_BUCKET = os.environ.get("MINIO_BUCKET", "products")

    # Convert "True"/"False" → Boolean
    MINIO_SECURE = os.environ.get("MINIO_SECURE", "False").lower() == "true"

    # -----------------------------
    # MQTT
    # -----------------------------
    MQTT_HOST = os.environ.get("MQTT_HOST", "hivemq")
    MQTT_PORT = int(os.environ.get("MQTT_PORT", 1883))
    MQTT_USERNAME = os.environ.get("MQTT_USERNAME")
    MQTT_PASSWORD = os.environ.get("MQTT_PASSWORD")

    # -----------------------------
    # InfluxDB
    # -----------------------------
    INFLUX_URL = os.environ.get("INFLUX_URL", "http://influxdb:8086")
    INFLUX_TOKEN = os.environ.get("INFLUX_TOKEN", "influx-token")
    INFLUX_ORG = os.environ.get("INFLUX_ORG", "warebot")
    INFLUX_BUCKET = os.environ.get("INFLUX_BUCKET", "telemetry")

    # -----------------------------
    # Debug
    # -----------------------------
    DEBUG = os.environ.get("FLASK_DEBUG", "0") == "1"
