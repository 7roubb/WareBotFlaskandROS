import os
from datetime import timedelta

class Config:
    # MongoDB
    MONGO_URI = os.getenv("MONGO_URI", "mongodb://mongo:27017/warebot_db")
    MONGO_DB_NAME = os.getenv("MONGO_DB_NAME", "warebot_db")

    # JWT
    JWT_SECRET_KEY = os.getenv("JWT_SECRET_KEY", "CHANGE_ME_SECRET")
    JWT_ACCESS_TOKEN_EXPIRES = timedelta(hours=2)

    # Swagger
    SWAGGER = {
        "title": "Warebot API",
        "uiversion": 3,
        "openapi": "3.0.2",
    }

    # MinIO (S3-Compatible Storage)
    MINIO_ENDPOINT = os.getenv("MINIO_ENDPOINT", "minio:9000")
    MINIO_ACCESS_KEY = os.getenv("MINIO_ACCESS_KEY", "admin")
    MINIO_SECRET_KEY = os.getenv("MINIO_SECRET_KEY", "admin123")
    MINIO_BUCKET = os.getenv("MINIO_BUCKET", "products")
    MINIO_SECURE = False  # inside Docker network, so not https

    # MQTT / HiveMQ
    MQTT_HOST = os.getenv("MQTT_HOST", "hivemq")
    MQTT_PORT = int(os.getenv("MQTT_PORT", 1883))
    MQTT_USERNAME = os.getenv("MQTT_USERNAME", "")
    MQTT_PASSWORD = os.getenv("MQTT_PASSWORD", "")
    MQTT_STATUS_TOPIC = os.getenv("MQTT_STATUS_TOPIC", "robots/mp400/+/status")

    # Debug
    DEBUG = os.getenv("FLASK_DEBUG", "false").lower() == "true"
