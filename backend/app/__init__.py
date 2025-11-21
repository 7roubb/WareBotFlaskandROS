from flask import Flask
from flasgger import Swagger
from flask_cors import CORS

from .config import Config
from .extensions import init_extensions
from .routes import api_bp
from .auth_routes import auth_bp
from .mqtt_client import start_mqtt_client

def create_app():
    app = Flask(__name__)
    app.config.from_object(Config)

    # Allow React / Bolt frontend
    CORS(app, resources={r"/api/*": {"origins": "*"}})

    # Initialize MongoDB, JWT, MinIO
    init_extensions(app)

    # Swagger Docs
    Swagger(app)

    # Register routes
    app.register_blueprint(auth_bp, url_prefix="/api/auth")
    app.register_blueprint(api_bp, url_prefix="/api")

    # Start MQTT listener and attach it to app
    app.mqtt = start_mqtt_client(app)

    return app
