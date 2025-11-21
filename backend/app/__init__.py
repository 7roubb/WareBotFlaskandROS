from flask import Flask
from flasgger import Swagger
from flask_cors import CORS
from .config import Config
from .extensions import init_extensions
from .routes import api_bp
from .auth_routes import auth_bp
from .mqtt_client import start_mqtt_client

from flask_socketio import SocketIO

socketio = SocketIO(cors_allowed_origins="*")   # NEW


def create_app():
    app = Flask(__name__)
    app.config.from_object(Config)

    # CORS
    CORS(app, resources={r"/api/*": {"origins": "*"}})

    # Init DB, JWT, Minio, Influx
    init_extensions(app)

    # Attach socket to app
    socketio.init_app(app)   # NEW

    # Swagger
    Swagger(app)

    # Routes
    app.register_blueprint(auth_bp, url_prefix="/api/auth")
    app.register_blueprint(api_bp, url_prefix="/api")

    # MQTT
    app.mqtt = start_mqtt_client(app)

    return app
