from flask import Flask
from flasgger import Swagger
from flask_cors import CORS
from flask_socketio import SocketIO

from .config import Config
from .extensions import init_extensions
from .routes import api_bp
from .auth_routes import auth_bp

# ---------------------------------------------------------
# Create the SocketIO instance BEFORE importing mqtt_client
# ---------------------------------------------------------
socketio = SocketIO(cors_allowed_origins="*")


def create_app():
    app = Flask(__name__)
    app.config.from_object(Config)

    # CORS
    CORS(app, resources={r"/api/*": {"origins": "*"}})

    # Initialize DB | JWT | MinIO | Influx
    init_extensions(app)

    # Init socketio with app
    socketio.init_app(app)

    # Swagger UI
    Swagger(app)

    # Register REST APIs
    app.register_blueprint(auth_bp, url_prefix="/api/auth")
    app.register_blueprint(api_bp, url_prefix="/api")

    # ---------------------------------------------------------
    # Import mqtt_client *AFTER* socketio is created
    # ---------------------------------------------------------
    from .mqtt_client import start_mqtt_client
    app.mqtt = start_mqtt_client(app)

    return app
