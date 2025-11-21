from flask import Flask
from flasgger import Swagger
from flask_cors import CORS
from flask_socketio import SocketIO
from threading import Thread
from datetime import datetime, timedelta
import time

from .config import Config
from .extensions import init_extensions, get_db
from .routes import api_bp
from .auth_routes import auth_bp

# Create socketio
socketio = SocketIO(cors_allowed_origins="*")


def start_robot_offline_checker(app):
    def checker():
        with app.app_context():
            db = get_db()
            while True:
                now = datetime.utcnow()
                robots = db.robots.find({"deleted": False})

                for r in robots:
                    last_seen = r.get("last_seen")

                    # First time or expired
                    if not last_seen or now - last_seen > timedelta(seconds=10):
                        db.robots.update_one(
                            {"_id": r["_id"]},
                            {"$set": {"status": "OFFLINE"}}
                        )

                        # Emit WS update
                        socketio.emit("telemetry", {
                            "robot": r["robot_id"],
                            "status": "OFFLINE"
                        })

                time.sleep(5)

    Thread(target=checker, daemon=True).start()


def create_app():
    app = Flask(__name__)
    app.config.from_object(Config)

    CORS(app, resources={r"/api/*": {"origins": "*"}})

    init_extensions(app)

    # Register socketio inside app (VERY IMPORTANT)
    socketio.init_app(app)
    app.extensions["socketio"] = socketio

    Swagger(app)

    app.register_blueprint(auth_bp, url_prefix="/api/auth")
    app.register_blueprint(api_bp, url_prefix="/api")

    # Import mqtt AFTER socketio is ready
    from .mqtt_client import start_mqtt_client
    app.mqtt = start_mqtt_client(app)

    start_robot_offline_checker(app)

    return app
