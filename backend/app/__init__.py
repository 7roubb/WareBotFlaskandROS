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


# =========================================================
# GLOBAL SOCKET.IO INSTANCE
# =========================================================
socketio = SocketIO(cors_allowed_origins="*")


# =========================================================
# ROBOT OFFLINE CHECKER
# =========================================================
def start_robot_offline_checker(app):
    """Background thread: auto-set robot status to OFFLINE if lost."""
    from .services import write_robot_telemetry_influx  # avoid circular import

    def checker():
        with app.app_context():
            db = get_db()

            while True:
                now = datetime.utcnow()
                robots = db.robots.find({"deleted": False})

                for r in robots:
                    last_seen = r.get("last_seen")

                    # Mark offline condition
                    if not last_seen or now - last_seen > timedelta(seconds=10):

                        # Update Mongo
                        db.robots.update_one(
                            {"_id": r["_id"]},
                            {"$set": {
                                "status": "OFFLINE",
                                "updated_at": now
                            }}
                        )

                        # Log offline to Influx
                        write_robot_telemetry_influx(r["robot_id"], {
                            "cpu_usage": float(r.get("cpu_usage") or 0),
                            "ram_usage": float(r.get("ram_usage") or 0),
                            "battery_level": float(r.get("battery_level") or 0),
                            "temperature": float(r.get("temperature") or 0),
                            "x": float(r.get("x") or 0),
                            "y": float(r.get("y") or 0),
                            "status": "OFFLINE"
                        })

                        # WebSocket broadcast
                        try:
                            socketio = app.extensions["socketio"]
                            socketio.emit("telemetry", {
                                "robot": r["robot_id"],
                                "status": "OFFLINE"
                            })
                        except:
                            pass

                time.sleep(5)

    Thread(target=checker, daemon=True).start()


# =========================================================
# APPLICATION FACTORY
# =========================================================
def create_app():
    app = Flask(__name__)
    app.config.from_object(Config)

    # Enable CORS globally
    CORS(app, resources={r"/*": {"origins": "*"}}, supports_credentials=True)

    # DB + JWT + MinIO + Influx
    init_extensions(app)

    # Initialize Socket.IO
    socketio.init_app(app)
    app.extensions["socketio"] = socketio

    # Swagger documentation
    Swagger(app)

    # Register Blueprints
    app.register_blueprint(auth_bp, url_prefix="/api/auth")
    app.register_blueprint(api_bp, url_prefix="/api")

    # Start MQTT client
    from .mqtt_client import start_mqtt_client
    from .services import get_ros2_service
    app.mqtt = start_mqtt_client(app)
    
    # Initialize ROS2 integration service with MQTT client
    ros2_service = get_ros2_service()
    ros2_service.set_mqtt_client(app.mqtt)

    # Start background offline robot checker
    start_robot_offline_checker(app)

    return app
