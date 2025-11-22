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

# Global SocketIO instance
socketio = SocketIO(cors_allowed_origins="*")


# =========================================================
# ROBOT OFFLINE CHECKER (Writes OFFLINE → Mongo + Influx)
# =========================================================
def start_robot_offline_checker(app):
    from .services import write_robot_telemetry_influx  # avoid circular import

    def checker():
        with app.app_context():
            db = get_db()

            while True:
                now = datetime.utcnow()
                robots = db.robots.find({"deleted": False})

                for r in robots:
                    last_seen = r.get("last_seen")

                    # If robot is offline
                    if not last_seen or now - last_seen > timedelta(seconds=10):

                        # Update Mongo snapshot
                        db.robots.update_one(
                            {"_id": r["_id"]},
                            {"$set": {
                                "status": "OFFLINE",
                                "updated_at": now
                            }}
                        )

                        # Write OFFLINE state into InfluxDB (as INT)
                        write_robot_telemetry_influx(r["robot_id"], {
                            "cpu_usage": int(r.get("cpu_usage") or 0),
                            "ram_usage": int(r.get("ram_usage") or 0),
                            "battery_level": int(r.get("battery_level") or 0),
                            "temperature": int(r.get("temperature") or 0),
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

                time.sleep(5)  # run checker every 5 seconds

    Thread(target=checker, daemon=True).start()

# =========================================================
# FLASK APPLICATION FACTORY
# =========================================================
def create_app():
    app = Flask(__name__)
    app.config.from_object(Config)

    # CORS for all routes
    CORS(app, resources={r"/*": {"origins": "*"}}, supports_credentials=True)

    # init DB + JWT + MinIO + Influx
    init_extensions(app)

    # init Socket.IO
    socketio.init_app(app)
    app.extensions["socketio"] = socketio

    # Swagger docs
    Swagger(app)

    # API routes
    app.register_blueprint(auth_bp, url_prefix="/api/auth")
    app.register_blueprint(api_bp, url_prefix="/api")

    # MQTT Client startup
    from .mqtt_client import start_mqtt_client
    app.mqtt = start_mqtt_client(app)

    # Start offline robot tracker
    start_robot_offline_checker(app)

    return app
