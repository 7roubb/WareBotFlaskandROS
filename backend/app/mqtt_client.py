import json
import paho.mqtt.client as mqtt
from flask import current_app
from influxdb_client import Point

from .extensions import get_db, get_influx
from .services import update_robot_telemetry, update_task_status


# ====================================================================
# Helper to safely emit socket events without importing socketio
# ====================================================================
def ws_emit(event: str, data: dict):
    try:
        socketio = current_app.extensions["socketio"]
        socketio.emit(event, data)
    except Exception as e:
        current_app.logger.error(f"[WS EMIT ERROR] {e}")


mqtt_client = None
client_started = False


# ====================================================================
# MQTT CONNECTION HANDLER
# ====================================================================
def on_connect(client, userdata, flags, reason_code, properties=None):
    app = userdata["app"]
    app.logger.info(f"[MQTT] Connected with code: {reason_code}")

    client.subscribe("robots/mp400/+/status")
    client.subscribe("robots/mp400/+/task_status")
    client.subscribe("warehouse/map")

    app.logger.info("[MQTT] Subscribed to all robot + map topics")


# ====================================================================
# MQTT MESSAGE HANDLER
# ====================================================================
def on_message(client, userdata, msg):
    app = userdata["app"]

    with app.app_context():

        topic = msg.topic
        payload = msg.payload.decode("utf-8")

        db = get_db()
        influx_client, write_api = get_influx()

        # -----------------------------------------------------------
        # TELEMETRY
        # -----------------------------------------------------------
        if topic.startswith("robots/mp400/") and topic.endswith("/status"):
            try:
                robot_name = topic.split("/")[2]
                data = json.loads(payload)

                telemetry = {
                    "cpu_usage": data.get("cpu_usage"),
                    "ram_usage": data.get("ram_usage"),
                    "battery_level": data.get("battery_level"),
                    "temperature": data.get("temperature"),
                    "x": data.get("x"),
                    "y": data.get("y"),
                    "status": data.get("status", "IDLE"),
                }

                # Update Mongo snapshot
                update_robot_telemetry(robot_name, telemetry)

                # Store historical InfluxDB
                point = (
                    Point("robot_telemetry")
                    .tag("robot", robot_name)
                    .field("cpu_usage", telemetry["cpu_usage"])
                    .field("ram_usage", telemetry["ram_usage"])
                    .field("battery_level", telemetry["battery_level"])
                    .field("temperature", telemetry["temperature"])
                    .field("x", telemetry["x"])
                    .field("y", telemetry["y"])
                    .field("status_code", status_to_code(telemetry["status"]))
                )

                write_api.write(
                    bucket=current_app.config["INFLUX_BUCKET"],
                    record=point
                )

                # Emit WebSocket
                ws_emit("telemetry", {
                    "robot": robot_name,
                    **telemetry
                })

            except Exception as e:
                app.logger.error(f"[MQTT Telemetry Error] {e}")

        # -----------------------------------------------------------
        # TASK STATUS
        # -----------------------------------------------------------
        elif topic.startswith("robots/mp400/") and topic.endswith("/task_status"):
            try:
                data = json.loads(payload)
                task_id = data.get("task_id")
                status = data.get("status")

                if task_id and status:
                    update_task_status(task_id, status)
                    ws_emit("task_status", data)

            except Exception as e:
                app.logger.error(f"[MQTT Task Status Error] {e}")

        # -----------------------------------------------------------
        # MERGED MAP
        # -----------------------------------------------------------
        elif topic == "warehouse/map":
            try:
                data = json.loads(payload)

                db.maps.update_one(
                    {"name": "merged_map"},
                    {"$set": data},
                    upsert=True
                )

                ws_emit("map_update", data)

            except Exception as e:
                app.logger.error(f"[MQTT Map Update Error] {e}")

        # -----------------------------------------------------------
        # OTHER CUSTOM TOPICS
        # -----------------------------------------------------------
        else:
            try:
                robot = db.robots.find_one({"topic": topic, "deleted": False})

                if robot:
                    ws_emit("robot_custom", {
                        "topic": topic,
                        "payload": payload
                    })

            except Exception as e:
                app.logger.error(f"[MQTT Custom Topic Error] {e}")


# ====================================================================
# START MQTT CLIENT
# ====================================================================
def start_mqtt_client(app):
    global mqtt_client, client_started

    if client_started:
        return mqtt_client

    mqtt_client = mqtt.Client(
        client_id="warebot_backend",
        userdata={"app": app},
        protocol=mqtt.MQTTv5
    )

    mqtt_client.on_connect = on_connect
    mqtt_client.on_message = on_message

    # MQTT auth (optional)
    username = app.config.get("MQTT_USERNAME")
    password = app.config.get("MQTT_PASSWORD")

    if username:
        mqtt_client.username_pw_set(username, password)

    mqtt_client.connect(
        app.config["MQTT_HOST"],
        int(app.config["MQTT_PORT"]),
        keepalive=60
    )

    mqtt_client.loop_start()
    client_started = True

    app.logger.info("[MQTT] Client started successfully")
    return mqtt_client


# ====================================================================
# Convert status to numeric
# ====================================================================
def status_to_code(status: str) -> int:
    s = status.upper()
    if s == "IDLE": return 0
    if s == "BUSY": return 1
    if s == "ERROR": return 2
    if s == "OFFLINE": return 3
    return -1
