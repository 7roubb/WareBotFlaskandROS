import json
import paho.mqtt.client as mqtt
from flask import current_app

from influxdb_client import Point
from .extensions import get_db, get_influx
from .services import update_robot_telemetry, update_task_status

# 🔥 WebSocket
from backend import socketio

mqtt_client = None
client_started = False


# =========================================================
# MQTT CALLBACKS
# =========================================================
def on_connect(client, userdata, flags, reason_code, properties=None):
    app = userdata["app"]
    app.logger.info(f"[MQTT] Connected with code: {reason_code}")

    # Subscribe to robot telemetry + task updates
    client.subscribe("robots/mp400/+/status")
    client.subscribe("robots/mp400/+/task_status")
    client.subscribe("warehouse/map")

    app.logger.info("[MQTT] Subscribed to topics")


def on_message(client, userdata, msg):
    app = userdata["app"]

    # VERY IMPORTANT FIX — activate Flask context
    with app.app_context():

        topic = msg.topic
        payload = msg.payload.decode("utf-8")

        db = get_db()
        influx_client, write_api = get_influx()

        app.logger.debug(f"[MQTT] → {topic}: {payload}")

        # -----------------------------------------------------
        # TELEMETRY: robots/mp400/<robot>/status
        # -----------------------------------------------------
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

                # Update Mongo Live Snapshot
                update_robot_telemetry(robot_name, telemetry)

                # Write History to InfluxDB
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

                # -----------------------------------------------------
                # 🔥 REAL-TIME WEB SOCKET EMIT
                # -----------------------------------------------------
                socketio.emit("telemetry", {
                    "robot": robot_name,
                    "cpu": telemetry["cpu_usage"],
                    "ram": telemetry["ram_usage"],
                    "battery": telemetry["battery_level"],
                    "temperature": telemetry["temperature"],
                    "x": telemetry["x"],
                    "y": telemetry["y"],
                    "status": telemetry["status"],
                })

            except Exception as e:
                app.logger.error(f"[MQTT] Telemetry error: {e}")

        # -----------------------------------------------------
        # TASK STATUS UPDATE
        # robots/mp400/<robot>/task_status
        # -----------------------------------------------------
        elif topic.startswith("robots/mp400/") and topic.endswith("/task_status"):
            try:
                data = json.loads(payload)
                task_id = data.get("task_id")
                status = data.get("status")

                if task_id and status:
                    update_task_status(task_id, status)

                    # 🔥 Send real-time task status
                    socketio.emit("task_status", {
                        "task_id": task_id,
                        "status": status
                    })

            except Exception as e:
                app.logger.error(f"[MQTT] Task status error: {e}")

        # -----------------------------------------------------
        # MERGED MAP (warehouse/map)
        # -----------------------------------------------------
        elif topic == "warehouse/map":
            try:
                data = json.loads(payload)
                db.maps.update_one(
                    {"name": "merged_map"},
                    {"$set": data},
                    upsert=True
                )
                app.logger.info("[MQTT] Merged map stored to DB")

                # 🔥 Real-time map broadcast
                socketio.emit("map_update", data)

            except Exception as e:
                app.logger.error(f"[MQTT] Map update error: {e}")

        # -----------------------------------------------------
        # CUSTOM TOPICS
        # -----------------------------------------------------
        else:
            try:
                robot = db.robots.find_one({"topic": topic, "deleted": False})
                if robot:
                    app.logger.info(f"[MQTT] Custom topic {topic}: {payload}")

                    # Optionally send this to WebSocket too
                    socketio.emit("robot_custom", {
                        "topic": topic,
                        "payload": payload
                    })

            except Exception as e:
                app.logger.error(f"[MQTT] Custom topic error: {e}")


# =========================================================
# START MQTT CLIENT
# =========================================================
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

    # Login if enabled
    username = app.config.get("MQTT_USERNAME")
    password = app.config.get("MQTT_PASSWORD")
    if username:
        mqtt_client.username_pw_set(username, password)

    # Connect to broker
    mqtt_client.connect(
        app.config["MQTT_HOST"],
        int(app.config["MQTT_PORT"]),
        keepalive=60
    )

    mqtt_client.loop_start()
    client_started = True

    app.logger.info("[MQTT] Client started")
    return mqtt_client


# =========================================================
# HELPERS
# =========================================================
def publish_message(topic, payload):
    """Publish JSON payload to a topic"""
    if not mqtt_client:
        print("[MQTT] ERROR: Client not running")
        return False

    mqtt_client.publish(topic, payload)
    return True


def status_to_code(status: str) -> int:
    """Convert status string → numeric for InfluxDB graphs"""
    s = status.upper()
    if s == "IDLE":
        return 0
    if s == "BUSY":
        return 1
    if s == "ERROR":
        return 2
    if s == "OFFLINE":
        return 3
    return -1
