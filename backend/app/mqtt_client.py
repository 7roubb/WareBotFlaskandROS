import json
import time
import paho.mqtt.client as mqtt
from flask import current_app
from .extensions import get_db
from .models import RobotTelemetry
from .services import (
    update_robot_telemetry,
    update_task_status
)

mqtt_client = None   # Global client (يُستخدم في Flask app)
client_started = False


# ---------------------------------------------------
# CONNECT CALLBACK
# ---------------------------------------------------
def on_connect(client, userdata, flags, reason_code, properties=None):
    app = userdata["app"]

    app.logger.info(f"[MQTT] Connected with code: {reason_code}")

    # Subscribe to robot telemetry
    client.subscribe("robots/mp400/+/status")

    # Subscribe to task status updates
    client.subscribe("robots/mp400/+/task_status")

    # Subscribe to merged map
    client.subscribe("warehouse/map")

    app.logger.info("[MQTT] Subscribed to all robot/map topics")


# ---------------------------------------------------
# MESSAGE CALLBACK
# ---------------------------------------------------
def on_message(client, userdata, msg):
    app = userdata["app"]
    topic = msg.topic
    payload_str = msg.payload.decode("utf-8")

    app.logger.debug(f"[MQTT] Message → {topic}: {payload_str}")

    db = get_db()

    # -----------------------------------------------
    # ROBOT TELEMETRY
    # robots/mp400/robot1/status
    # -----------------------------------------------
    if topic.startswith("robots/mp400/") and topic.endswith("/status"):
        try:
            robot_name = topic.split("/")[2]
            data = json.loads(payload_str)

            telemetry = {
                "cpu_usage": data.get("cpu_usage"),
                "ram_usage": data.get("ram_usage"),
                "battery_level": data.get("battery_level"),
                "temperature": data.get("temperature"),
                "x": data.get("x"),
                "y": data.get("y"),
                "status": data.get("status", "IDLE")
            }

            update_robot_telemetry(robot_name, telemetry)

        except Exception as e:
            app.logger.error(f"[MQTT] Telemetry parsing error: {e}")

    # -----------------------------------------------
    # TASK STATUS
    # robots/mp400/robot1/task_status
    # -----------------------------------------------
    elif topic.startswith("robots/mp400/") and topic.endswith("/task_status"):
        try:
            data = json.loads(payload_str)
            task_id = data.get("task_id")
            status = data.get("status")

            if task_id and status:
                update_task_status(task_id, status)

        except Exception as e:
            app.logger.error(f"[MQTT] Task status error: {e}")

    # -----------------------------------------------
    # MAP MERGE
    # warehouse/map
    # -----------------------------------------------
    elif topic == "warehouse/map":
        try:
            data = json.loads(payload_str)
            db.maps.update_one(
                {"name": "merged_map"},
                {"$set": data},
                upsert=True,
            )
        except Exception as e:
            app.logger.error(f"[MQTT] Map merge error: {e}")


# ---------------------------------------------------
# START MQTT CLIENT
# ---------------------------------------------------
def start_mqtt_client(app):
    global mqtt_client, client_started

    if client_started:
        return

    mqtt_client = mqtt.Client(
        client_id="warebot-backend",
        userdata={"app": app},
        protocol=mqtt.MQTTv5
    )

    mqtt_client.on_connect = on_connect
    mqtt_client.on_message = on_message

    # Auth if provided
    username = app.config.get("MQTT_USERNAME")
    password = app.config.get("MQTT_PASSWORD")
    if username:
        mqtt_client.username_pw_set(username, password)

    # Connect to HiveMQ
    mqtt_client.connect(
        app.config["MQTT_HOST"],
        app.config["MQTT_PORT"],
        keepalive=60
    )

    mqtt_client.loop_start()
    client_started = True

    app.mqtt = mqtt_client

    app.logger.info("[MQTT] Client initialized & loop started")


# ---------------------------------------------------
# PUBLISH WRAPPER
# ---------------------------------------------------
def publish_message(topic: str, payload: str):
    if mqtt_client is None:
        print("[MQTT] ERROR: client not started")
        return False

    mqtt_client.publish(topic, payload)
    print(f"[MQTT] Published → {topic}: {payload}")
    return True
