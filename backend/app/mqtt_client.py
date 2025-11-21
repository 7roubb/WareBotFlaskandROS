import json
import paho.mqtt.client as mqtt
from flask import current_app
from .extensions import get_db
from .services import update_robot_telemetry, update_task_status

mqtt_client = None
client_started = False

# ===========================================
# MQTT CALLBACKS
# ===========================================
def on_connect(client, userdata, flags, reason_code, properties=None):
    app = userdata["app"]
    app.logger.info(f"[MQTT] Connected with code: {reason_code}")

    # Subscriptions
    client.subscribe("robots/mp400/+/status")
    client.subscribe("robots/mp400/+/task_status")
    client.subscribe("warehouse/map")

    app.logger.info("[MQTT] Subscribed to robot and map topics")


def on_message(client, userdata, msg):
    app = userdata["app"]
    topic = msg.topic
    payload_str = msg.payload.decode("utf-8")

    db = get_db()
    app.logger.debug(f"[MQTT] Message → {topic}: {payload_str}")

    # ----------------------------
    # ROBOT TELEMETRY
    # ----------------------------
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
            app.logger.error(f"[MQTT] Telemetry error: {e}")

    # ----------------------------
    # TASK STATUS
    # ----------------------------
    elif topic.startswith("robots/mp400/") and topic.endswith("/task_status"):
        try:
            data = json.loads(payload_str)
            if data.get("task_id") and data.get("status"):
                update_task_status(data["task_id"], data["status"])
        except Exception as e:
            app.logger.error(f"[MQTT] Task error: {e}")

    # ----------------------------
    # MERGED MAP
    # ----------------------------
    elif topic == "warehouse/map":
        try:
            data = json.loads(payload_str)
            db.maps.update_one(
                {"name": "merged_map"},
                {"$set": data},
                upsert=True
            )
        except Exception as e:
            app.logger.error(f"[MQTT] Map error: {e}")

    # ----------------------------
    # CUSTOM ROBOT TOPICS
    # ----------------------------
    else:
        robot = db.robots.find_one({"topic": topic, "deleted": False})
        if robot:
            try:
                data = json.loads(payload_str)
                app.logger.info(f"[MQTT] Custom topic {topic}: {data}")
            except Exception as e:
                app.logger.error(f"[MQTT] Custom topic error: {e}")


# ===========================================
# START MQTT CLIENT (THIS WAS MISSING!)
# ===========================================
def start_mqtt_client(app):
    global mqtt_client, client_started

    if client_started:
        return mqtt_client

    mqtt_client = mqtt.Client(
        client_id="warebot_backend",
        userdata={"app": app},
        protocol=mqtt.MQTTv5
    )

    # Set callbacks
    mqtt_client.on_connect = on_connect
    mqtt_client.on_message = on_message

    # Auth
    username = app.config.get("MQTT_USERNAME")
    password = app.config.get("MQTT_PASSWORD")
    if username:
        mqtt_client.username_pw_set(username, password)

    # Connect
    mqtt_client.connect(
        app.config["MQTT_HOST"],
        int(app.config["MQTT_PORT"]),
        keepalive=60
    )

    mqtt_client.loop_start()
    client_started = True

    app.logger.info("[MQTT] Client started")
    return mqtt_client


# ===========================================
# PUBLISH WRAPPER
# ===========================================
def publish_message(topic, payload):
    if not mqtt_client:
        print("[MQTT] ERROR: MQTT client not running")
        return False

    mqtt_client.publish(topic, payload)
    return True
