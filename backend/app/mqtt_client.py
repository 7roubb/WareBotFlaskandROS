import json
import time
import paho.mqtt.client as mqtt
from flask import current_app
from influxdb_client import Point
from datetime import datetime

from .extensions import get_db, get_influx
from .services import update_robot_telemetry, update_task_status


def ws_emit(event: str, data: dict):
    """Emit WebSocket event if socketio is available."""
    try:
        socketio = current_app.extensions["socketio"]
        # Log the outgoing websocket event for easier debugging
        try:
            current_app.logger.debug(f"[WS EMIT] event={event} payload={data}")
        except Exception:
            # ignore logging errors
            pass
        socketio.emit(event, data)
    except Exception as e:
        current_app.logger.error(f"[WS EMIT ERROR] {e}")


def ws_emit_to_room(event: str, data: dict, room: str):
    """Emit WebSocket event to a specific room (live update subscribers)"""
    try:
        socketio = current_app.extensions["socketio"]
        socketio.emit(event, data, room=room)
    except Exception as e:
        current_app.logger.error(f"[WS EMIT ROOM ERROR] {e}")


mqtt_client = None
client_started = False


# =========================================================
# MQTT CONNECT
# =========================================================
def on_connect(client, userdata, flags, reason_code, properties=None):
    app = userdata["app"]
    app.logger.info(f"[MQTT] Connected with code: {reason_code}")

    # Telemetry + tasks + merged map + shelf location updates
    client.subscribe("robots/mp400/+/status")
    client.subscribe("robots/mp400/+/task_status")
    
    # ✅ CRITICAL: Subscribe to task status from NEW task runner
    client.subscribe("robot/+/task/status")
    
    client.subscribe("robot/+/shelf/location")  # Real-time shelf location updates
    client.subscribe("robot/+/position/update")  # Robot position updates
    client.subscribe("robot/+/task/progress")  # Task progress tracking
    client.subscribe("warehouse/map")

    app.logger.info("[MQTT] Subscribed to all robot + map + shelf location + position + task progress topics")


# =========================================================
# MQTT MESSAGE HANDLER
# =========================================================
def on_message(client, userdata, msg):
    app = userdata["app"]

    with app.app_context():
        topic = msg.topic
        payload = msg.payload.decode("utf-8")

        db = get_db()
        influx_client, write_api = get_influx()

        # =============================
        # TELEMETRY: robots/mp400/<robot>/status
        # =============================
        if topic.startswith("robots/mp400/") and topic.endswith("/status"):
            try:
                robot_name = topic.split("/")[2]
                data = json.loads(payload)

                telemetry = {
                    "cpu_usage": data.get("cpu_usage", 0.0),
                    "ram_usage": data.get("ram_usage", 0.0),
                    "battery_level": data.get("battery_level", 0.0),
                    "temperature": data.get("temperature", 0.0),
                    "x": data.get("x", 0.0),
                    "y": data.get("y", 0.0),
                    "yaw": data.get("yaw", 0.0),  # Robot orientation in radians
                    "status": data.get("status", "IDLE"),
                }

                # ✓ CRITICAL: Update BOTH telemetry AND position in MongoDB
                update_robot_telemetry(robot_name, telemetry)

                # Attempt to read the robot document to include canonical IDs in the emit
                try:
                    robot_doc = db.robots.find_one({"robot_id": robot_name, "deleted": False})
                except Exception:
                    robot_doc = None

                # Time-series in InfluxDB (if configured)
                if influx_client is not None and write_api is not None:
                    try:
                        point = (
                            Point("robot_telemetry")
                            .tag("robot", robot_name)
                            .field("cpu_usage", float(telemetry["cpu_usage"]))
                            .field("ram_usage", float(telemetry["ram_usage"]))
                            .field("battery_level", float(telemetry["battery_level"]))
                            .field("temperature", float(telemetry["temperature"]))
                            .field("x", float(telemetry["x"]))
                            .field("y", float(telemetry["y"]))
                            .field("yaw", float(telemetry["yaw"]))  # Robot orientation
                            .field("status_code", status_to_code(telemetry["status"]))
                        )

                        write_api.write(
                            bucket=current_app.config["INFLUX_BUCKET"],
                            org=current_app.config["INFLUX_ORG"],
                            record=point
                        )
                    except Exception as e:
                        app.logger.error(f"[InfluxDB Write Error] {e}")

                # =============================
                # CRITICAL: WebSocket payload must include ALL identifier fields
                # and BOTH x/y and current_x/current_y for maximum compatibility
                # =============================
                emit_payload = {
                    # === IDENTIFIERS (multiple formats for compatibility) ===
                    "robot": robot_name,           # Original format
                    "robot_id": robot_name,        # Always use robot_name from topic as primary ID
                    "name": robot_doc.get("name") if robot_doc else robot_name,
                    "id": str(robot_doc.get("_id")) if robot_doc and robot_doc.get("_id") else None,
                    
                    # === POSITION DATA (BOTH formats) ===
                    "x": float(telemetry["x"]),
                    "y": float(telemetry["y"]),
                    "yaw": float(telemetry["yaw"]),
                    "current_x": float(telemetry["x"]),
                    "current_y": float(telemetry["y"]),
                    "current_yaw": float(telemetry["yaw"]),
                    
                    # === TELEMETRY ===
                    "status": telemetry["status"],
                    "cpu_usage": float(telemetry["cpu_usage"]),
                    "ram_usage": float(telemetry["ram_usage"]),
                    "battery_level": float(telemetry["battery_level"]),
                    "temperature": float(telemetry["temperature"]),
                    
                    # === METADATA ===
                    "timestamp": time.time(),
                }

                # ✓ DEBUG: Log the complete payload being emitted
                app.logger.info(
                    f"[MQTT->WS] Emitting telemetry for robot_id={robot_name}: "
                    f"x={emit_payload['x']}, y={emit_payload['y']}, yaw={emit_payload['yaw']}, "
                    f"identifiers: robot={emit_payload['robot']}, "
                    f"robot_id={emit_payload['robot_id']}, "
                    f"name={emit_payload['name']}, "
                    f"id={emit_payload['id']}"
                )

                # Emit to WebSocket
                ws_emit("telemetry", emit_payload)

            except Exception as e:
                app.logger.error(f"[MQTT Telemetry Error] {e}", exc_info=True)

        # =============================
        # ✅ NEW TASK STATUS: robot/<robot_id>/task/status (from task runner)
        # =============================
        elif topic.startswith("robot/") and topic.endswith("/task/status"):
            try:
                data = json.loads(payload)
                task_id = data.get("task_id")
                status = data.get("status")
                robot_id = topic.split("/")[1]

                app.logger.info(
                    f"[MQTT TASK STATUS] Received from robot/{robot_id}: "
                    f"task_id={task_id}, status={status}"
                )

                if task_id and status:
                    # ✅ Update task status in database
                    success = update_task_status(task_id, status)
                    
                    if success:
                        app.logger.info(
                            f"✅ [TASK UPDATE] Task {task_id} updated to {status}"
                        )
                    else:
                        app.logger.warn(
                            f"⚠️  [TASK UPDATE] Failed to update task {task_id} to {status}"
                        )

                    # Log to InfluxDB if configured
                    if influx_client is not None and write_api is not None:
                        try:
                            point = (
                                Point("robot_task")
                                .tag("robot", robot_id)
                                .tag("status", status)
                                .field("task_id", str(task_id))
                                .field("success", 1 if success else 0)
                            )
                            write_api.write(
                                bucket=current_app.config["INFLUX_BUCKET"],
                                org=current_app.config["INFLUX_ORG"],
                                record=point
                            )
                        except Exception as e:
                            app.logger.error(f"[InfluxDB Task Write Error] {e}")

                    # Emit to WebSocket
                    ws_emit("task_status", {
                        "task_id": task_id,
                        "robot_id": robot_id,
                        "status": status,
                        "timestamp": data.get("timestamp", time.time())
                    })
                    
                    ws_emit_to_room("task_update", {
                        "task_id": task_id,
                        "robot_id": robot_id,
                        "status": status,
                        "timestamp": data.get("timestamp", time.time())
                    }, room="tasks_room")

            except Exception as e:
                app.logger.error(f"[MQTT Task Status Error] {e}", exc_info=True)

        # =============================
        # LEGACY TASK STATUS: robots/mp400/<robot>/task_status
        # =============================
        elif topic.startswith("robots/mp400/") and topic.endswith("/task_status"):
            try:
                data = json.loads(payload)
                task_id = data.get("task_id")
                status = data.get("status")

                if task_id and status:
                    update_task_status(task_id, status)

                    if influx_client is not None and write_api is not None:
                        try:
                            point = (
                                Point("robot_task")
                                .tag("robot", topic.split("/")[2])
                                .field("task_id", str(task_id))
                                .field("status", str(status))
                            )
                            write_api.write(
                                bucket=current_app.config["INFLUX_BUCKET"],
                                org=current_app.config["INFLUX_ORG"],
                                record=point
                            )
                        except Exception as e:
                            app.logger.error(f"[InfluxDB Task Write Error] {e}")

                    ws_emit("task_status", data)

            except Exception as e:
                app.logger.error(f"[MQTT Task Status Error] {e}")

        # =============================
        # MERGED MAP: warehouse/map
        # =============================
        elif topic == "warehouse/map":
            try:
                data = json.loads(payload)

                # Avoid storing or emitting the potentially large 'array' field
                data_to_store = dict(data) if isinstance(data, dict) else {}
                if "array" in data_to_store:
                    data_to_store.pop("array", None)
                    try:
                        app.logger.debug("[MQTT] Stripped 'array' field from warehouse/map payload before store/emit")
                    except Exception:
                        pass

                # Save latest merged map without the large array
                db.maps.update_one(
                    {"name": "merged_map"},
                    {"$set": data_to_store},
                    upsert=True
                )

                # Notify frontend with the smaller payload (no 'array')
                ws_emit("map_update", data_to_store)

            except Exception as e:
                app.logger.error(f"[MQTT Map Update Error] {e}")

        # =============================
        # SHELF LOCATION UPDATE: robot/<robot_id>/shelf/location
        # =============================
        elif topic.startswith("robot/") and topic.endswith("/shelf/location"):
            try:
                data = json.loads(payload)
                shelf_id = data.get("shelf_id")
                x = data.get("x")
                y = data.get("y")
                yaw = data.get("yaw", 0.0)
                task_id = data.get("task_id")
                location_status = data.get("location_status", "IN_TRANSIT")

                if shelf_id and x is not None and y is not None:
                    from .services.shelf_location_service import update_shelf_current_location, get_shelf_location_info
                    
                    success = update_shelf_current_location(
                        shelf_id,
                        float(x),
                        float(y),
                        float(yaw),
                        location_status=location_status,
                        task_id=task_id
                    )
                    
                    if success:
                        app.logger.info(f"[MQTT] Updated shelf {shelf_id} current location: ({x}, {y}) status={location_status}")
                    else:
                        app.logger.warning(f"[MQTT] Failed to update shelf {shelf_id} location")
                        return

                    location_info = get_shelf_location_info(shelf_id)
                    
                    if location_info:
                        ws_emit("shelf_location_update", {
                            "shelf_id": shelf_id,
                            "current_x": location_info["current_x"],
                            "current_y": location_info["current_y"],
                            "current_yaw": location_info["current_yaw"],
                            "storage_x": location_info["storage_x"],
                            "storage_y": location_info["storage_y"],
                            "storage_yaw": location_info["storage_yaw"],
                            "location_status": location_info["location_status"],
                            "task_id": task_id,
                            "timestamp": datetime.utcnow().isoformat()
                        })
                        
                        ws_emit_to_room("shelf_update", {
                            "shelf_id": shelf_id,
                            "current_x": location_info["current_x"],
                            "current_y": location_info["current_y"],
                            "current_yaw": location_info["current_yaw"],
                            "storage_x": location_info["storage_x"],
                            "storage_y": location_info["storage_y"],
                            "storage_yaw": location_info["storage_yaw"],
                            "location_status": location_info["location_status"],
                            "task_id": task_id,
                            "timestamp": datetime.utcnow().isoformat()
                        }, room="shelves_room")

            except Exception as e:
                app.logger.error(f"[MQTT Shelf Location Update Error] {e}", exc_info=True)

        # =============================
        # ROBOT POSITION UPDATE: robot/<robot_id>/position/update
        # =============================
        elif topic.startswith("robot/") and topic.endswith("/position/update"):
            try:
                robot_id = topic.split("/")[1]
                data = json.loads(payload)
                x = data.get("x")
                y = data.get("y")
                yaw = data.get("yaw", 0.0)

                if x is not None and y is not None:
                    # Update robot position in database
                    db.robots.update_one(
                        {"robot_id": robot_id, "deleted": False},
                        {"$set": {
                            "current_x": float(x),
                            "current_y": float(y),
                            "current_yaw": float(yaw),
                            "updated_at": datetime.utcnow()
                        }}
                    )
                    app.logger.debug(f"[MQTT] Updated robot {robot_id} position: ({x}, {y})")

                    # Emit position update with ALL identifier formats
                    position_payload = {
                        "robot": robot_id,
                        "robot_id": robot_id,
                        "id": robot_id,
                        "x": float(x),
                        "y": float(y),
                        "yaw": float(yaw),
                        "current_x": float(x),
                        "current_y": float(y),
                        "current_yaw": float(yaw),
                        "timestamp": data.get("timestamp", time.time())
                    }
                    
                    ws_emit("robot_position_update", position_payload)
                    ws_emit_to_room("robot_update", position_payload, room="robots_room")

            except Exception as e:
                app.logger.error(f"[MQTT Robot Position Update Error] {e}")

        # =============================
        # TASK PROGRESS UPDATE: robot/<robot_id>/task/progress
        # =============================
        elif topic.startswith("robot/") and topic.endswith("/task/progress"):
            try:
                data = json.loads(payload)
                task_id = data.get("task_id")
                robot_id = data.get("robot_id")
                task_state = data.get("task_state")
                status = data.get("status")

                if task_id and status:
                    update_task_status(task_id, status)
                    app.logger.debug(f"[MQTT] Task {task_id} progressed to {status}")

                    if influx_client is not None and write_api is not None:
                        try:
                            point = (
                                Point("robot_task_progress")
                                .tag("robot_id", robot_id or "unknown")
                                .tag("task_state", task_state or "UNKNOWN")
                                .field("task_id", str(task_id))
                                .field("status", str(status))
                                .time(datetime.utcnow())
                            )
                            write_api.write(
                                bucket=current_app.config["INFLUX_BUCKET"],
                                org=current_app.config["INFLUX_ORG"],
                                record=point
                            )
                        except Exception as e:
                            app.logger.error(f"[InfluxDB Task Progress Write Error] {e}")

                    ws_emit("task_progress_update", {
                        "task_id": task_id,
                        "robot_id": robot_id,
                        "task_state": task_state,
                        "status": status,
                        "timestamp": data.get("timestamp", time.time())
                    })
                    
                    ws_emit_to_room("task_update", {
                        "task_id": task_id,
                        "robot_id": robot_id,
                        "task_state": task_state,
                        "status": status,
                        "timestamp": data.get("timestamp", time.time())
                    }, room="tasks_room")

            except Exception as e:
                app.logger.error(f"[MQTT Task Progress Update Error] {e}")

        # =============================
        # OTHER CUSTOM TOPICS
        # =============================
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


def status_to_code(status: str) -> int:
    s = (status or "").upper()
    if s == "IDLE":
        return 0
    if s == "BUSY":
        return 1
    if s == "ERROR":
        return 2
    if s == "OFFLINE":
        return 3
    return -1