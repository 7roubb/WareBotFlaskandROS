"""
MQTT Communication Handler
"""
import json
import threading
import time
from collections import deque
from typing import Dict, Any, Optional, Callable
import paho.mqtt.client as mqtt


class MQTTHandler:
    """Handles MQTT connectivity and message publishing"""
    
    def __init__(self, robot_id: str, host: str, port: int, qos: int, 
                 reconnect_base: float, reconnect_max: float, logger,
                 on_task_assignment: Callable = None,
                 on_reference_update: Callable = None):
        self.robot_id = robot_id
        self.host = host
        self.port = port
        self.qos = qos
        self.logger = logger
        
        self._reconnect_base = reconnect_base
        self._reconnect_max = reconnect_max
        self._mqtt_connected = False
        self._mqtt_reconnect_attempts = 0
        self._mqtt_lock = threading.Lock()
        
        # Callbacks
        self.on_task_assignment = on_task_assignment
        self.on_reference_update = on_reference_update
        
        # Status publishing queue
        self.status_update_queue = deque()
        self.status_publish_lock = threading.Lock()
        
        # Initialize MQTT client
        try:
            self.mqtt_client = mqtt.Client(
                client_id=self.robot_id,
                callback_api_version=mqtt.CallbackAPIVersion.VERSION1
            )
        except Exception as e:
            self.logger.warn(f"mqtt.Client init fallback: {e}")
            self.mqtt_client = mqtt.Client(client_id=self.robot_id)
        
        self.mqtt_client.on_connect = self._on_connect
        self.mqtt_client.on_message = self._on_message
        self.mqtt_client.on_disconnect = self._on_disconnect
        self.mqtt_client.on_publish = self._on_publish
    
    def start(self):
        """Start MQTT connection"""
        try:
            self.logger.info(f"Connecting to MQTT broker {self.host}:{self.port}")
            self.mqtt_client.connect(self.host, self.port, keepalive=60)
            self.mqtt_client.loop_start()
        except Exception as e:
            self.logger.error(f"MQTT connect error: {e}")
            self._schedule_reconnect()
    
    def stop(self):
        """Stop MQTT connection"""
        try:
            self.mqtt_client.disconnect()
            self.mqtt_client.loop_stop()
        except Exception:
            pass
    
    def is_connected(self) -> bool:
        """Check if MQTT is connected"""
        return self._mqtt_connected
    
    def _schedule_reconnect(self):
        """Schedule reconnect with exponential backoff"""
        with self._mqtt_lock:
            self._mqtt_reconnect_attempts += 1
            delay = min(
                self._reconnect_base * (2 ** (self._mqtt_reconnect_attempts - 1)),
                self._reconnect_max
            )
            self.logger.info(f"Scheduling MQTT reconnect in {delay:.1f}s")
            timer = threading.Timer(delay, self._attempt_reconnect)
            timer.daemon = True
            timer.start()
    
    def _attempt_reconnect(self):
        """Try to reconnect"""
        with self._mqtt_lock:
            try:
                self.logger.info("Attempting MQTT reconnect...")
                self.mqtt_client.reconnect()
            except Exception as e:
                self.logger.warn(f"Reconnect failed: {e}")
                self._schedule_reconnect()
    
    def _on_connect(self, client, userdata, flags, rc, properties=None):
        """MQTT connect callback"""
        if rc == 0:
            self.logger.info("✅ MQTT connected")
            self._mqtt_connected = True
            self._mqtt_reconnect_attempts = 0
            
            try:
                client.subscribe(f"robot/{self.robot_id}/task/assignment", qos=self.qos)
                client.subscribe(f"robot/{self.robot_id}/reference_point/update", qos=self.qos)
                client.subscribe("robot/all/task/assignment", qos=self.qos)
                self.logger.info("✅ MQTT subscriptions set")
                
                # Publish ready status
                self.queue_status_update("READY", None)
            except Exception as e:
                self.logger.error(f"Subscription failed: {e}")
        else:
            self.logger.error(f"❌ MQTT connection failed, rc={rc}")
            self._schedule_reconnect()
    
    def _on_disconnect(self, client, userdata, rc, properties=None):
        """MQTT disconnect callback"""
        self._mqtt_connected = False
        if rc != 0:
            self.logger.warn(f"⚠️  Unexpected MQTT disconnect (rc={rc})")
            self._schedule_reconnect()
    
    def _on_publish(self, client, userdata, mid):
        """MQTT publish callback"""
        self.logger.debug(f"✅ MQTT message {mid} published successfully")
    
    def _on_message(self, client, userdata, msg):
        """MQTT message callback"""
        try:
            payload = msg.payload.decode("utf-8")
            data = json.loads(payload)
        except Exception as e:
            self.logger.error(f"Failed to decode MQTT payload: {e}")
            return
        
        topic = msg.topic
        if "task/assignment" in topic:
            self.logger.info(f"📨 Task assignment received via MQTT: {data.get('task_id')}")
            if isinstance(data, dict) and self.on_task_assignment:
                self.on_task_assignment(data)
        elif "reference_point/update" in topic:
            self.logger.info("Reference point update received")
            if isinstance(data, dict) and self.on_reference_update:
                self.on_reference_update(data)
    
    def queue_status_update(self, status: str, task_id: Optional[str]):
        """Queue a status update for robust delivery"""
        with self.status_publish_lock:
            self.status_update_queue.append({
                "status": status,
                "task_id": task_id,
                "timestamp": time.time(),
                "retry_count": 0
            })
            self.logger.info(f"📝 Status queued: {status}")
    
    def publish_direct(self, payload: Dict[str, Any], topic_suffix: str = "task/status"):
        """Direct MQTT publish (for non-queued messages)"""
        try:
            topic = f"robot/{self.robot_id}/{topic_suffix}"
            payload_str = json.dumps(payload)
            self.mqtt_client.publish(topic, payload_str, qos=self.qos)
            self.logger.debug(f"📤 Published to {topic}")
        except Exception as e:
            self.logger.error(f"MQTT publish failed: {e}")
    
    def publish_with_retry(self, payload: Dict[str, Any], status_item: Dict):
        """Publish to MQTT with automatic retry on failure"""
        try:
            topic = f"robot/{self.robot_id}/task/status"
            payload_str = json.dumps(payload)
            
            result = self.mqtt_client.publish(
                topic,
                payload_str,
                qos=self.qos,
                retain=False
            )
            
            if result.rc != mqtt.MQTT_ERR_SUCCESS:
                status_item["retry_count"] += 1
                self.logger.warn(
                    f"⚠️  MQTT publish returned rc={result.rc}, will retry "
                    f"(attempt {status_item['retry_count']}/10)"
                )
            else:
                self.logger.debug(f"✅ Published to {topic}")
                
        except Exception as e:
            status_item["retry_count"] += 1
            self.logger.error(f"MQTT publish error: {e} (attempt {status_item['retry_count']}/10)")