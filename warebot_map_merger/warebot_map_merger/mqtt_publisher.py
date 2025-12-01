#!/usr/bin/env python3
import json
import paho.mqtt.client as mqtt


class MQTTPublisher:
    def __init__(self, host="localhost", port=1883, topic="warehouse/map"):
        self.topic = topic

        self.client = mqtt.Client(
            client_id="warebot_map_merger",
            protocol=mqtt.MQTTv5
        )

        self.client.connect(host, port, keepalive=60)
        self.client.loop_start()

    def publish_map(self, data: dict):
        try:
            payload = json.dumps(data)
            self.client.publish(self.topic, payload)
        except Exception as e:
            print(f"[MQTT ERROR] {e}")
