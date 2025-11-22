#!/usr/bin/env python3
import json
import paho.mqtt.client as mqtt


class MQTTPublisher:
    def __init__(self, host="localhost", port=1883, topic="warehouse/map"):
        self.topic = topic

        self.client = mqtt.Client(
            client_id="map_merger_node",
            callback_api_version=mqtt.CallbackAPIVersion.VERSION2
        )

        self.client.connect(host, port, keepalive=60)
        self.client.loop_start()

    def publish_map(self, data: dict):
        self.client.publish(self.topic, json.dumps(data))
