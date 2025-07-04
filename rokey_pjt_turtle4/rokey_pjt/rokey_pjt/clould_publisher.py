import random
import time
from paho.mqtt import client as mqtt_client

class MQTTPublisher:
    def __init__(self, broker, port, username, password, topic):
        self.broker = broker
        self.port = port
        self.username = username
        self.password = password
        self.topic = 'robot4/flag'
        # self.topic = topic
        self.client_id = f'python-mqtt-{random.randint(0, 100)}'
        self.client = mqtt_client.Client(client_id=self.client_id, protocol=mqtt_client.MQTTv311)

        # TLS, 인증, 콜백 등록
        self.client.tls_set()
        self.client.username_pw_set(self.username, self.password)
        self.client.on_connect = self.on_connect

    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            print("✅ Connected to MQTT Broker!")
        else:
            print(f"❌ Failed to connect, return code {rc}")

    def connect(self):
        self.client.connect(self.broker, self.port)
        self.client.loop_start()

    def publish_loop(self):
        msg_count = 0
        try:
            while True:
                msg = f"Message number {msg_count}"
                result = self.client.publish(self.topic, msg)
                if result[0] == 0:
                    print(f"📤 Sent `{msg}` to topic `{self.topic}`")
                else:
                    print(f"❌ Failed to send message to topic `{self.topic}`")
                msg_count += 1
                time.sleep(1)
        except KeyboardInterrupt:
            print("⏹️ Publishing stopped by user.")
        finally:
            self.client.loop_stop()

    def run(self):
        self.connect()
        self.publish_loop()


if __name__ == '__main__':
    publisher = MQTTPublisher(
        broker='kd2b8171.ala.us-east-1.emqxsl.com',
        port=8883,
        username='rokey',
        password='rokey1234',
        topic='python/mqtt'
    )
    publisher.run()
