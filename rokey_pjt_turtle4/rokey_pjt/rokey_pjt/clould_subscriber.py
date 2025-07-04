import random
from paho.mqtt import client as mqtt_client

class MQTTSubscriber:
    def __init__(self, broker, port, username, password, topic):
        self.broker = broker
        self.port = port
        self.username = username
        self.password = password
        self.topic = topic
        self.client_id = f'python-mqtt-{random.randint(0, 100)}'

        # MQTT 클라이언트 생성 및 설정
        self.client = mqtt_client.Client(client_id=self.client_id, protocol=mqtt_client.MQTTv311)
        self.client.tls_set()
        self.client.username_pw_set(self.username, self.password)

        # 콜백 등록
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            print("✅ Connected to MQTT Broker!")
            client.subscribe(self.topic)
            print(f"📥 Subscribed to topic `{self.topic}`")
        else:
            print(f"❌ Failed to connect, return code {rc}")

    def on_message(self, client, userdata, msg):
        print(f"📩 Received `{msg.payload.decode()}` from `{msg.topic}`")

    def run(self):
        self.client.connect(self.broker, self.port)
        self.client.loop_forever()


if __name__ == '__main__':
    subscriber = MQTTSubscriber(
        broker='kd2b8171.ala.us-east-1.emqxsl.com',
        port=8883,
        username='rokey',
        password='rokey1234',
        topic='python/mqtt'
    )
    subscriber.run()
