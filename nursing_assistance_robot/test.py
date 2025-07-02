# python3.8

import random

from paho.mqtt import client as mqtt_client


broker = 'kd2b8171.ala.us-east-1.emqxsl.com'
port = 8883
username = 'rokey'
password = 'rokey1234'

topic = "robot4/flag"  #토픽이름은 자유롭게 정하면 됨
client_id = f'python-mqtt-{random.randint(0, 100)}' #세션ID가 자동으로 랜덤 생성되어 관리되므로 그대로 사용해도 됨.


def connect_mqtt() -> mqtt_client.Client:   
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("Connected to MQTT Broker!")
            # 연결되면 구독 바로 실행
            client.subscribe(topic)
        else:
            print("Failed to connect, return code %d\n", rc)

    client = mqtt_client.Client(client_id=client_id, protocol=mqtt_client.MQTTv311)
    client.tls_set()  # TLS 활성화
    client.username_pw_set(username, password)
    client.on_connect = on_connect
    return client


def run():
    client = connect_mqtt()

    def on_message(client, userdata, msg):
        print(f"Received `{msg.payload.decode()}` from `{msg.topic}` topic")

    client.on_message = on_message

    # 브로커에 연결
    client.connect(broker, port)

    # 메시지 루프 시작
    client.loop_forever()


if __name__ == '__main__':
    run()