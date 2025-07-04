#!/usr/bin/env python3
import random
import time
from paho.mqtt import client as mqtt_client

# 브로커 설정
broker = 'kd2b8171.ala.us-east-1.emqxsl.com'
port = 8883
username = 'rokey'
password = 'rokey1234'

# 토픽 및 클라이언트 ID
topic = "robot4/flag"
client_id = f'python-mqtt-pub-{random.randint(0, 1000)}'

# MQTT 연결 함수
def connect_mqtt():
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("✅ MQTT 브로커 연결 완료!")
        else:
            print(f"❌ 연결 실패, 코드: {rc}")

    client = mqtt_client.Client(client_id=client_id, protocol=mqtt_client.MQTTv311)
    client.tls_set()  # TLS 사용 (port=8883이면 보통 TLS)
    client.username_pw_set(username, password)
    client.on_connect = on_connect
    client.connect(broker, port)
    return client

# 메시지 퍼블리시 함수
def publish_loop(client):
    count = 1
    try:
        while True:
            msg = f"True"
            result = client.publish(topic, msg)
            status = result[0]
            if status == 0:
                print(f"📤 발행 성공: `{msg}` → `{topic}`")
            else:
                print(f"❌ 발행 실패: `{msg}`")
            count += 1
            time.sleep(20)
    except KeyboardInterrupt:
        print("\n⛔️ 사용자 중단 → 퍼블리셔 종료")

# 실행 함수
def run():
    client = connect_mqtt()
    client.loop_start()  # 비동기 처리
    publish_loop(client)

if __name__ == '__main__':
    run()
