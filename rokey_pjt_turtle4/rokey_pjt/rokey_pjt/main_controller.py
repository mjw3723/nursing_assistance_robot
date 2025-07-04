# main_controller.py

import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import numpy as np
import cv2
import threading

import tkinter as tk
from PIL import Image, ImageTk
from tkinter import scrolledtext

import random
from paho.mqtt import client as mqtt_client

from geometry_msgs.msg import Point
from rokey_interfaces.srv import Firstcmd, EndFlag


'''MQTT clould 통신 클래스'''
class MQTT(Node):
    def __init__(self, controller_node=None):
        super().__init__('mqtt_node')

        # MQTT clould 설정
        self.broker = 'kd2b8171.ala.us-east-1.emqxsl.com'
        self.port = 8883
        self.username = 'rokey'
        self.password = 'rokey1234'
        self.robot1_pose_topic = 'robot1/position'
        # self.robot1_flag_topic = 'robot1/flag'
        self.robot4_flag_topic = 'robot4/flag'
        self.client_id = f'python-mqtt-{random.randint(0, 100)}'

        # MQTT 클라이언트 생성 및 설정
        self.client = mqtt_client.Client(client_id=self.client_id, protocol=mqtt_client.MQTTv311)
        self.client.tls_set()
        self.client.username_pw_set(self.username, self.password)

        # 콜백 등록
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        
        # 컨트롤러 노드
        self.controller_node = controller_node

        # robot1 좌표 초기화
        self.robot1_x = 0.0
        self.robot1_y = 0.0


    '''MQTT 브로커에 연결되었을 때 자동으로 호출되는 콜백 함수'''
    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info("✅ Connected to MQTT Broker!")

            client.subscribe(self.robot1_pose_topic)
            self.get_logger().info(f"📥 Topic Subscriber 생성 `{self.robot1_pose_topic}`")

            # client.subscribe(self.robot1_flag_topic)
            # self.get_logger().info(f"📥 Topic Subscriber 생성 `{self.robot1_flag_topic}`")
        else:
            self.get_logger().info(f"❌ Failed to connect, return code {rc}")

    
    '''robot4가 랑데부 포인트에 도착했을 때 Robot1 Main Controller에 퍼블리시하는 함수'''
    def robot4_flag_publish(self):
        try:
            self.get_logger().info(f"self.controller_node.robot4_rendezvous_flag = {self.controller_node.robot4_rendezvous_flag}")
            msg = int(self.controller_node.robot4_rendezvous_flag)
            result = self.client.publish(self.robot4_flag_topic, msg)
            if result[0] == 0:
                self.get_logger().info(f"📤 Robot1 Main Controller에 Publish '{self.robot4_flag_topic}': '{msg}'")
            else:
                self.get_logger().info(f"❌ Failed to send message to topic `{self.robot4_flag_topic}`")
        except KeyboardInterrupt:
            self.get_logger().info("⏹️ Publishing stopped by user.")
        finally:
            self.client.loop_stop()


    '''robot1_pose 토픽메시지가 도착했을 때 호출되는 콜백 함수'''
    def on_message(self, client, userdata, msg):
        topic = msg.topic
        msg_decode = msg.payload.decode()

        # robot1의 위치를 구독할 때
        if topic == 'robot1/position':
            msg_x, msg_y = msg_decode.split(',')
            self.robot1_x = float(msg_x)
            self.robot1_y = float(msg_y)
            self.get_logger().info(f"📥 MQTT 메시지 Subscription x: {self.robot1_x}, y: {self.robot1_y}, type: {type(self.robot1_x)}")
            self.controller_node.log_callback(f'[robot4] robot1 위치 MQTT 메시지 수신 x={self.robot1_x}, y={self.robot1_y}')

            #####
            self.controller_node.robot1_rendezvous_flag = True
            self.get_logger().info("✅ robot1_rendezvous_flag = True")
            self.controller_node.log_callback(f'[robot1] 랑데부 포인트 도착!')

            point = Point()
            point.x = self.robot1_x
            point.y = self.robot1_y
            point.z = 0.0
            self.controller_node.robot1_position_pub.publish(point)
            self.get_logger().info("✅ ROS2 토픽 /robot1/position 으로 publish 완료")
            self.controller_node.log_callback(f'[robot4] robot1 위치 nav node에 publish!')
            #####

        # robot1 도착 여부를 구독할 때
        # elif topic == 'robot1/flag':
        #     # robot1이 랑데부 포인트 도착 시 플래그를 True로 설정하고, nav node의 robot1 좌표값을 publish
        #     if msg_decode:
        #         self.controller_node.robot1_rendezvous_flag = True
        #         self.get_logger().info("✅ robot1_rendezvous_flag = True")
        #         self.controller_node.log_callback(f'[robot1] 랑데부 포인트 도착 MQTT 메시지 수신!')

        #         point = Point()
        #         point.x = self.robot1_x
        #         point.y = self.robot1_y
        #         point.z = 0.0
        #         self.controller_node.robot1_position_pub.publish(point)
        #         self.get_logger().info("✅ ROS2 토픽 /robot1/position 으로 publish 완료")
        #         self.controller_node.log_callback(f'[robot4] robot1 위치 nav node에 publish!')

        #     else:
        #         self.controller_node.robot1_rendezvous_flag = False
        #         self.get_logger().info("✅ robot1_rendezvous_flag = False")


    '''MQTT 클라이언트를 브로커에 연결하고, 메시지 수신 루프를 시작하는 함수'''
    def run(self):
        self.client.connect(self.broker, self.port)
        self.client.loop_forever()


'''main controller ros2 node 클래스'''
class MainControllerNode(Node):
    def __init__(self, mqtt, update_callback, log_callback):
        super().__init__('main_controller')

        self.get_logger().info("🖥️  Main Controller 시작!")

        self.bridge = CvBridge()
        self.latest_image = None

        # 이미지 토픽 subscriber 생성
        self.subscription = self.create_subscription(CompressedImage, '/robot4/detect_img', self.image_callback,10)
        self.get_logger().info("✅ Subscribed to /robot4/oakd/rgb/image_raw/compressed")

        # robot1 랑데부 도착 시 x,y 값 퍼블리셔
        self.robot1_position_pub = self.create_publisher(Point, '/robot4/robot1_pos', 10)

        # robot4 rendezvous 도착 여부 서비스 서버 생성
        self.srv = self.create_service(EndFlag, '/robot4_rendezvous', self.robot4_rendezvous_callback)
        self.get_logger().info("✅ '/robot4_rendezvous' 서비스 서버 시작")
        
        # robot4 배송 물품 요청 서비스 클라이언트 생성
        self.robot4_object_cli = self.create_client(Firstcmd, '/object_name')
        while not self.robot4_object_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("⏰ /object_name 서비스 서버 대기 중...")
        self.get_logger().info("✅ /object_name 서비스 서버 연결 완료")

        # robot4 랑데부 포인트 만남 서비스 클라이언트 생성
        self.robot4_meet_cli = self.create_client(EndFlag, '/robot4_meet')
        while not self.robot4_meet_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("⏰ /robot4_meet 서비스 서버 대기 중...")
        self.get_logger().info("✅ /robot4_meet 서비스 서버 연결 완료")

        # mqtt 클래스
        self.mqtt = mqtt

        # 콜백함수 설정
        self.update_callback = update_callback
        self.log_callback = log_callback

        # 로봇 flag
        self.moving_flag = False
        self.after_rendezvous = False

        # 터틀봇 랑데부 포인트 도착 여부 flag
        self.robot1_rendezvous_flag = False
        self.robot4_rendezvous_flag = False


    '''터틀봇의 RBG 이미지를 subscribtion하면 실행되는 callback 함수'''
    def image_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if image is not None:
                self.latest_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)  # RGB 변환
                self.update_callback(self.latest_image)
        except Exception as e:
            self.get_logger().error(f"이미지 처리 오류: {e}")


    '''GUI에서 배달 물품 요청을 했을 때 robot4에 서비스를 request 하는 함수'''
    def send_robot4_object_request(self, label, callback):
        request = Firstcmd.Request()
        request.label = label
        future = self.robot4_object_cli.call_async(request)
        future.add_done_callback(lambda f: callback(f.result()))


    '''robot4가 랑데부 포인트에 도착했을 때 도착 여부를 request를 받고 response를 보내는 함수'''
    def robot4_rendezvous_callback(self, request, response):
        data = request.data
        self.get_logger().info(f"[robot4] 랑데부 포인트 도착 여부: {data}")

        if data:
            msg = "✅ [robot4] 랑데부 포인트 도착!"
            self.get_logger().info(msg)
            self.log_callback(msg)
            self.robot4_rendezvous_flag = True
            response.success = True
        else:
            msg = "⚠️ [robot4] 랑데부 포인트 도착 실패"
            self.get_logger().info(msg)
            self.log_callback(msg)
            self.robot4_rendezvous_flag = False
            response.success = False

        # self.mqtt.robot4_flag_publish()

        return response
    

    '''랑데부 포인트에 robot이 둘 다 도착했을 때, 도착 여부를 robot4에 request 하는 함수'''
    def send_robot4_meet_request(self, data, callback):
        request = EndFlag.Request()
        request.data = data
        future = self.robot4_meet_cli.call_async(request)
        future.add_done_callback(lambda f: callback(f.result()))


'''main controller GUI 제작 클래스'''
class MainControllerGUI(tk.Tk):
    def __init__(self, ros_node):
        super().__init__()
        self.title("Main Controller")
        self.geometry("480x900")
        # self.resizable(False, False)
        
        # 이미지 텍스트 라벨
        self.input_label = tk.Label(self, text="Robot4 RGB Cam", font=("Arial", 16, "bold"))
        self.input_label.pack()

        # 이미지 라벨
        self.image_label = tk.Label(self)
        self.image_label.pack(expand=True)

        # 물품 배송 안내 라벨
        self.input_label = tk.Label(self, text="배송할 물품을 입력하세요")
        self.input_label.pack()

        # 물품 배송 입력창 및 버튼
        self.entry = tk.Entry(self)
        self.entry.pack()

        # 물품 배송 요청 버튼
        self.send_button = tk.Button(self, text="물품 배송 요청", command=self.delivery_send_button_callback)
        self.send_button.pack()

        # 물품 배송 상태 라벨
        self.status_label = tk.Label(self, text="터틀봇이 초기 상태에 위치합니다!")
        self.status_label.pack()

        # 두 로봇이 랑데부 포인트 도착 시 클릭하는 버튼
        self.robot_meet_button = tk.Button(self, text="랑데부 포인트 도착", command=self.robot_meet_button_callback)
        self.robot_meet_button.pack()

        # robot1 랑데부 포인트 도착 후 상태 라벨 
        self.robot1_status_label = tk.Label(self, text="⚠️ [robot1] 아직 robot1이 도착하지 않았습니다!")
        self.robot1_status_label.pack()

        # robot4 랑데부 포인트 도착 후 상태 라벨 
        self.robot4_status_label = tk.Label(self, text="⚠️ [robot4] 아직 robot4가 도착하지 않았습니다!")
        self.robot4_status_label.pack()

        # 로그 출력 영역
        self.log_text = scrolledtext.ScrolledText(self, height=8, state='disabled')
        self.log_text.pack(fill='both', expand=False, padx=5, pady=5)
        
        # ros node 인스턴스 생성
        self.ros_node = ros_node
        self.current_photo = None

        # ROS spin 주기적 호출
        self.after(10, self.spin_ros)

        # 종료 버튼 추가
        self.quit_button = tk.Button(self, text="종료", command=self.on_quit)
        self.quit_button.pack(pady=10)  # 여백 추가

        # 종료 시점에 실행될 콜백 등록
        self.protocol("WM_DELETE_WINDOW", self.on_quit)


    '''ROS 2 노드를 주기적으로 실행하기 위한 함수'''
    def spin_ros(self):
        rclpy.spin_once(self.ros_node, timeout_sec=0)
        self.after(10, self.spin_ros)


    """OpenCV(RGB) 이미지를 Tkinter 이미지로 변환 후 Label에 표시"""
    def update_image(self, image):
        im_pil = Image.fromarray(image)
        imgtk = ImageTk.PhotoImage(image=im_pil)

        self.current_photo = imgtk
        self.image_label.config(image=imgtk)


    """로그 메시지를 Text 위젯에 출력"""
    def log(self, message: str):
        self.log_text.config(state='normal')
        self.log_text.insert('end', message + '\n')
        self.log_text.see('end')  # 자동 스크롤
        self.log_text.config(state='disabled')


    """물품 배송 요청 버튼을 눌렀을 때 실행되는 함수"""
    def delivery_send_button_callback(self):
        label = self.entry.get().strip()

        # 라벨을 입력하지 않았을 때
        if not label:
            self.status_label.config(text="⚠️ 라벨을 입력하세요.")
            self.log("⚠️ 라벨을 입력하세요.")
            return
        
        # 라벨이 클래스 내에 없을 때
        elif label not in ["bandage", "codaewon", "cup", "tylenol"]:
            self.status_label.config(text=f"⚠️ [robot4] '{label}'는 제조실에 없습니다.")
            self.log(f"⚠️ [robot4] '{label}'는 제조실에 없습니다.")
            return
        
        # 로봇이 라벨을 입력 받고 이미 이동중일 때
        elif self.ros_node.moving_flag:
            self.status_label.config(text="⚠️ [robot4] 터틀봇이 이미 배송중입니다.")
            self.log("⚠️ [robot4] 터틀봇이 이미 배송중입니다.")
            return

        self.status_label.config(text="✅ [robot4] 서비스 요청 중...")
        self.log("✅ [robot4] 서비스 요청 중...")

        self.status_label.config(text=f"✅ [robot4] '{label}'을 찾으러 이동합니다!")
        self.log(f"✅ [robot4] '{label}'을 찾으러 이동합니다!")

        self.ros_node.moving_flag = True

        # main controller에서 response를 받았을 때 실행되는 callback 함수
        def robot4_object_request_callback(response):
            if response is None:
                self.status_label.config(text="❌ 서비스 응답 실패")
            elif response.success:
                self.status_label.config(text=f"✅ [robot4] '{label}' 인식 위치에 도착했습니다!")
                self.log(f"✅ [robot4] '{label}' 인식 위치에 도착했습니다!")
                self.log(f"✅ [robot4] '{label}' YOLO detect 시작")
            else:
                self.status_label.config(text=f"⚠️ [robot4] '{label}'는 제조실에 없습니다.")
                self.log(f"⚠️ [robot4] '{label}'는 제조실에 없습니다.")

        self.ros_node.send_robot4_object_request(label, robot4_object_request_callback)

    
    """랑데부 포인트 도착 버튼을 눌렀을 때 실행되는 함수"""
    def robot_meet_button_callback(self):
        # 로봇이 랑데부 포인트에 둘 다 도착하지 않았을 때
        if not self.ros_node.robot1_rendezvous_flag and not self.ros_node.robot4_rendezvous_flag:
            self.robot1_status_label.config(text=f"⚠️ [robot1] 아직 robot1이 도착하지 않았습니다!")
            self.robot4_status_label.config(text=f"⚠️ [robot4] 아직 robot4가 도착하지 않았습니다!")
            self.log(f"⚠️ [robot1] [robot4] 아직 robot이 도착하지 않았습니다!")
            return

        # 로봇4만 랑데부 포인트에 도착했을 때
        if not self.ros_node.robot1_rendezvous_flag:
            self.robot1_status_label.config(text=f"⚠️ [robot1] 아직 robot1이 도착하지 않았습니다!")
            self.robot4_status_label.config(text=f"✅ [robot4] robot4가 도착했습니다!")
            self.log(f"⚠️ [robot1] 아직 robot1이 도착하지 않았습니다!")
            return
        
        # 로봇1만 랑데부 포인트에 도착했을 때
        if not self.ros_node.robot4_rendezvous_flag:
            self.robot1_status_label.config(text=f"✅ [robot1] robot1이 도착했습니다!")
            self.robot4_status_label.config(text=f"⚠️ [robot4] 아직 robot4가 도착하지 않았습니다!")
            self.log(f"⚠️ [robot4] 아직 robot4가 도착하지 않았습니다!")
            return
        
        # 이미 로봇이 물품을 전달했을 때
        if self.ros_node.after_rendezvous:
            self.log(f"⚠️ [robot1] [robot4] 이미 로봇이 물품 전달을 완료했습니다!")
            return
        
        # log 출력
        self.log("✅ [robot1] [robot4] 서비스 요청 중...")
        self.log("✅ [robot1] [robot4] 로봇이 랑데부 포인트에 모두 도착했습니다!")
        self.log("✅ [robot1] 병동으로 이동합니다!")
        self.log("✅ [robot4] dock 위치로 이동합니다!")

        # 라벨 출력
        self.robot1_status_label.config(text=f"✅ [robot1] robot1이 도착했습니다!")
        self.robot4_status_label.config(text=f"✅ [robot4] robot4가 도착했습니다!")

        # main controller에서 response를 받았을 때 실행되는 callback 함수
        def robot4_meet_request_callback(response):
            if response is None:
                self.robot4_status_label.config(text="❌ 서비스 응답 실패")
            elif response.success:
                self.robot4_status_label.config(text=f"✅ [robot4] 물품을 전달하고 초기 dock 위치로 이동합니다!")
                self.log(f"✅ [robot4] 물품을 전달하고 초기 dock 위치로 이동합니다!")
            else:
                self.robot4_status_label.config(text=f"⚠️ [robot4] 초기 dock 위치 이동 실패")
                self.log(f"⚠️ [robot4] 초기 dock 위치 이동 실패")
        
        meet = True
        self.ros_node.after_rendezvous = True

        # robot4_flag 다시 MQTT로 publish
        self.ros_node.mqtt.robot4_flag_publish()

        self.ros_node.send_robot4_meet_request(meet, robot4_meet_request_callback)


    '''main controller GUI를 종료시키는 함수'''
    def on_quit(self):
        self.log("🛑 GUI 종료 및 ROS2 노드 셧다운 중...")
        if self.ros_node:
            self.ros_node.destroy_node()
        rclpy.shutdown()
        self.destroy()  # Tkinter 창 닫기


def main():
    rclpy.init()

    mqtt = MQTT()  # 먼저 MQTT 객체 생성
    controller_node = MainControllerNode(mqtt, None, None)  # GUI 없이 일단 생성
    mqtt.controller_node = controller_node  # MQTT에 controller_node 할당

    gui = MainControllerGUI(controller_node)  # GUI는 controller_node를 알고 있게
    controller_node.update_callback = gui.update_image
    controller_node.log_callback = gui.log

    gui.ros_node = controller_node

    # MQTT Subscriber 쓰레드 실행
    mqtt_thread = threading.Thread(target=mqtt.run, daemon=True)
    mqtt_thread.start()

    gui.mainloop()

    controller_node.destroy_node()
    mqtt.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()