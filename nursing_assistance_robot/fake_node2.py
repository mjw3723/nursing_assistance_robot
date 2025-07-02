#!/usr/bin/env python3
import sys
import time
import threading
import rclpy
from rclpy.node import Node
from PyQt5.QtCore import QThread
from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QPushButton, QVBoxLayout, QTextEdit
import random
from rokey_interfaces.srv import AssignPatient, NotifyArrival, GoToRoom
from paho.mqtt import client as mqtt_client
from geometry_msgs.msg import PoseWithCovarianceStamped
class PatrolControlNode(Node):
    def __init__(self,namespace=''):
        ##########namespace 지정 #####################################
        super().__init__('patrol_control_node',namespace=namespace)

        ############ROBOT1##########################
        self.assign_patient_client = self.create_client(AssignPatient, 'assign_patient')
        self.notify_arrival_service = self.create_service(
            NotifyArrival,
            'notify_arrival',
            self.handle_notify_arrival
        )
        self.go_to_room_client = self.create_client(GoToRoom, 'go_to_room')
        self.broker = 'kd2b8171.ala.us-east-1.emqxsl.com'
        self.port = 8883
        self.username = 'rokey'
        self.password = 'rokey1234'

        self.topic = "robot1/position"  #토픽이름은 자유롭게 정하면 됨
        self.client_id = f'python-mqtt-{random.randint(0, 100)}' #세션ID가 자동으로 랜덤 생성되어 관리되므로 그대로 사용해도 됨.
        self.client = self.connect_mqtt()
        self.client.loop_start()
        self.amcl_subscription = self.create_subscription(
                PoseWithCovarianceStamped,
                '/robot1/amcl_pose',
                self.amcl_listener_callback,
                10
            )
        ############ROBOT4##########################

        self.gui_log_callback = None  # GUI 로그 출력을 위한 콜백

    def register_gui_logger(self, log_func):
        self.gui_log_callback = log_func
    
    def amcl_listener_callback(self,msg):
        self.amcl_x = msg.pose.pose.position.x
        self.amcl_y = msg.pose.pose.position.y
        self.get_logger().info(f"📍 수신된 AMCL 위치: x={self.amcl_x:.2f}, y={self.amcl_y :.2f}")

    def connect_mqtt(self) -> mqtt_client.Client:
        def on_connect(client, userdata, flags, rc):
            if rc == 0:
                print("Connected to MQTT Broker!")
            else:
                print("Failed to connect, return code %d\n", rc)

        client = mqtt_client.Client(client_id=self.client_id, protocol=mqtt_client.MQTTv311)
        client.tls_set()
        client.username_pw_set(self.username, self.password)
        client.on_connect = on_connect
        client.connect(self.broker, self.port)
        return client
    
    def publish(self,msg):
        result = self.client.publish(self.topic, msg)
        status = result[0]
        if status == 0:
            print(f"Sent `{msg}` to topic `{self.topic}`")
        else:
            print(f"Failed to send message to topic {self.topic}")

    def call_service(self, client, req, callback):
        def wait_and_call():
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn(f'{client.srv_name} 서비스가 활성화될 때까지 대기 중...')
                time.sleep(1.0)
            future = client.call_async(req)

            def future_done(fut):
                try:
                    res = fut.result()
                    callback(success=True, response=res)
                except Exception as e:
                    self.get_logger().error(f'{client.srv_name} 서비스 호출 실패: {e}')
                    callback(success=False, response=None)

            future.add_done_callback(future_done)

        threading.Thread(target=wait_and_call, daemon=True).start()

    # NotifyArrival 서비스 요청 처리
    def handle_notify_arrival(self, request, response):
        self.patient_id = request.patient_id
        self.get_logger().info(f"{self.patient_id} 수신됨 → patient_idpatient_idpatient_id")
        self.client.publish(self.amcl_x+','+self.amcl_y)
        response.ack = True
        return response
        

class PatrolControlGUI(QWidget):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.ros_node.register_gui_logger(self.log)

        self.setWindowTitle('순찰 로봇 제어 GUI')
        self.setGeometry(100, 100, 400, 400)

        self.log_box = QTextEdit(self)
        self.log_box.setReadOnly(True)

        self.btn_assign_patient = QPushButton('환자 배정 (AssignPatient)', self)
        self.btn_go_to_room = QPushButton('병실 이동 허가 (GoToRoom)', self)
        self.btn_exit = QPushButton('종료', self)

        layout = QVBoxLayout()
        layout.addWidget(QLabel('🧠 순찰 로봇 서비스 제어'))
        layout.addWidget(self.btn_assign_patient)
        layout.addWidget(self.btn_go_to_room)
        layout.addWidget(self.log_box)
        layout.addWidget(self.btn_exit)
        self.setLayout(layout)

        self.btn_assign_patient.clicked.connect(self.assign_patient)
        self.btn_go_to_room.clicked.connect(self.room)
        self.btn_exit.clicked.connect(self.close)

    def log(self, msg):
        self.log_box.append(msg)

    def assign_patient(self):
        patient_id = 1
        self.log(f'환자 배정 요청 (patient_id={patient_id})')
        req = AssignPatient.Request()
        req.patient_id = patient_id

        def callback(success, response):
            if success and response.success:
                self.log('✅ 환자 배정 성공!')
            else:
                self.log('❌ 환자 배정 실패')

        self.ros_node.call_service(self.ros_node.assign_patient_client, req, callback)

    def room(self):
        permission =  True
        self.log(f'병원 이동 요청 (permission={permission})')
        req = GoToRoom.Request()
        req.permission = permission

        def callback(success, response):
            if success and response.accepted:
                self.log('✅ 환자 배정 성공!')
            else:
                self.log('❌ 환자 배정 실패')

        self.ros_node.call_service(self.ros_node.go_to_room_client, req, callback)


def main():
    rclpy.init()
    ros_node = PatrolControlNode()

    class RosSpinThread(QThread):
        def __init__(self, node):
            super().__init__()
            self.node = node

        def run(self):
            rclpy.spin(self.node)

    ros_spin_thread = RosSpinThread(ros_node)
    ros_spin_thread.start()

    app = QApplication(sys.argv)
    gui = PatrolControlGUI(ros_node)

    def on_close():
        gui.log('ROS2 노드 종료 중...')
        ros_node.destroy_node()
        rclpy.shutdown()
        ros_spin_thread.quit()
        ros_spin_thread.wait()

    app.aboutToQuit.connect(on_close)
    gui.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
