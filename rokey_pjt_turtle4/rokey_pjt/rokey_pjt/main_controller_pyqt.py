#!/usr/bin/env python3
import sys
import time
import threading
import rclpy
from rclpy.node import Node
from PyQt5.QtCore import QThread
from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QPushButton, QVBoxLayout, QTextEdit, QLineEdit

from rokey_interfaces.srv import AssignPatient, NotifyArrival, GoToRoom, Firstcmd, EndFlag


class PatrolControlNode(Node):
    def __init__(self,namespace=''):
        super().__init__('patrol_control_node',namespace=namespace)

        '''ROBOT1'''
        self.assign_patient_client = self.create_client(AssignPatient, 'assign_patient')
        self.notify_arrival_service = self.create_service(NotifyArrival, 'notify_arrival', self.handle_notify_arrival)
        self.go_to_room_client = self.create_service(GoToRoom, 'go_to_room',self.handle_go_to_room)

        '''ROBOT4'''
        # robot4 rendezvous 도착 여부 서비스 서버 생성
        self.srv = self.create_service(EndFlag, '/robot4_rendezvous', self.handle_robot4_rendezvous)
        # robot4 배송 물품 요청 서비스 클라이언트 생성
        self.robot4_object_client = self.create_client(Firstcmd, '/object_name')
        # robot4 랑데부 포인트 만남 서비스 클라이언트 생성
        self.robot4_meet_client = self.create_client(EndFlag, '/robot4_meet')

        # GUI 로그 출력을 위한 콜백
        self.gui_log_callback = None

    def register_gui_logger(self, log_func):
        self.gui_log_callback = log_func

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
        patient_id = request.patient_id
        log_msg = f'🚑 [NotifyArrival] 도착 통보 요청 수신 (patient_id={patient_id})'
        self.get_logger().info(log_msg)

        if self.gui_log_callback:
            self.gui_log_callback(log_msg)
            self.gui_log_callback('✅ 도착 통보 응답: ack=True')

        response.ack = True
        return response
        
    # 버튼 클릭 시 사용할 내부 처리 메서드
    def simulate_notify_arrival(self, patient_id):
        req = NotifyArrival.Request()
        req.patient_id = patient_id
        res = NotifyArrival.Response()
        return self.handle_notify_arrival(req, res)
    

    def handle_go_to_room(self,request,response):
        permission = request.permission
        log_msg = f'🚑 [permission] 도착 통보 요청 수신 (permission={permission})'
        self.get_logger().info(log_msg)
        if self.gui_log_callback:
            self.gui_log_callback(log_msg)
            self.gui_log_callback('✅ 도착 통보 응답: permission=True')

        response.accepted = True
        return response
    
    def simulate_go_to_room(self,permission):
        req = GoToRoom.Request()
        req.permission = permission
        res = GoToRoom.Response()
        return self.handle_go_to_room(req, res)
    

    #################ROBOT4########################
    def handle_robot4_rendezvous(self,request,response):
        data = request.data
        log_msg = f'🚑 [data] 도착 통보 요청 수신 (data={data})'
        self.get_logger().info(log_msg)
        if self.gui_log_callback:
            self.gui_log_callback(log_msg)
            self.gui_log_callback('✅ 도착 통보 응답: data=True')
        response.success = True
        return response
        
    def simulate_robot4_rendezvous(self, data):
        req = EndFlag.Request()
        req.data = data
        res = EndFlag.Response()
        return self.handle_robot4_rendezvous(req,res)


class PatrolControlGUI(QWidget):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.ros_node.register_gui_logger(self.log)

        self.setWindowTitle('순찰 로봇 제어 GUI')
        self.setGeometry(100, 100, 600, 800)

        self.log_box = QTextEdit(self)
        self.log_box.setReadOnly(True)

        self.btn_assign_patient = QPushButton('환자 배정 (AssignPatient)', self)
        self.btn_notify_arrival = QPushButton('도착 통보 (NotifyArrival)', self)
        self.btn_go_to_room = QPushButton('병실 이동 허가 (GoToRoom)', self)
        self.label_input = QLineEdit(self)
        self.label_input.setPlaceholderText('배송할 물품명을 입력하세요 (예: bandage)')
        self.btn_robot4_object = QPushButton('배송 물품 요청 (object)', self)
        self.btn_robot4_rendezvous = QPushButton('랑데부 포인트 (rendezvous)', self)
        self.btn_robot4_meet = QPushButton('로봇 만남 완료(meet)', self)
        self.btn_exit = QPushButton('종료', self)

        layout = QVBoxLayout()
        layout.addWidget(QLabel('🧠 순찰 로봇 서비스 제어'))
        layout.addWidget(self.btn_assign_patient)
        layout.addWidget(self.btn_notify_arrival)
        layout.addWidget(self.btn_go_to_room)
        layout.addWidget(self.label_input)
        layout.addWidget(self.btn_robot4_object)
        layout.addWidget(self.btn_robot4_rendezvous)
        layout.addWidget(self.btn_robot4_meet)
        layout.addWidget(self.log_box)
        layout.addWidget(self.btn_exit)
        self.setLayout(layout)

        self.btn_assign_patient.clicked.connect(self.assign_patient)
        self.btn_notify_arrival.clicked.connect(self.notify_arrival)
        self.btn_go_to_room.clicked.connect(self.go_to_room)

        self.btn_robot4_rendezvous.clicked.connect(self.robot4_rendezvous)
        self.btn_robot4_object.clicked.connect(self.robot4_object)
        self.btn_robot4_meet.clicked.connect(self.robot4_meet)
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

    def notify_arrival(self):
        patient_id = 1
        self.log(f'🔘 도착 통보 버튼 클릭됨 (patient_id={patient_id})')

        response = self.ros_node.simulate_notify_arrival(patient_id)

        if response.ack:
            self.log('✅ 도착 통보 처리 완료: ack=True')
        else:
            self.log('❌ 도착 통보 처리 실패: ack=False')

        return response  

    def go_to_room(self):
        permission = True
        self.log(f'🔘 병실 허가 요청 (permission={permission})')
        req = GoToRoom.Request()
        req.permission = True

        response = self.ros_node.simulate_go_to_room(permission)

        if response.accepted:
            self.log('✅ 도착 통보 처리 완료: ack=True')
        else:
            self.log('❌ 도착 통보 처리 실패: ack=False')

        return response 
    
    def robot4_object(self):
        label = self.label_input.text().strip()
        if not label:
            self.log('❗ 물품명을 입력하세요.')
            return

        self.log(f"🚚 물건 배달 요청 중... (label='{label}')")
        req = Firstcmd.Request()
        req.label = label  # 👉 입력값을 서비스 요청에 넣음

        def callback(success, response):
            if success and response.success:
                self.log(f"✅ 물건 '{label}' 배달 처리 완료")
            else:
                self.log(f"❌ 물건 '{label}' 배달 처리 실패")

        self.ros_node.call_service(self.ros_node.robot4_object_client, req, callback)

    def robot4_meet(self):
        self.log('랑데부 포인트 만남 요청 중...')
        req = EndFlag.Request()

        def callback(success, response):
            if success and response.success:
                self.log('✅ 랑데부 포인트 만남처리 완료 ')
            else:
                self.log('랑데부 포인트 만남 처리 실패')

        self.ros_node.call_service(self.ros_node.robot4_meet_client, req, callback)

    def robot4_rendezvous(self):
        data = True
        self.log(f'🔘 랑데뷰 도착 여부 :  (data={data})')
        req = EndFlag.Request()
        req.data = True

        response = self.ros_node.simulate_robot4_rendezvous(data)

        if response.success:
            self.log('✅ 도착 통보 처리 완료: success=True')
        else:
            self.log('❌ 도착 통보 처리 실패: success=False')

        return response  


def main():
    rclpy.init()
    ros_node = PatrolControlNode(namespace='/robot1')

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
