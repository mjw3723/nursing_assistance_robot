#!/usr/bin/env python3
import sys
import time
import threading
import rclpy
from rclpy.node import Node
from PyQt5.QtCore import QThread
from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QPushButton, QVBoxLayout, QTextEdit,QProgressBar,QLineEdit
from geometry_msgs.msg import PoseWithCovarianceStamped
from rokey_interfaces.srv import AssignPatient, NotifyArrival, GoToRoom
from std_msgs.msg import String
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float32
from PyQt5.QtWidgets import QTableWidget, QTableWidgetItem
from PyQt5.QtGui import QImage, QPixmap
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os
os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"] = "/usr/lib/x86_64-linux-gnu/qt5/plugins/platforms"
class PatrolControlNode(Node):
    def __init__(self,namespace=''):
        ##########namespace 지정 #####################################
        super().__init__('patrol_control_node',namespace=namespace)
        self.bridge = CvBridge()
        ############ROBOT1##########################
        self.assign_patient_client = self.create_client(AssignPatient, 'assign_patient')
        self.notify_arrival_service = self.create_service(
            NotifyArrival,
            'notify_arrival',
            self.handle_notify_arrival
        )
        self.go_to_room_client = self.create_client(GoToRoom, 'go_to_room')
        self.amcl_x = 0.0
        self.amcl_y = 0.0
        self.cloud_pub = self.create_publisher(String, '/mqtt_sub', 10)
        self.cloud_sub = self.create_subscription(String, '/mqtt_received',self.mqtt_sub_callback ,10)
        ############ROBOT4##########################

        self.gui_log_callback = None  # GUI 로그 출력을 위한 콜백
        self.battery_callback = None
        self.position_state_callback = None
        self.image_callback = None
        self.amcl_subscription = self.create_subscription(
                PoseWithCovarianceStamped,
                '/robot1/amcl_pose',
                self.amcl_listener_callback,
                10
        )
        self.bpm_subscription = self.create_subscription(Float32,'/bpm',self.bpm_listner_callback,10)
        self.spo2_subscription = self.create_subscription(Float32, '/spo2',self.spo2_listner_callback, 10)
        self.sbp_subscription = self.create_subscription(Float32, '/sbp', self.sbp_listner_callback ,10)
        self.dbp_subscription = self.create_subscription(Float32, '/dbp', self.dbp_listner_callback,10)
        self.position_subscription = self.create_subscription(String,'/position',self.state_callback,10)
        self.create_subscription(
            BatteryState,
            '/robot1/battery_state',
            self.battery_listener_callback,
            10
        )  
        self.image_sub = self.create_subscription(  
            Image,
            '/rppg_image',
            self.image_listener_callback,
            10
        )
        self.id = 1

    def update_id(self,id):
        self.id = int(id)
        ##########
    def bpm_listner_callback(self,msg):
        if self.id is not None:
            self.gui_log_callback(f'{self.id}번 환자 bpm ------- {str(msg.data)}')
    
    def spo2_listner_callback(self,msg):
        if self.id is not None:
            self.gui_log_callback(f'{self.id}번 환자 spo2 ------- {str(msg.data)}')

    def sbp_listner_callback(self,msg):
        if self.id is not None:
            self.gui_log_callback(f'{self.id}번 환자 sbp ------- {str(msg.data)}')

    def dbp_listner_callback(self,msg):
        if self.id is not None:
            self.gui_log_callback(f'{self.id}번 환자 dbp ------- {str(msg.data)}')


    
    def state_callback(self,msg):
        self.position_state_callback(f'{msg}')

    def mqtt_sub_callback(self,msg):
        self.get_logger().info(f'mqtt sub = {msg.data}')
        self.room_call()

    def battery_listener_callback(self,msg:BatteryState):
        percentage = msg.percentage
        self.battery_callback(int(percentage * 100))

    def amcl_listener_callback(self,msg):
        self.amcl_x = msg.pose.pose.position.x
        self.amcl_y = msg.pose.pose.position.y
        self.get_logger().info(f"📍 수신된 AMCL 위치: x={self.amcl_x:.2f}, y={self.amcl_y :.2f}")

    def register_gui_logger(self, log_func):
        self.gui_log_callback = log_func

    def register_battery(self,battery):
        self.battery_callback = battery

    def register_state(self,state):
        self.position_state_callback = state

    def register_image_callback(self, callback):
        self.image_callback = callback

    def image_listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            if self.image_callback:
                self.image_callback(cv_image)
        except Exception as e:
            self.get_logger().error(f'이미지 변환 오류: {e}')

    def room_call(self):
        permission = True
        self.get_logger().info(f'🏥 병원 이동 요청 (permission={permission})')
        req = GoToRoom.Request()
        req.permission = permission
        def callback(success, response):
            if success and response.accepted:
                self.get_logger().info('✅ 환자 병실 이동 허가됨!')
                self.gui_log_callback('✅ 환자 병실 이동 허가됨!')
                self.gui_log_callback('📡 약 받은 후 병동으로 이동합니다.')
            else:
                self.get_logger().info('❌ 병실 이동 실패')

        self.call_service(self.go_to_room_client, req, callback)

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
        self.gui_log_callback('병동 이동 허가 대기중 ..')
        
        msg = String()
        msg.data = str(self.amcl_x) + ',' + str(self.amcl_y)
        self.cloud_pub.publish(msg)
        response.ack = True
        return response
        

class PatrolControlGUI(QWidget):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.ros_node.register_gui_logger(self.log)
        self.ros_node.register_battery(self.battery)
        self.ros_node.register_state(self.position_state)
        self.setWindowTitle('🏥 병원 로봇 모니터링 시스템')
        self.setGeometry(100, 100, 1000, 1000)
        self.setFixedSize(1000,1000)
        self.setStyleSheet("""
            QWidget {
                font-family: 'Nanum Gothic';
                font-size: 16px;
                background-color: #f5f9ff;
            }

            QLabel {
                color: #2c3e50;
                font-weight: bold;
            }

            QLineEdit {
                background-color: #ffffff;
                border: 1px solid #cccccc;
                border-radius: 6px;
                padding-left: 10px;
                font-size: 16px;
            }

            QPushButton {
                background-color: #3498db;
                color: white;
                border-radius: 8px;
                font-size: 15px;
                font-weight: bold;
                padding: 8px;
            }

            QPushButton:hover {
                background-color: #2980b9;
            }

            QTextEdit {
                background-color: #ffffff;
                border: 1px solid #cccccc;
                border-radius: 8px;
                padding: 10px;
                font-size: 15px;
            }

            QProgressBar {
                border: 1px solid #aaa;
                border-radius: 8px;
                text-align: center;
                font-weight: bold;
                background-color: #ecf0f1;
            }

            QProgressBar::chunk {
                background-color: #1abc9c;
                border-radius: 8px;
            }
        """)

        self.log_box = self.create_textedit(10,400,490,540)
        self.create_label('환자 ID ',10,10,90,50)
        self.paitient_label = self.create_lineedit(100,10,100,50)
        self.btn_assign_patient = self.create_button('환자 배정 (AssignPatient)',210,10,220,50)
        
        self.btn_go_to_room = self.create_button('병실 이동 허가 (GoToRoom)',460,10,220,50)
        self.btn_exit = self.create_button('종료',10,940,980,50)

        self.create_label('현재 로봇 위치',10,200,200,50)
        self.position_label = self.create_lineedit(10,250,200,50)
        self.position_label.setReadOnly(True)
        
        self.create_label('Robot Battery',10,300,200,30)
        self.battery_progress = self.create_progress(10,330,800,50)
        self.position_label.setText('대기중..')

                # __init__ 안에 추가 (예: 10, 70 위치에 표시)
        self.create_label('📋 환자 목록', 230, 70, 200, 30)
        self.patient_table = self.create_table(230, 100, 600, 200)
        self.patient_table.cellClicked.connect(self.on_table_click)

        self.btn_assign_patient.clicked.connect(lambda:self.assign_patient(self.paitient_label.text()))
        self.btn_go_to_room.clicked.connect(self.room)
        self.btn_exit.clicked.connect(self.close)

        self.image_label = QLabel(self)
        self.image_label.setGeometry(510, 400, 490, 540)  # 크기/위치 조절
        self.image_label.setStyleSheet("border: 1px solid #ccc; background-color: #eee;")
        self.image_label.setScaledContents(True)
        self.ros_node.register_image_callback(self.update_image)

    def update_image(self, cv_img):
        height, width, channel = cv_img.shape
        bytes_per_line = 3 * width
        q_img = QImage(cv_img.data, width, height, bytes_per_line, QImage.Format.Format_RGB888).rgbSwapped()
        pixmap = QPixmap.fromImage(q_img)
        self.image_label.setPixmap(pixmap)
        
    def on_table_click(self, row, column):
        patient_number_item = self.patient_table.item(row, 0)  # 0은 첫 번째 열
        if patient_number_item:
            patient_number = patient_number_item.text()
            self.paitient_label.setText(patient_number)

    def create_textedit(self,x,y,w,h):
        textedit = QTextEdit(self)
        textedit.setReadOnly(True)
        textedit.setGeometry(x,y,w,h)
        return textedit

    def create_button(self,text,x,y,w,h):
        button = QPushButton(self)
        button.setText(text)
        button.setGeometry(x,y,w,h)
        return button
    
    def create_lineedit(self,x,y,w,h):
        lineedit = QLineEdit(self)
        lineedit.setText('')
        lineedit.setGeometry(x,y,w,h)
        return lineedit
        
    def create_label(self,text,x,y,w,h):
        label = QLabel(self)
        label.setText(text)
        label.setGeometry(x,y,w,h)
        return label
    
    def create_progress(self,x,y,w,h):
        progress = QProgressBar(self)
        progress.setValue(0)
        progress.setMaximum(100)
        progress.setGeometry(x,y,w,h)
        return progress
    
    def create_table(self, x, y, w, h):
        table = QTableWidget(self)
        table.setGeometry(x, y, w, h)
        table.setColumnCount(3)
        table.setHorizontalHeaderLabels(['환자 번호', '이름', '전화번호'])
        table.setRowCount(0)  # 처음엔 비워놓기
        table.setColumnWidth(0, 200)
        table.setColumnWidth(1, 200)
        table.setColumnWidth(2, 200)

        patients = [
            ('1', '정민섭', '010-1234-5678'),
            ('2', '이영희', '010-2345-6666'),
            ('3', '최정호', '010-0000-1111'),
            ('4', '문준웅', '010-2222-1111'),
            ('5', '이경민', '010-3333-4444'),
        ]

        table.setRowCount(len(patients))

        for row, (pid, name, phone) in enumerate(patients):
            table.setItem(row, 0, QTableWidgetItem(pid))
            table.setItem(row, 1, QTableWidgetItem(name))
            table.setItem(row, 2, QTableWidgetItem(phone))
        table.setStyleSheet("""
            QTableWidget {
                background-color: #ffffff;
                border: 1px solid #ccc;
                border-radius: 6px;
                font-size: 15px;
            }
            QHeaderView::section {
                background-color: #3498db;
                color: white;
                padding: 4px;
                font-weight: bold;
                border: 1px solid #2980b9;
            }
        """)
        return table

    def log(self, msg):
        self.log_box.append(msg)

    def battery(self,msg):
        self.battery_progress.setValue(int(msg))

    def position_state(self,msg):
        self.position_label.setText(str(msg))

    def assign_patient(self,id):
        patient_id = int(id)
        self.log(f'환자 배정 요청 (환자 ID = {patient_id})')
        req = AssignPatient.Request()
        req.patient_id = patient_id

        def callback(success, response):
            if success and response.success:
                self.log('✅ 환자 배정 성공 ! 조제실로 이동합니다 !')
                self.position_state('조제실')
            else:
                self.log('❌ 환자 배정 실패')

        self.ros_node.call_service(self.ros_node.assign_patient_client, req, callback)

    def room(self):
        permission =  True
        self.log(f'병동 이동 요청 수신')
        req = GoToRoom.Request()
        req.permission = permission

        def callback(success, response):
            if success and response.accepted:
                self.log('✅ 환자에게 이동합니다 !')
                self.position_state('병동')
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
