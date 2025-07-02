#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from irobot_create_msgs.msg import AudioNoteVector, AudioNote
from builtin_interfaces.msg import Duration
from tf_transformations import quaternion_from_euler
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from paho.mqtt import client as mqtt_client
import time
from rokey_interfaces.srv import AssignPatient, NotifyArrival, GoToRoom
from enum import Enum
# from rokey_interfaces.msg import Aruco_Marker
import random

class State(Enum):
    WAIT_ID = 0
    TO_RENDEZVOUS = 1
    WAIT_PERMISSION = 2
    WAIT_ROOM = 3
    TO_ROOM = 4
    TO_DOCK = 5

class PatrolNavigator(Node):
    def __init__(self):
        super().__init__('patrol_navigator')
        self.state = State.WAIT_ID
        self.patient_id = None
        self.person_detected = False
        self.should_resume = False
        # 감지 토픽 구독
        self.create_subscription(Bool, '/person_detected', self.person_callback, 10)
        self.create_subscription(Bool, '/person_cleared', self.clear_callback, 10)
        self.audio_publisher = self.create_publisher(AudioNoteVector, '/robot1/cmd_audio', 10)
        self.current_index = 0
        #서비스 설정
        self.get_logger().info('서비스 등록 대기')
        self.assign_patient_service = self.create_service(AssignPatient, 'assign_patient', self.assign_patient_cb)
        self.get_logger().info(f'assign_patient 서비스 등록 완료: {self.assign_patient_service is not None}')
        self.arrival_client = self.create_client(NotifyArrival, 'notify_arrival')
        self.permission_client = self.create_service(GoToRoom, 'go_to_room',self.go_to_room_cb)
        #self.yolo_client = self.create_client(CheckDetection, 'check_detection')
        self.cmd_vel_pub = self.create_publisher(Twist, '/robot1/cmd_vel', 10)
        # self.marker_sub = self.create_subscription(Aruco_Marker, '/aruco_marker',self.aruco_callback,10)
        self.waypoints = [1,2,3]
        self.create_timer(1.0, self.run) 

    def person_callback(self, msg):
        if msg.data and not self.person_detected:
            self.get_logger().info('사람 감지됨! 주행 중단')
            self.person_detected = True

    def clear_callback(self, msg):
        if msg.data and self.person_detected:
            self.get_logger().info('사람 사라짐! 주행 재개 예정')
            self.person_detected = False
            self.should_resume = True

    def aruco_callback(self,msg):
        self.marker_id = msg.id
        self.pos_x = msg.pos_x
        self.pos_y = msg.pos_y
        self.pos_z = msg.pos_z
        self.rot_x = msg.rot_x
        self.rot_y = msg.rot_y
        self.rot_z = msg.rot_z


    
    #환자 ID 수신 메서드
    def assign_patient_cb(self, request, response):
        self.current_index = 0
        self.patient_id = request.patient_id
        self.get_logger().info(f"환자 ID {self.patient_id} 수신됨 → 랑데뷰 포인트 이동 준비")
        self.state = State.TO_RENDEZVOUS
        response.success = True
        return response
    
    def go_to_room_cb(self,request,response):
        self.permission = request.permission
        self.get_logger().info(f"{self.permission} 수신됨 → 병실 포인트 이동 준비")
        self.state = State.TO_ROOM
        self.current_index+=1
        response.accepted = True
        return response
        
    def init_robot(self):
        self.get_logger().info('초기 위치 설정 중...')
        self.state = State.WAIT_PERMISSION
        self.start_audio()

    def destroy_robot(self):
        self.get_logger().info('도킹 요청 완료')

    def nav_go_pose(self):
        self.get_logger().info('dsadsadsa')
        
        # while self.current_index < len(self.waypoints):
        #     if self.state == State.TO_RENDEZVOUS or self.State.WAIT_PERMISSION:
        #         self.task_rendezvous()
        #     if self.state ==  State.TO_ROOM:
        #         self.task_room()
        #     if self.state == State.TO_DOCK:
        #         self.task_dock()
        
    def run(self):
        if self.state == State.WAIT_ID:
            return  # 아무것도 안 함
        if self.state == State.TO_RENDEZVOUS:
            self.init_robot()
        elif self.state == State.WAIT_PERMISSION:
            self.task_rendezvous()
        elif self.state == State.WAIT_ROOM:
            self.task_waitRoom()
        elif self.state == State.TO_ROOM:
            self.task_room()
        elif self.state == State.TO_DOCK:
            self.task_dock()
            self.destroy_robot()

    def task_rendezvous(self):
        if self.arrival_client.wait_for_service(timeout_sec=5.0):
            req = NotifyArrival.Request()
            req.patient_id = self.patient_id
            future = self.arrival_client.call_async(req)
            future.add_done_callback(self.handle_notify_arrival_response)
    
    def task_waitRoom(self):
        self.get_logger().info("병실 이동 허가 대기 중...")

    def handle_notify_arrival_response(self, future):
        res = future.result()
        if res.ack:
            self.get_logger().info("✅ 허가 수신 → 수락")
            self.get_logger().info("도착 여부 송신 완료")
            self.state = State.WAIT_ROOM
            self.start_audio()
        else:
            self.get_logger().info("✅ 허가 수신 → 거절")


    def task_room(self):
        self.get_logger().info('❌ nav2 작업 취소됨. 로봇 회전 중...')
        self.spin_robot(0.3,1.4)
        self.wait_robot(3.0)
        for i in range(4):
            self.spin_robot(-0.3,1.3)
            self.wait_robot(5.0)
        self.get_logger().info('✅ 회전 완료! nav2 다시 실행')
        self.state = State.TO_DOCK  
    
    def wait_robot(self,count):
        wait_start = time.time()
        while time.time() - wait_start < count:
            rclpy.spin_once(self, timeout_sec=0.1)

    

    def spin_robot(self,angular,duration):
        twist = Twist()
        twist.angular.z = angular
        start_time = time.time()
        duration = 1.4 
        while time.time() - start_time < duration:
            self.cmd_vel_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.1)

    def task_dock(self):
        self.state = State.WAIT_ID

    def start_audio(self):
        notes = []
        state = random.randint(1,6)
        if state == 1:
            notes = [
                (659, 0.125), (659, 0.125), (0, 0.125), (659, 0.125), (0, 0.125),
                (523, 0.125), (659, 0.125), (0, 0.125), (784, 0.125), (0, 0.375),
                (392, 0.125), (0, 0.375),
                (523, 0.125), (0, 0.25), (392, 0.125), (0, 0.25),
                (330, 0.125), (0, 0.125), (440, 0.125), (0, 0.125),
                (494, 0.125), (0, 0.125), (466, 0.125), (0, 0.125),
                (440, 0.125), (0, 0.125), (392, 0.125), (659, 0.125),
                (784, 0.125), (880, 0.25), (698, 0.125), (784, 0.125),
                (659, 0.125), (523, 0.125), (587, 0.125), (494, 0.125),
            ]
        elif state == 2:
            notes = [
                (440, 0.2), (494, 0.2), (523, 0.2), (587, 0.2),
                (659, 0.2), (698, 0.2), (784, 0.3), (880, 0.3), (784, 0.2),
                (698, 0.2), (659, 0.2), (587, 0.2), (523, 0.2), (494, 0.2), (440, 0.3),
                (0, 0.2),
                (440, 0.2), (494, 0.2), (523, 0.2), (587, 0.2),
                (659, 0.2), (698, 0.2), (784, 0.3), (880, 0.3), (784, 0.2),
                (698, 0.2), (659, 0.2), (587, 0.2), (523, 0.2), (494, 0.2), (440, 0.3),
            ]
        elif state == 3:
            notes = [
                (659, 0.2), (622, 0.2), (659, 0.2), (622, 0.2), (659, 0.2), (494, 0.2), (587, 0.2), (523, 0.2),
                (440, 0.4), (0, 0.2), (262, 0.2), (330, 0.2), (440, 0.2), (494, 0.2),
                (330, 0.2), (415, 0.2), (494, 0.2), (523, 0.2), (330, 0.2),
                (659, 0.2), (622, 0.2), (659, 0.2), (622, 0.2), (659, 0.2), (494, 0.2), (587, 0.2), (523, 0.2)
            ]
        elif state == 4:
            notes = [
                (440, 0.3), (392, 0.3), (440, 0.3), (392, 0.3), (440, 0.3), (349, 0.3), (392, 0.3), (440, 0.3),  # 루돌프 사슴코는
                (392, 0.3), (440, 0.3), (392, 0.3), (440, 0.3), (349, 0.3), (392, 0.3), (440, 0.3),             # 매우 반짝이는 코~
                (440, 0.3), (440, 0.3), (440, 0.3), (392, 0.3), (349, 0.3), (440, 0.3),                         # 만일 네가 봤다면
                (392, 0.3), (440, 0.3), (392, 0.3), (349, 0.3), (392, 0.6),                                     # 분명 놀랐을 거야~

                (392, 0.3), (392, 0.3), (392, 0.3), (440, 0.3), (392, 0.3), (349, 0.3),                         # 다른 순록들하고
                (392, 0.3), (440, 0.3), (392, 0.3), (440, 0.3), (392, 0.3),                                     # 놀지 못했지만
                (440, 0.3), (494, 0.3), (523, 0.3), (494, 0.3), (440, 0.3),                                     # 루돌프 사슴코가
                (392, 0.3), (440, 0.6), (0, 0.2),                                                               # 내일 밤에 말했대~
            ]
        elif state == 5:
            notes = [
                (659, 0.2), (784, 0.2), (880, 0.2), (988, 0.2), (880, 0.2), (988, 0.2), (880, 0.2),
                (784, 0.2), (659, 0.2), (784, 0.2), (880, 0.2), (784, 0.2), (659, 0.2),
                (523, 0.2), (659, 0.2), (784, 0.2), (659, 0.2), (523, 0.2),

                (440, 0.3), (440, 0.2), (440, 0.2), (523, 0.2), (659, 0.2),
                (784, 0.3), (880, 0.2), (988, 0.2), (880, 0.2),
                (784, 0.2), (659, 0.2), (784, 0.2), (880, 0.2), (784, 0.2), (659, 0.2),
                (523, 0.2), (659, 0.2), (784, 0.2), (659, 0.2), (523, 0.3),
            ]

        audio_vector_msg = AudioNoteVector()
        audio_vector_msg.append = False
        for freq, sec in notes:
            note = AudioNote()
            note.frequency = int(freq)
            note.max_runtime = Duration(sec=0, nanosec=int(sec * 1e9))
            audio_vector_msg.notes.append(note)
        self.audio_publisher.publish(audio_vector_msg)

def main():
    rclpy.init()
    node = None
    node = PatrolNavigator()
    try:
        rclpy.spin(node) 
    except KeyboardInterrupt:
        print("KeyboardInterrupt 발생 – 노드 종료합니다.")
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()