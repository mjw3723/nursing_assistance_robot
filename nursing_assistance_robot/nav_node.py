#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator
from irobot_create_msgs.msg import AudioNoteVector, AudioNote
from builtin_interfaces.msg import Duration
from tf_transformations import quaternion_from_euler
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
import time
from rokey_interfaces.srv import AssignPatient, NotifyArrival, GoToRoom
from enum import Enum
from nursing_assistance_robot.cloud import Clound
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
        self.dock_navigator = TurtleBot4Navigator()
        self.nav_navigator = BasicNavigator()

        self.state = State.WAIT_ID
        self.patient_id = None
        self.person_detected = False
        self.should_resume = False
        # 감지 토픽 구독
        self.create_subscription(Bool, '/person_detected', self.person_callback, 10)
        self.create_subscription(Bool, '/person_cleared', self.clear_callback, 10)
        self.audio_publisher = self.create_publisher(AudioNoteVector, '/robot1/cmd_audio', 10)
        self.waypoints = []
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
        self.clound = Clound()
        self.clound.publish(0.0)

        self.waypoints = [
            self.create_pose(4.09, 0.89, 180.0),
            self.create_pose(1.06, 0.75, 90.0),
            self.create_pose(0.05, 0.05, 0.0)
        ]
        self.create_timer(1.0, self.run) 

    def person_callback(self, msg):
        if msg.data and not self.person_detected:
            self.get_logger().info('사람 감지됨! 주행 중단')
            self.person_detected = True
            self.nav_navigator.cancelTask()

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

    def create_pose(self, x, y, yaw_deg):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.nav_navigator.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        yaw_rad = yaw_deg * 3.141592 / 180.0
        q = quaternion_from_euler(0, 0, yaw_rad)
        pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = q
        return pose
    
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
        response.accepted = True
        return response

    def init_robot(self):
        initial_pose = self.create_pose(-0.01, -0.01, 0.0)
        self.nav_navigator.setInitialPose(initial_pose)
        self.get_logger().info('초기 위치 설정 중...')
        time.sleep(1.0)
        self.nav_navigator.waitUntilNav2Active()
        if self.dock_navigator.getDockedStatus():
            self.get_logger().info('도킹 상태 → 언도킹')
            self.dock_navigator.undock()
        self.state = State.WAIT_PERMISSION
        self.get_logger().info('State = WAIT_PERMISSION')
        self.nav_go_pose(0)
        self.clound.publish(0.0)
        

    def destroy_robot(self):
        self.start_audio()
        self.dock_navigator.dock()
        self.get_logger().info('도킹 요청 완료')

    def nav_go_pose(self,position):
        if self.should_resume:
            self.should_resume = False
            self.get_logger().info('사람 사라짐 → 경로 재실행')
        self.nav_navigator.goToPose(self.waypoints[position])
        while not self.nav_navigator.isTaskComplete():
            if self.person_detected:
                self.get_logger().info('사람 감지! 경로 취소 중...')
                self.nav_navigator.cancelTask()
                break
            feedback = self.nav_navigator.getFeedback()
            if feedback:
                self.get_logger().info(f'남은 거리: {feedback.distance_remaining:.2f}m')
            rclpy.spin_once(self, timeout_sec=0.5)
        self.get_logger().info(f'Waypoint {position + 1} 도달 완료')

    def detect(self):
        self.nav_navigator.cancelTask()
        
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
            self.nav_go_pose(1)
            self.task_room()
        elif self.state == State.TO_DOCK:
            self.task_dock()
            self.destroy_robot()

    def task_rendezvous(self):
        self.nav_navigator.cancelTask()
        self.get_logger().info('❌ nav2 작업 취소됨. ')
        if self.arrival_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('-- 도착 완료 알림 서비스 실행 --')
            req = NotifyArrival.Request()
            req.patient_id = self.patient_id
            future = self.arrival_client.call_async(req)
            future.add_done_callback(self.handle_notify_arrival_response)

    def task_waitRoom(self):
        self.get_logger().info("병실 이동 허가 대기 중...")

    def handle_notify_arrival_response(self, future):
        res = future.result()
        if res.ack:
            self.get_logger().info("✅ 허가 수신 → 수락 --- 도착 여부 송신 완료")
            self.state = State.WAIT_ROOM
            self.get_logger().info("STATE = WAIT_ROOM")
            self.start_audio()
        else:
            self.get_logger().info("✅ 허가 수신 → 거절")

    def handle_permission_response(self,future):
        res = future.result()
        if res.accepted:
            self.get_logger().info("✅ 허가 수신 → 수락")
            self.state = State.TO_ROOM
        else:
            self.get_logger().info("✅ 허가 수신 → 거절")

    def task_room(self):
        self.nav_navigator.cancelTask()
        self.get_logger().info('❌ nav2 작업 취소됨. 로봇 회전 중...')
        self.spin_robot(0.3,1.4)
        self.wait_robot(3.0)
        for i in range(3):
            self.spin_robot(-0.3,1.3)
            self.wait_robot(5.0)
        self.get_logger().info('✅ 회전 완료! nav2 다시 실행')
        self.state = State.TO_DOCK
        self.get_logger().info("STATE = TO_DOCK")
        self.nav_go_pose(2)
    
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
        self.start_audio(random.randint(1,6))
        self.state = State.WAIT_ID

    def start_audio(self):
        state = random.randint(1,6)
        notes = []
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