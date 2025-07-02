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
import random

class PatrolNavigator(Node):
    def __init__(self):
        super().__init__('patrol_navigator')
        self.dock_navigator = TurtleBot4Navigator()
        self.nav_navigator = BasicNavigator()
        self.assign_patient_service = self.create_service(AssignPatient, 'assign_patient', self.assign_patient_callback)
        self.arrival_client = self.create_client(NotifyArrival, 'notify_arrival')
        self.permission_service = self.create_service(GoToRoom, 'go_to_room',self.go_to_room_callback)
        self.person_detected = False
        self.should_resume = False
        self.create_subscription(Bool, '/person_detected', self.person_callback, 10)
        self.create_subscription(Bool, '/person_cleared', self.clear_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/robot1/cmd_vel', 10)
        
        # self.waypoints = [
        #     self.create_pose(4.09, 0.89, 180.0),
        #     self.create_pose(1.06, 0.75, 90.0),
        #     self.create_pose(0.05, 0.05, 0.0)
        # ] 
    

    def assign_patient_callback(self,request,response):
        self.patient_id = request.patient_id
        self.get_logger().info(f"환자 ID {self.patient_id} 수신됨 → 랑데뷰 포인트 이동 준비")
        
        # self.nav_navigator.waitUntilNav2Active()
        self.get_logger().info('Nav2 활성화 확인 완료')

        # initial_pose = self.create_pose(-0.01, -0.01, 0.0)
        # self.nav_navigator.setInitialPose(initial_pose)
        self.get_logger().info('초기 위치 설정 중...')
        time.sleep(1.0)

        # if self.dock_navigator.getDockedStatus():
        #     self.get_logger().info('도킹 상태 → 언도킹')
        #     self.dock_navigator.undock()

        self.get_logger().info('랑데뷰 으로 이동')
        #self.nav_go_pose(0)
        self.get_logger().info('랑데뷰 으로 이동 완료')
        req = NotifyArrival.Request()
        req.patient_id = self.patient_id
        self.get_logger().info('도착 서비스 요청')
        future = self.arrival_client.call_async(req)
        future.add_done_callback(self.handle_notify_arrival_response)
        self.get_logger().info('도착 서비스 완려')
        response.success = True
        return response
    
    def handle_notify_arrival_response(self, future):
        res = future.result()
        if res.ack:
            self.get_logger().info("✅ 허가 수신 → 수락 --- 도착 여부 송신 완료")
        else:
            self.get_logger().info("✅ 허가 수신 → 거절")

    def go_to_room_callback(self,request,response):
        self.permission = request.permission
        self.get_logger().info(f"{self.permission} 수신됨 → 병실 포인트 이동 준비")
        self.get_logger().info('병동 으로 이동')
        self.get_logger().info('병동 으로 이동 완료')
        
        self.search_face()
        response.accepted = True
        return response
    
    def search_face(self):
        self.get_logger().info('search_face')
        # self.spin_robot(0.3,1.4)
        # self.wait_robot(3.0)
        # for i in range(4):
        #     self.spin_robot(-0.3,1.3)
        #     self.wait_robot(5.0)
    
    def wait_robot(self,count):
        wait_start = time.time()
        while time.time() - wait_start < count:
            rclpy.spin_once(self, timeout_sec=0.1)

    def spin_robot(self,angular,duration):
        twist = Twist()
        twist.angular.z = angular
        start_time = time.time() 
        while time.time() - start_time < duration:
            self.cmd_vel_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.1)



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


def main(args=None):
    rclpy.init(args=args)
    node = PatrolNavigator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()