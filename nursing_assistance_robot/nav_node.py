#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator
from tf_transformations import quaternion_from_euler
from std_msgs.msg import Bool
import time
class PatrolNavigator(Node):
    def __init__(self):
        super().__init__('patrol_navigator')
        self.dock_navigator = TurtleBot4Navigator()
        self.nav_navigator = BasicNavigator()
        # 사람 감지 상태 변수
        self.person_detected = False
        self.should_resume = False
        # 감지 토픽 구독
        self.create_subscription(Bool, '/person_detected', self.person_callback, 10)
        self.create_subscription(Bool, '/person_cleared', self.clear_callback, 10)
        self.waypoints = []
        self.current_index = 0
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
    def create_pose(self, x, y, yaw_deg):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.nav_navigator.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        yaw_rad = yaw_deg * 3.141592 / 180.0
        q = quaternion_from_euler(0, 0, yaw_rad)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        return pose
    def run(self):
        initial_pose = self.create_pose(-0.01, -0.01, 0.0)
        self.nav_navigator.setInitialPose(initial_pose)
        self.get_logger().info('초기 위치 설정 중...')
        time.sleep(1.0)
        self.nav_navigator.waitUntilNav2Active()
        if self.dock_navigator.getDockedStatus():
            self.get_logger().info('도킹 상태 → 언도킹')
            self.dock_navigator.undock()
        self.waypoints = [
            self.create_pose(4.09, 0.89, 0.0),
            self.create_pose(1.06, 0.75, 180.0),
            self.create_pose(-0.01, -0.01, 0.0)
        ]
        while self.current_index < len(self.waypoints):
            self.get_logger().info(f'{self.current_index + 1}번째 경로 이동 중...')
            self.nav_navigator.goToPose(self.waypoints[self.current_index])
            while not self.nav_navigator.isTaskComplete():
                if self.person_detected:
                    self.get_logger().info('정지 상태 대기 중...')
                    rclpy.spin_once(self, timeout_sec=0.5)
                    continue
                feedback = self.nav_navigator.getFeedback()
                if feedback:
                    self.get_logger().info(f'남은 거리: {feedback.distance_remaining:.2f}m')
                rclpy.spin_once(self, timeout_sec=0.5)
            if not self.person_detected:
                self.get_logger().info(f'Waypoint {self.current_index + 1} 도달 완료')
                self.current_index += 1
            else:
                self.get_logger().info('중단됨, 현재 인덱스를 유지합니다')
            # 중단 후 재개
            while self.person_detected:
                rclpy.spin_once(self, timeout_sec=0.5)
        self.dock_navigator.dock()
        self.get_logger().info('도킹 요청 완료')
def main():
    rclpy.init()
    node = PatrolNavigator()
    node.run()
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()