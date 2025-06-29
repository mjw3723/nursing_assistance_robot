# stop_client.py
import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import Stop  # 형님이 만든 서비스 타입

class StopClientNode(Node):
    def __init__(self):
        super().__init__('stop_client')
        self.client = self.create_client(Stop, 'stop')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('서비스 기다리는 중...')
        self.send_request()

    def send_request(self):
        req = Stop.Request()
        req.person_detect = True  # 원하는 요청 값 설정
        self.get_logger().info('서비스 보냄')
        response = self.client.call(req)
        self.get_logger().info(f'응답 받음: success = {response.success}')

def main(args=None):
    rclpy.init(args=args)
    node = StopClientNode() 
    # rclpy.spin(node)  # 🔁 계속 돌면서 응답 기다림
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
