# stop_server.py
import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import Stop

class StopServerNode(Node):
    def __init__(self):
        super().__init__('stop_server')
        self.srv = self.create_service(Stop, 'stop', self.callback_stop_service)
        self.get_logger().info('🟢 Stop 서비스 서버 시작됨!')

    def callback_stop_service(self, request, response):
        self.get_logger().info(f'요청 받음: stop = {request.person_detect}')
        
        if request.person_detect:
            self.get_logger().info('>> 로봇 정지 명령 수신됨.')
            response.success = True
        else:
            self.get_logger().info('>> 정지 명령 아님.')
            response.success = False
        
        return response

def main(args=None):
    rclpy.init(args=args)
    node = StopServerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
