import rclpy
from rclpy.node import Node
from rokey_interfaces.srv import Firstcmd

class ObjectNameService(Node):
    def __init__(self):
        super().__init__('object_name_service')
        self.srv = self.create_service(Firstcmd, '/object_name', self.object_name_callback)
        self.get_logger().info("✅ '/object_name' 서비스 서버 시작")

    def object_name_callback(self, request, response):
        label = request.label
        self.get_logger().info(f"요청 받은 라벨: '{label}'")

        if label.lower() in ['bandage', 'codaewon', 'cup', 'tylenol']:
            self.get_logger().info("제조실에 있는 객체입니다.")
            response.success = True
        else:
            self.get_logger().info("제조실에 없는 객체입니다.")
            response.success = False
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ObjectNameService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
