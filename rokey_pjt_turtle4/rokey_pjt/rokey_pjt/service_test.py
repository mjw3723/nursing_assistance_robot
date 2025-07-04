import rclpy
from rclpy.node import Node
from rokey_interfaces.srv import ObjectPosition  # :흰색_확인_표시: 커스텀 서비스 타입 사용

class ObjectPositionServer(Node):
    def __init__(self):
        super().__init__('object_position_server')
        # :흰색_확인_표시: 커스텀 타입 기반 서비스 등록
        self.srv = self.create_service(
            ObjectPosition,
            'object_position_receiver',
            self.callback
        )
        self.get_logger().info("ObjectPosition 커스텀 서비스 서버 시작됨.")

    def callback(self, request, response):
        label = request.label
        x = request.x
        y = request.y
        self.get_logger().info(f"[Service] 수신: label='{label}', x={x:.3f}, y={y:.3f}")
        response.success = True
        response.message = f"수신 완료: {label} at ({x:.3f}, {y:.3f})"
        return response
    
def main():
    rclpy.init()
    node = ObjectPositionServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()