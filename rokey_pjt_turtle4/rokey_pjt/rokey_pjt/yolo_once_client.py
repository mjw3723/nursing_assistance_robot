# yolo_once_client.py

import rclpy
from rclpy.node import Node
from rokey_interfaces.srv import ObjectPosition

class ObjectPositionClient(Node):
    def __init__(self):
        super().__init__('object_position_client')
        self.client = self.create_client(ObjectPosition, 'object_position')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("서비스 서버 대기 중...")

    def send_request(self, label):
        request = ObjectPosition.Request()
        request.label = label

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()

        if response is not None:
            if response.x >= 0.0 or response.y >= 0.0:
                print(f"[응답 수신] 객체 위치: x = {response.x:.2f}, y = {response.y:.2f}")
            else:
                print(f"[실패] 객체 위치: x = {response.x:.2f}, y = {response.y:.2f}")
        else:
            print("❌ 서비스 응답 실패")

def main():
    rclpy.init()
    client = ObjectPositionClient()
    label = input("찾고 싶은 객체 라벨 입력: ")
    client.send_request(label)
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
