import rclpy
from rclpy.node import Node
from rokey_interfaces.srv import EndFlag

class RendezvousClient(Node):
    def __init__(self):
        super().__init__('rendezvous_client')
        self.cli = self.create_client(EndFlag, '/robot4_rendezvous')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('⏳ /robot4_rendezvous 서비스 대기 중...')

    def robot4_rendezvous_request(self, arrived_flag: bool):
        request = EndFlag.Request()
        request.data = arrived_flag

        self.get_logger().info(f"📤 '도착 여부: {arrived_flag}' 요청 전송 중...")

        future = self.cli.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            if future.result().success:
                self.get_logger().info("✅ 도착 성공")
            else:
                self.get_logger().info("❌ 도착 실패")
        else:
            self.get_logger().error("서비스 응답이 없습니다.")

def main():
    rclpy.init()
    client = RendezvousClient()

    user_input = input("🤖 robot4가 랑데부 지점에 도착했습니까? (y/n): ").strip().lower()
    flag = user_input == 'y'

    client.robot4_rendezvous_request(flag)

    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
