#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import threading
import time

from rokey_interfaces.srv import AssignPatient, NotifyArrival, GoToRoom

class FakePatrolNode(Node):
    def __init__(self,namespace=''):
        super().__init__('fake_patrol_node',namespace=namespace)

        # AssignPatient 서버 생성 (GUI가 클라이언트)
        self.assign_patient_server = self.create_service(AssignPatient, 'assign_patient', self.handle_assign_patient)

        # NotifyArrival 클라이언트 생성 (GUI가 서버)
        self.notify_arrival_client = self.create_client(NotifyArrival, 'notify_arrival')

        # GoToRoom 클라이언트 생성 (GUI가 서버)
        self.go_to_room_client = self.create_client(GoToRoom, 'go_to_room')

        # GUI가 서비스 서버일 경우, 테스트로 요청도 보내보기
        threading.Thread(target=self.simulate_notify_arrival, daemon=True).start()
        threading.Thread(target=self.simulate_go_to_room, daemon=True).start()

        self.get_logger().info('✅ FakePatrolNode: AssignPatient 서버 + notify/go_to_room 클라이언트 준비됨')

    def handle_assign_patient(self, request, response):
        self.get_logger().info(f'👨‍⚕️ AssignPatient 요청 받음: patient_id={request.patient_id}')
        response.success = True
        return response

    def simulate_notify_arrival(self):
        while not self.notify_arrival_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('❗ notify_arrival 서비스 기다리는 중...')
        
        req = NotifyArrival.Request()
        req.patient_id = 42

        future = self.notify_arrival_client.call_async(req)
        future.add_done_callback(self.notify_arrival_response_callback)

    def notify_arrival_response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'📨 NotifyArrival 응답: ack={response.ack}')
        except Exception as e:
            self.get_logger().error(f'❌ NotifyArrival 응답 실패: {e}')

    def simulate_go_to_room(self):
        while not self.go_to_room_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('❗ go_to_room 서비스 기다리는 중...')
        
        req = GoToRoom.Request()
        req.permission = True

        future = self.go_to_room_client.call_async(req)
        future.add_done_callback(self.go_to_room_response_callback)

    def go_to_room_response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'🚪 GoToRoom 응답: accepted={response.accepted}')
        except Exception as e:
            self.get_logger().error(f'❌ GoToRoom 응답 실패: {e}')

def main():
    rclpy.init()
    node = FakePatrolNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
