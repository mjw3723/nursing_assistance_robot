#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator
from geometry_msgs.msg import Point
from rokey_interfaces.srv import Firstcmd, ObjectPosition, EndFlag
from irobot_create_msgs.msg import AudioNoteVector, AudioNote
import time

WAYPOINT_POSES = ([4.429, 0.885], TurtleBot4Directions.SOUTH)
STOPOVER_POSES = ([5.4, 0.0], TurtleBot4Directions.SOUTH)
MEDICINE_POSE = ([4.624, 0.13], 263.44)
PRODUCT_POSE = ([5.47, -0.04], 243.5)
INITIAL_POSE = ([5.85, 1.2], TurtleBot4Directions.NORTH)

# 오른쪽 선반 정중앙 좌표 x:4.5, y:-0.3
# 왼쪽 선반 정중앙 좌표 x:5.2, y:-0.29

class Robot4Navigation(Node):
    def __init__(self):
        super().__init__('robot4_navigation')

        self.get_logger().info("🚗 Navigation Node 시작!")

        self.navigator = TurtleBot4Navigator(namespace='/robot4')

        # Firstcmd 서비스 서버
        self.srv = self.create_service(Firstcmd, '/object_name', self.object_name_request)
        self.get_logger().info("✅ '/object_name' 서비스 서버 시작") 

        # robot4 랑데부 포인트 만남 후 서비스 서버 생성
        self.srv = self.create_service(EndFlag, '/robot4_meet', self.robot4_meet_callback)
        self.get_logger().info("✅ '/robot4_meet' 서비스 서버 시작")

        # ObjectPosition 클라이언트
        self.position_client = self.create_client(ObjectPosition, '/object_position')
        while not self.position_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('⏳ /object_position 서비스 대기 중...')

        # robot4 rendezvous 도착 여부 서비스 클라이언트 생성
        self.cli = self.create_client(EndFlag, '/robot4_rendezvous')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('⏳ /robot4_rendezvous 서비스 대기 중...')

        self.initial_pose_position = [5.5513458251953125, 1.052027702331543]
        self.initial_pose_direction = TurtleBot4Directions.NORTH

        self.robot1_x = 0.0
        self.robot1_y = 0.0
        self.rang_flage = False

        # 랑데부에서의 robot1 pose x,y 값 subscribe
        self.create_subscription(Point, '/robot4/robot1_pos', self.robot1_position_callback, 10)

        self.audio_pub = self.create_publisher(AudioNoteVector, '/robot4/cmd_audio', 10)
    
    '''robot4가 배달 물품 요청을 request 받으면 로봇이 선반 위치로 이동하는 함수'''
    def object_name_request(self, request, response):
        label = request.label.strip().lower()
        self.get_logger().info(f"request 수신 label: '{label}'")

        # 1. Nav2 활성화 대기
        self.navigator.waitUntilNav2Active()
        self.get_logger().info('Nav2 활성화 확인 완료')

        # 2. 초기 포즈 설정
        init_pose = self.navigator.getPoseStamped(
            self.initial_pose_position,
            self.initial_pose_direction
        )
        self.navigator.setInitialPose(init_pose)
        self.get_logger().info('초기 포즈 설정 완료')

        # # 3. 도킹 상태이면 언도킹
        if self.navigator.getDockedStatus():
            self.navigator.undock()
            self.get_logger().info('언도킹 완료')
        else:
            self.get_logger().info('이미 언도킹 상태')

        # 🎵 이동 전 멜로디 재생
        self.play_melody()

        if label == 'bandage' or label == 'cup':
            # 4. 하드코딩된 좌표로 이동
            goal_position, goal_direction = PRODUCT_POSE
            self.get_logger().info(f"목표 좌표로 이동 시작: {goal_position}, 방향={goal_direction}")
            goal_pose = self.navigator.getPoseStamped(goal_position, goal_direction)
            self.navigator.startToPose(goal_pose)
            self.get_logger().info("목표 좌표 이동 완료")
            time.sleep(5.0)

            # 5. 서비스 응답 먼저 전송
            response.success = True

            # 6. label 기반 후처리 요청 (비동기)
            self.call_object_position_service(label)

        elif label == 'tylenol' or label == 'codaewon':
            # 4. 하드코딩된 좌표로 이동
            goal_position, goal_direction = MEDICINE_POSE
            self.get_logger().info(f"목표 좌표로 이동 시작: {goal_position}, 방향={goal_direction}")
            goal_pose = self.navigator.getPoseStamped(goal_position, goal_direction)
            self.navigator.startToPose(goal_pose)
            self.get_logger().info("목표 좌표 이동 완료")
            time.sleep(5.0)

            # 5. 서비스 응답 먼저 전송
            response.success = True

            # 6. label 기반 후처리 요청 (비동기)
            self.call_object_position_service(label)

        else:
            response.success = False
            response.message = f"Unsupported label: {label}"
            self.get_logger().warn(response.message)

        return response


    '''vision node에 라벨을 request하고 물체 좌표를 response 받아 해당 좌표로 이동하는 함수'''
    def call_object_position_service(self, label: str):
        req = ObjectPosition.Request()
        req.label = label
        future = self.position_client.call_async(req)

        # 비동기로 처리
        def callback(fut):
            if fut.result() is not None:
                x = fut.result().x
                y = fut.result().y

                offset_y = y + 0.20
                self.get_logger().info(f"{label} 응답 좌표: x={x}, y={y}")
                # 1. 목표 좌표로 이동
                self.move_to_coordinates(x, offset_y)
            
            else:
                self.get_logger().error(f"{label} 서비스 호출 실패")

        future.add_done_callback(callback)

    # def move_rang(self):
    #     if self.robot1_x != 0.0 and self.robot1_y != 0.0:
    #         self.get_logger().info("3초 자는중")
    #         time.sleep(3.0)
    #         if self.rang_flage:
    #             self.get_logger().info("재귀함수 탈출중")
    #         else:
    #             self.rang_flage = True
    #             self.get_logger().info("🟢 robot1 좌표 수신 완료, 랑데부 포인트로 이동합니다.")
    #             self.move_to_waypoint(self.robot1_x, self.robot1_y)



    '''물체가 있는 좌표로 이동하는 함수'''
    def move_to_coordinates(self, x, y):
        # 목표 좌표로 이동하는 로직
        self.get_logger().info(f"목표 좌표로 이동: x={x}, y={y}")
        
        # 목표 포즈를 설정
        goal_position = [x, y]
        goal_direction = 250.0  # 기본적으로 북쪽으로 설정 (방향을 동적으로 설정하고 싶다면 추가 작업 필요)
        goal_pose = self.navigator.getPoseStamped(goal_position, goal_direction)
        
        # 로봇을 목표 좌표로 이동시킴
        self.navigator.startToPose(goal_pose)
        self.get_logger().info("목표 좌표로 이동 완료")
        time.sleep(7.0)

        self.get_logger().info(f"self.robot1_x = {self.robot1_x}, self.robot1_y = {self.robot1_y}")
        # 🟢 랑데부 위치 이동을 위한 타이머 시작
        # self.move_rang()
        if self.robot1_x != 0.0 and self.robot1_y != 0.0:
            self.get_logger().info(f"waypoint 함수 시작!!!")
            # time.sleep(1)
            # self.get_logger().info(f"대기중 ...")
            self.move_to_waypoint(self.robot1_x, self.robot1_y)
        


    '''랑데부 포인트로 이동하고 main controller에 도착 여부를 request 보내는 함수'''
    def move_to_waypoint(self, x, y):
        

        self.get_logger().info(f"🚗 robot1 위치로 이동 시작 → x={x}, y={y}")

        # 경유지로 이동
        goal_position, goal_direction = STOPOVER_POSES
        goal_pose = self.navigator.getPoseStamped(goal_position, goal_direction)

        # 로봇을 목표 좌표로 이동시킴
        self.navigator.startToPose(goal_pose)
        self.get_logger().info("경유지로 이동 완료")
        time.sleep(1.0)
        
        # robot1 위치로 이동
        goal_position = [x+0.35, y]
        goal_direction = 0.0  # 방향은 필요에 따라 조정
        goal_pose = self.navigator.getPoseStamped(goal_position, goal_direction)
        
        # 로봇을 목표 좌표로 이동시킴
        self.navigator.startToPose(goal_pose)
        self.get_logger().info(f"📍 robot1 위치로 이동 완료: {goal_position}")

        # robot4가 접선 장소에 도착했을 때 서비스 request 전송
        self.robot4_rendezvous_request(True)


    '''robot4 랑데부 포인트 도착 여부를 request 하는 함수'''
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


    '''robot4가 랑데부 포인트에서 물품을 전달 후 초기 위치로 이동하는 callback 함수'''
    def robot4_meet_callback(self, request, response):
        data = request.data
        self.get_logger().info(f"robot4 랑데부 포인트 도착 여부: '{data}'")

        if data:
            self.get_logger().info("[robot4] 물품을 전달하고 초기 위치로 이동합니다!")
            response.success = True

            goal_position, goal_direction = INITIAL_POSE
            initial_pose = self.navigator.getPoseStamped(goal_position, goal_direction)
            self.navigator.startToPose(initial_pose)
            self.navigator.goToPose
            time.sleep(2.0)
            self.navigator.dock()

            self.get_logger().info("[robot4] 초기 위치로 이동 완료!")

        else:
            self.get_logger().info("[robot4] 초기 위치 이동 실패")
            response.success = False

        return response

    '''robot1 랑데부 포인트에서의 위치를 수신 후 robot4가 이동하는 callback 함수'''
    def robot1_position_callback(self, msg: Point):
        self.robot1_x = msg.x
        self.robot1_y = msg.y
        self.get_logger().info(f"📡 robot1 위치 수신 → x: {self.robot1_x}, y: {self.robot1_y}")

        #self.move_rang()

    def play_melody(self):
        msg = AudioNoteVector()
        msg.append = False
        notes = [
            (392.0, 250), (329.63, 150), (392.0, 250), (329.63, 150),
            (440.0, 200), (392.0, 200), (349.0, 200), (329.63, 200),
            (349.0, 200), (392.0, 250), (329.63, 150), (349.0, 150),
            (392.0, 250), (329.63, 200), (392.0, 200), (329.63, 150),
            (349.0, 150), (329.63, 150), (293.66, 150), (246.94, 150), (261.63, 150)
        ]
        for freq, duration_ms in notes:
            note = AudioNote()
            note.frequency = int(freq)  # 🔴 float -> int로 변환
            note.max_runtime.sec = 0
            note.max_runtime.nanosec = duration_ms * 1_000_000
            msg.notes.append(note)

        self.audio_pub.publish(msg)
        self.get_logger().info("🎵 멜로디 재생 명령 전송 완료")


def main(args=None):
    rclpy.init(args=args)
    node = Robot4Navigation()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()