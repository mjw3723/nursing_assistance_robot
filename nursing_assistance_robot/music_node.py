#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import threading
from irobot_create_msgs.msg import AudioNoteVector, AudioNote
from builtin_interfaces.msg import Duration

class AudioTestNode(Node):
    def __init__(self):
        super().__init__('audio_test_node')
        self.robot1_audio_publisher = self.create_publisher(AudioNoteVector, '/robot1/cmd_audio', 10)
        self.robot4_audio_publisher = self.create_publisher(AudioNoteVector, '/robot4/cmd_audio', 10)
        # 입력 대기 스레드 시작
        threading.Thread(target=self.input_thread, daemon=True).start()

    def input_thread(self):
        while rclpy.ok():
            try:
                self.get_logger().info('노래 번호 입력 (1~6), 종료는 q:')
                user_input = input()
                if user_input.lower() == 'q':
                    self.get_logger().info('노드 종료 요청 받음. 종료합니다.')
                    rclpy.shutdown()
                    break
                state = int(user_input)
                if state in range(1, 15):
                    self.get_logger().info(f'노래 상태 {state} 재생 시작!')
                    self.start_audio(state)
                else:
                    self.get_logger().warn('1~6 사이 숫자만 입력하세요!')
            except Exception as e:
                self.get_logger().error(f'입력 처리 중 오류: {e}')

    def start_audio(self, state):
        if state == 1:
            notes = [
                (659, 0.125), (659, 0.125), (0, 0.125), (659, 0.125), (0, 0.125),
                (523, 0.125), (659, 0.125), (0, 0.125), (784, 0.125), (0, 0.375),
                (392, 0.125), (0, 0.375),
                (523, 0.125), (0, 0.25), (392, 0.125), (0, 0.25),
                (330, 0.125), (0, 0.125), (440, 0.125), (0, 0.125),
                (494, 0.125), (0, 0.125), (466, 0.125), (0, 0.125),
                (440, 0.125), (0, 0.125), (392, 0.125), (659, 0.125),
                (784, 0.125), (880, 0.25), (698, 0.125), (784, 0.125),
                (659, 0.125), (523, 0.125), (587, 0.125), (494, 0.125),
            ]
        elif state == 2:
            notes = [
                (440, 0.2), (494, 0.2), (523, 0.2), (587, 0.2),
                (659, 0.2), (698, 0.2), (784, 0.3), (880, 0.3), (784, 0.2),
                (698, 0.2), (659, 0.2), (587, 0.2), (523, 0.2), (494, 0.2), (440, 0.3),
                (0, 0.2),
                (440, 0.2), (494, 0.2), (523, 0.2), (587, 0.2),
                (659, 0.2), (698, 0.2), (784, 0.3), (880, 0.3), (784, 0.2),
                (698, 0.2), (659, 0.2), (587, 0.2), (523, 0.2), (494, 0.2), (440, 0.3),
            ]
        elif state == 3:
            notes = [
                (659, 0.2), (622, 0.2), (659, 0.2), (622, 0.2), (659, 0.2), (494, 0.2), (587, 0.2), (523, 0.2),
                (440, 0.4), (0, 0.2), (262, 0.2), (330, 0.2), (440, 0.2), (494, 0.2),
                (330, 0.2), (415, 0.2), (494, 0.2), (523, 0.2), (330, 0.2),
                (659, 0.2), (622, 0.2), (659, 0.2), (622, 0.2), (659, 0.2), (494, 0.2), (587, 0.2), (523, 0.2)
            ]
        elif state == 4:
            notes = [
                (440, 0.3), (392, 0.3), (440, 0.3), (392, 0.3), (440, 0.3), (349, 0.3), (392, 0.3), (440, 0.3),
                (392, 0.3), (440, 0.3), (392, 0.3), (440, 0.3), (349, 0.3), (392, 0.3), (440, 0.3),
                (440, 0.3), (440, 0.3), (440, 0.3), (392, 0.3), (349, 0.3), (440, 0.3),
                (392, 0.3), (440, 0.3), (392, 0.3), (349, 0.3), (392, 0.6),
                (392, 0.3), (392, 0.3), (392, 0.3), (440, 0.3), (392, 0.3), (349, 0.3),
                (392, 0.3), (440, 0.3), (392, 0.3), (440, 0.3), (392, 0.3),
                (440, 0.3), (494, 0.3), (523, 0.3), (494, 0.3), (440, 0.3),
                (392, 0.3), (440, 0.6), (0, 0.2),
            ]
        elif state == 5:
            notes = [
                (659, 0.2), (784, 0.2), (880, 0.2), (988, 0.2), (880, 0.2), (988, 0.2), (880, 0.2),
                (784, 0.2), (659, 0.2), (784, 0.2), (880, 0.2), (784, 0.2), (659, 0.2),
                (523, 0.2), (659, 0.2), (784, 0.2), (659, 0.2), (523, 0.2),
                (440, 0.3), (440, 0.2), (440, 0.2), (523, 0.2), (659, 0.2),
                (784, 0.3), (880, 0.2), (988, 0.2), (880, 0.2),
                (784, 0.2), (659, 0.2), (784, 0.2), (880, 0.2), (784, 0.2), (659, 0.2),
                (523, 0.2), (659, 0.2), (784, 0.2), (659, 0.2), (523, 0.3),
            ]
        elif state == 6:
            notes = [
                (440, 0.3), (494, 0.3), (523, 0.3), (587, 0.3),  # A B C D
                (659, 0.6), (0, 0.1),                             # E (쉼)
                (659, 0.3), (587, 0.3), (523, 0.3), (494, 0.3),  # E D C B
                (440, 0.6), (0, 0.2),                             # A (쉼)
                (523, 0.3), (587, 0.3), (659, 0.3), (698, 0.3),  # C D E F
                (784, 0.6), (0, 0.1),                             # G (쉼)
                (784, 0.3), (698, 0.3), (659, 0.3), (587, 0.3),  # G F E D
                (523, 0.6), (0, 0.2),                             # C (쉼)
            ]
        elif state == 7:
            notes = [
                (392, 0.5), (440, 0.5), (523, 0.75), (587, 0.75),
                (659, 1.0), (587, 0.5), (523, 0.5), (440, 1.0),
                (392, 0.5), (392, 0.5), (440, 0.75), (523, 0.75),
                (587, 1.0), (523, 0.5), (440, 0.5), (392, 1.5),
                (349, 0.5), (392, 0.5), (440, 1.0), (392, 0.5),
                (349, 0.5), (330, 1.5), (0, 0.5)
            ]
        audio_vector_msg = AudioNoteVector()
        audio_vector_msg.append = False
        for freq, sec in notes:
            note = AudioNote()
            note.frequency = int(freq)
            note.max_runtime = Duration(sec=0, nanosec=int(sec * 1e9))
            audio_vector_msg.notes.append(note)
        self.robot1_audio_publisher.publish(audio_vector_msg)
        self.robot4_audio_publisher.publish(audio_vector_msg)
        rclpy.spin_once(self, timeout_sec=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = AudioTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
