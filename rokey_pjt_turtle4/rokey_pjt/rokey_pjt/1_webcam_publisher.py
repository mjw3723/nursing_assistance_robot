# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
# import cv2
# import time

# class WebcamPublisher(Node):
#     def __init__(self):
#         super().__init__('webcam_raw_publisher')

#         self.publisher = self.create_publisher(Image, '/robot4/oakd/rgb/image_raw', 1)
#         self.bridge = CvBridge()

#         self.cap = cv2.VideoCapture(0)
#         if not self.cap.isOpened():
#             self.get_logger().error("❌ 웹캠 열기 실패")
#             exit(1)

#         self.get_logger().info("✅ 웹캠 열기 성공, 퍼블리시 시작")

#     def publish_frame(self):
#         ret, frame = self.cap.read()
#         if not ret:
#             self.get_logger().warn("⚠️ 프레임 캡처 실패")
#             return

#         # OpenCV BGR 이미지를 ROS Image 메시지로 변환
#         img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
#         img_msg.header.stamp = self.get_clock().now().to_msg()
#         img_msg.header.frame_id = "webcam_frame"

#         self.publisher.publish(img_msg)
#         self.get_logger().info("📤 이미지 퍼블리시", throttle_duration_sec=1.0)

#     def destroy_node(self):
#         self.cap.release()
#         super().destroy_node()

# def main():
#     rclpy.init()
#     node = WebcamPublisher()

#     try:
#         while rclpy.ok():
#             rclpy.spin_once(node, timeout_sec=0.01)
#             node.publish_frame()
#             time.sleep(0.033)  # 30fps 기준
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()


import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import time

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')
        self.publisher = self.create_publisher(Int32, 'simple_topic', 10)

    def publish_data(self):
        msg = Int32()
        msg.data = 1
        self.publisher.publish(msg)
        self.get_logger().info('Publishing: 1')

def main():
    rclpy.init()
    node = SimplePublisher()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            node.publish_data()
            time.sleep(0.1)  # 10Hz 주기
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
