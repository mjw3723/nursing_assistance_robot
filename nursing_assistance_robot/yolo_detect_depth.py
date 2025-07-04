#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import time

class YoloSubscriber(Node):
    def __init__(self):
        super().__init__('yolo_subscriber')
        self.bridge = CvBridge()
        self.model = YOLO('/home/rokey/rokey_ws/src/rokey_pjt/rokey_pjt/best.pt', verbose=False)

        self.rgb_subscription = self.create_subscription(
            Image,
            '/robot1/oakd/rgb/preview/image_raw',
            self.listener_callback,
            10)

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )

        self.person_pub = self.create_publisher(Bool, '/person_detected', qos_profile)
        self.cleared_pub = self.create_publisher(Bool, '/person_cleared', qos_profile)
        self.cmd_vel_pub = self.create_publisher(Twist, '/robot1/cmd_vel', 10)

        self.person_published = False
        self.person_cleared_published = False
        self.no_wheelchair_frame_count = 0
        self.no_wheelchair_threshold = 5
        self.sleeping = False

    def listener_callback(self, msg):
        wheelchair_detected_now = False
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        results = self.model(cv_image, imgsz=320, conf=0.7)[0]
        annotated_frame = results.plot()

        if results.boxes is not None and results.boxes.xyxy is not None:
            for box, cls_id in zip(results.boxes.xyxy.cpu().numpy(), results.boxes.cls.cpu().numpy()):
                class_name = self.model.names[int(cls_id)]
                if class_name == 'wheelchair':
                    wheelchair_detected_now = True
                    self.get_logger().info("🦽 휠체어 감지됨!")

                    if not self.sleeping:
                        self.get_logger().info("⏸ 5초간 정지 중 (cmd_vel)...")
                        self.sleeping = True
                        self.person_published = True
                        self.person_cleared_published = False
                        self.publish_person_detect()
                        self.stop_robot()  # 0 속도 퍼블리시
                        time.sleep(5.0)
                        self.sleeping = False

        if not wheelchair_detected_now and self.person_published:
            self.no_wheelchair_frame_count += 1
            self.get_logger().info(f"🕵️ 휠체어 미감지 프레임 수: {self.no_wheelchair_frame_count}")
            if self.no_wheelchair_frame_count >= self.no_wheelchair_threshold:
                if not self.person_cleared_published:
                    self.publish_person_cleared()
                    self.no_wheelchair_frame_count = 0
        else:
            self.no_wheelchair_frame_count = 0

        cv2.imshow("YOLOv8 Detection", annotated_frame)
        cv2.waitKey(1)

    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info("🛑 로봇 정지 cmd_vel 발행")

    def publish_person_detect(self):
        msg = Bool()
        msg.data = True
        self.person_pub.publish(msg)
        self.get_logger().info('✅ /person_detected → True 발행')

    def publish_person_cleared(self):
        msg = Bool()
        msg.data = True
        self.cleared_pub.publish(msg)
        self.get_logger().info('⚠️ /person_cleared → True 발행')
        self.person_published = False
        self.person_cleared_published = True


def main(args=None):
    rclpy.init(args=args)
    node = YoloSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("❌ 종료됨 (Ctrl+C)")
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
