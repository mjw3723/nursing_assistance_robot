import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import time
import tf2_ros
import tf2_geometry_msgs 
from visualization_msgs.msg import Marker
from std_msgs.msg import Bool 
from std_msgs.msg import Float64
from geometry_msgs.msg import Point
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
# import logging
# logging.getLogger("ultralytics").setLevel(logging.ERROR)
DEPTH_TOPIC = '/robot1/oakd/stereo/image_raw'  # Depth 이미지 토픽
MAX_DEPTH_METERS = 5.0                 # 시각화 시 최대 깊이 값 (m)
NORMALIZE_DEPTH_RANGE = 3.0     
class YoloSubscriber(Node):
    def __init__(self):
        super().__init__('yolo_subscriber')
        self.bridge = CvBridge()
        self.model = YOLO('/home/moon/turtlebot4_ws/src/yolov8_ros/yolov8_ros/best.pt',verbose=False)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )
        self.rgb_subscription = self.create_subscription(
            Image,
            '/robot1/oakd/rgb/preview/image_raw',
            self.listener_callback,
            1)
        self.distance_subscription = self.create_subscription(
            Float64,
            '/distance',
            self.distance_callback,
            1
        )
        self.point_publisher = self.create_publisher(
            Point,
            '/depth_point',
            1
        )
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.marker_pub = self.create_publisher(Marker, '/robot1/yolo_marker', 10)
        self.marker_id = 0
        self.person_published = False
        self.person_cleared_published = False
        self.person_pub = self.create_publisher(Bool, '/person_detected', qos_profile)
        self.cleared_pub = self.create_publisher(Bool, '/person_cleared', qos_profile)
        self.no_person_frame_count = 0
        self.no_person_frame_threshold = 5 
        self.distance_m = None

    def listener_callback(self, msg):
        person_detected_now = False
        # ROS 이미지 → OpenCV 이미지
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # YOLO 추론
        results = self.model(cv_image, imgsz=320, conf=0.7)[0]
        annotated_frame = results.plot()  # 결과 시각화
        if results.boxes is not None and results.boxes.xyxy is not None:
            for box, cls_id in zip(results.boxes.xyxy.cpu().numpy(), results.boxes.cls.cpu().numpy()):
                class_name = self.model.names[int(cls_id)]
                if class_name != 'wheelchair' and class_name != 'patient':
                    continue
                self.get_logger().info(f"class Name = ===== {class_name}")
                x1, y1, x2, y2 = box[:4]
                cx = int((x1 + x2) / 2)
                cy = int((y1 + y2) / 2)
                self.publish_point(cx,cy)
                if self.distance_m is not None:
                    if self.distance_m <= 5.0:
                        person_detected_now = True
                        if not self.person_published:
                            self.publish_person_detect()
                    else:
                        person_detected_now = False
        if person_detected_now:
            self.no_person_frame_count = 0  # 감지됐으면 초기화
        else:
            if self.person_published:  # 사람이 이전에 감지된 상태일 때만 체크
                self.no_person_frame_count += 1
                if self.no_person_frame_count >= self.no_person_frame_threshold:
                    if not self.person_cleared_published:
                        self.publish_person_cleared()
                        self.person_cleared_published = True
                    self.no_person_frame_count = 0  # 초기화
        cv2.imshow("YOLOv8 Detection", annotated_frame)
        cv2.waitKey(1)

    def distance_callback(self,msg:Float64):
        self.distance_m = msg.data

    def publish_person_detect(self):
        msg = Bool()
        msg.data = True
        self.person_pub.publish(msg)
        self.get_logger().info('✅ /person_detected → True 발행')
        self.person_published = True
        self.person_cleared_published = False

    def publish_person_cleared(self):
        msg = Bool()
        msg.data = True
        self.cleared_pub.publish(msg)
        self.get_logger().info('⚠️ /person_cleared → True 발행')

    def publish_marker(self,point_map:PointStamped):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'yolo'
        marker.id = self.marker_id
        self.marker_id += 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = point_map.point.x
        marker.pose.position.y = point_map.point.y
        marker.pose.position.z = 0.1
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.lifetime.sec = 2  # 2초간 유지
        self.marker_pub.publish(marker)

    def publish_point(self,cx:int,cy:int):
        point_msg = Point()
        point_msg.x = float(cx)
        point_msg.y = float(cy)
        point_msg.z = 0.0
        self.point_publisher.publish(point_msg)

    def point_transform(self,distance:float):
        point_base = PointStamped()
        point_base.header.stamp = rclpy.time.Time().to_msg()
        point_base.header.frame_id = 'base_link'
        point_base.point.x = distance
        point_base.point.y = 0.0
        point_base.point.z = 0.0
        if self.tf_buffer.can_transform('map', 'base_link', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0)):
            point_map = self.tf_buffer.transform(
                point_base,
                'map',
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
            self.get_logger().info(f"[Map] ({point_map.point.x:.2f}, {point_map.point.y:.2f}, {point_map.point.z:.2f})")
            return point_base
        else:
            self.get_logger().warn('Transform from base_link to map is not available yet.')
            return None
    
        
def main(args=None):
    rclpy.init(args=args)
    node = YoloSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('종료합니다.')
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
