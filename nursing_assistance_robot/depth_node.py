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
DEPTH_TOPIC = '/robot1/oakd/stereo/image_raw'  # Depth 이미지 토픽
MAX_DEPTH_METERS = 5.0                 # 시각화 시 최대 깊이 값 (m)
NORMALIZE_DEPTH_RANGE = 3.0     
class DepthSubscriber(Node):
    def __init__(self):
        super().__init__('depth_subscriber')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            DEPTH_TOPIC,
            self.depth_callback,
            1)
        self.point_subscription = self.create_subscription(
            Point,
            '/depth_point',
            self.point_callback,
            1
        )
        self.distance_publisher = self.create_publisher(
            Float64,
            '/distance',
            1
        )
        self.cx = None
        self.cy = None

    def depth_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            if self.cx is not None and self.cy is not None: 
                self.cx = int(self.cx)
                self.cy = int(self.cy)
                distance_mm = self.depth_image[self.cy, self.cx]
                distance_m = distance_mm / 1000.0
                msg = Float64()
                msg.data = float(distance_m)
                self.distance_publisher.publish(msg)
        except AttributeError:
            self.get_logger().warn('depth image None')

    def point_callback(self,msg:Point):
        self.cx = msg.x
        self.cy = msg.y
        self.get_logger().info('Point Callback !!')
        
def main(args=None):
    rclpy.init(args=args)
    node = DepthSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('종료합니다.')
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
