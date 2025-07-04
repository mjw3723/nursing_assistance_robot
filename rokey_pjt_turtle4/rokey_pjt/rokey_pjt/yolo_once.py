# object_position_server.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from rokey_interfaces.srv import ObjectPosition
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
import tf2_ros
import tf2_geometry_msgs
from ultralytics import YOLO
import numpy as np
import cv2
import torch

ROBOT_NAMESPACE = 'robot4'
RGB_TOPIC = f'/{ROBOT_NAMESPACE}/oakd/rgb/image_raw'
DEPTH_TOPIC = f'/{ROBOT_NAMESPACE}/oakd/stereo/image_raw'
CAMERA_INFO_TOPIC = f'/{ROBOT_NAMESPACE}/oakd/stereo/camera_info'

CONFIDENCE_THRESHOLD = 0.7
YOLO_PATH = '/home/hongha/rokey_ws/src/rokey_pjt/yolo_weights/best.pt'


class ObjectDetectionServer(Node):
    def __init__(self):
        super().__init__('object_position_server')
        self.bridge = CvBridge()
        self.model = YOLO(YOLO_PATH)
        self.model.to('cuda' if torch.cuda.is_available() else 'cpu')
        self.classNames = getattr(self.model, 'names', [])

        self.latest_rgb = None
        self.latest_depth = None
        self.K = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(Image, RGB_TOPIC, self.rgb_callback, 10)
        self.create_subscription(Image, DEPTH_TOPIC, self.depth_callback, 10)
        self.create_subscription(CameraInfo, CAMERA_INFO_TOPIC, self.camera_info_callback, 10)

        self.srv = self.create_service(ObjectPosition, 'object_position_receiver', self.handle_object_request)
        self.get_logger().info("✅ ObjectPosition 서비스 서버 시작됨.")

    def rgb_callback(self, msg):
        try:
            self.latest_rgb = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.rgb_msg = msg
        except Exception as e:
            self.get_logger().error(f"RGB 변환 오류: {e}")

    def depth_callback(self, msg):
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
        except Exception as e:
            self.get_logger().error(f"Depth 변환 오류: {e}")

    def camera_info_callback(self, msg):
        if self.K is None:
            self.K = np.array(msg.k).reshape(3, 3)
            self.get_logger().info("CameraInfo 수신 완료")

    def handle_object_request(self, request, response):
        label_req = request.label.lower().strip()
        self.get_logger().info(f"[서비스 요청 수신] label='{label_req}'")

        if self.latest_rgb is None or self.latest_depth is None or self.K is None:
            self.get_logger().warn("이미지 또는 카메라 정보가 아직 수신되지 않았습니다.")
            response.x = -1.0
            response.y = -1.0
            return response

        frame = self.latest_rgb.copy()
        results = self.model(frame, verbose=False)

        for result in results:
            for box in result.boxes:
                cls = int(box.cls[0])
                label = self.classNames[cls].lower()

                if label != label_req:
                    continue

                u, v = map(int, box.xywh[0][:2].cpu().numpy())
                if not (0 <= v < self.latest_depth.shape[0] and 0 <= u < self.latest_depth.shape[1]):
                    continue

                conf = float(box.conf[0])
                if conf < CONFIDENCE_THRESHOLD:
                    continue

                z = float(self.latest_depth[v, u]) / 1000.0
                fx, fy = self.K[0, 0], self.K[1, 1]
                cx, cy = self.K[0, 2], self.K[1, 2]
                x = (u - cx) * z / fx
                y = (v - cy) * z / fy

                pt_camera = PointStamped()
                pt_camera.header.frame_id = self.rgb_msg.header.frame_id
                pt_camera.header.stamp = rclpy.time.Time(seconds=0).to_msg()  # 가장 최신 TF 사용

                pt_camera.point.x, pt_camera.point.y, pt_camera.point.z = x, y, z

                try:
                    pt_map = self.tf_buffer.transform(pt_camera, 'map', timeout=rclpy.duration.Duration(seconds=0.5))
                    response.x = pt_map.point.x
                    response.y = pt_map.point.y
                    self.get_logger().info(f"[응답] {label} 위치: ({response.x:.2f}, {response.y:.2f})")
                    return response
                except Exception as e:
                    self.get_logger().warn(f"TF 변환 실패: {e}")

        response.x = -1.0
        response.y = -1.0
        self.get_logger().warn(f"'{label_req}' 객체를 찾을 수 없습니다.")
        return response


def main():
    rclpy.init()
    node = ObjectDetectionServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
