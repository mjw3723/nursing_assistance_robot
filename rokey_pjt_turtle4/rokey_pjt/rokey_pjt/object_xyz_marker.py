import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge
import tf2_ros
import tf2_geometry_msgs
import numpy as np
import cv2
import threading
import time
import os
import shutil
import sys
import csv
from ultralytics import YOLO
from pathlib import Path
import torch
import argparse
import math


# robot namespace 지정
ROBOT_NAMESPACE = 'robot4'

# topic 이름 지정
RGB_TOPIC = f'/{ROBOT_NAMESPACE}/oakd/rgb/image_raw'
RGB_TOPIC_COMPRESSED = f'/{ROBOT_NAMESPACE}/oakd/rgb/image_raw/compressed'
DEPTH_TOPIC = f'/{ROBOT_NAMESPACE}/oakd/stereo/image_raw'
CAMERA_INFO_TOPIC = f'/{ROBOT_NAMESPACE}/oakd/stereo/camera_info'
MARKER_TOPIC = f'/{ROBOT_NAMESPACE}/detected_objects_marker'

# YOLO confidence, model path
CONFIDENCE_THRESHOLD = 0.7
YOLO_PATH = '/home/hongha/YOLO Dataset_turtlebot4/runs/detect/yolov8s/weights/best.pt'

# raw, compressed 모드 설정
# SUB_IMAGE_TYPE = 'compressed'
SUB_IMAGE_TYPE = 'raw'

# 인식된 객체의 시각화 색상
color_dict = {
    "bandage": (0, 255, 255),      # cyan (BGR)
    "codaewon": (0, 255, 0),       # lime/yellow-green (BGR)
    "cup": (0, 0, 255),            # red (BGR)
    "tylenol": (255, 0, 255)       # purple (BGR)
}


class YoloDepthToMap(Node):
    def __init__(self, model, output_dir):
        super().__init__('yolo_depth_to_map')
        self.get_logger().info("YoloDepthToMap node is starting...")

        self.model = model
        self.output_dir = output_dir
        self.bridge = CvBridge()
        self.classNames = getattr(self.model, 'names', [])

        self.K = None
        self.latest_rgb = self.latest_depth = self.latest_rgb_msg = self.processed_frame = None
        self.lock = threading.Lock()
        self.should_shutdown = False

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.marker_pub = self.create_publisher(Marker, MARKER_TOPIC, 10)
        self.marker_id = 0

        self.csv_output, self.confidences = [], []
        self.max_object_count = 0
        
        # self.create_subscription(Image, RGB_TOPIC, self.rgb_callback, 1)
        self.create_subscription(Image, DEPTH_TOPIC, self.depth_callback, 1)
        self.create_subscription(CameraInfo, CAMERA_INFO_TOPIC, self.camera_info_callback, 1)

        if SUB_IMAGE_TYPE == 'compressed':
            self.create_subscription(CompressedImage, RGB_TOPIC_COMPRESSED, self.rgb_callback, 1)
            self.get_logger().info(f"Subscribing to topics:\n  RGB: {RGB_TOPIC_COMPRESSED}\n  Depth: {DEPTH_TOPIC}\n  CameraInfo: {CAMERA_INFO_TOPIC}\n  MarkerPub: {MARKER_TOPIC}")
        elif SUB_IMAGE_TYPE == 'raw':
            self.create_subscription(Image, RGB_TOPIC, self.rgb_callback, 1)
            self.get_logger().info(f"Subscribing to topics:\n  RGB: {RGB_TOPIC}\n  Depth: {DEPTH_TOPIC}\n  CameraInfo: {CAMERA_INFO_TOPIC}\n  MarkerPub: {MARKER_TOPIC}")


    def camera_info_callback(self, msg):
        if self.K is None:
            self.K = np.array(msg.k).reshape(3, 3)
            self.get_logger().info(f"CameraInfo: fx={self.K[0,0]:.2f}, fy={self.K[1,1]:.2f}, cx={self.K[0,2]:.2f}, cy={self.K[1,2]:.2f}")

    def rgb_callback(self, msg):
        try:
            # img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            # 이미지 메시지 -> OpenCV 이미지 변환
            if SUB_IMAGE_TYPE == 'compressed':
                np_arr = np.frombuffer(msg.data, np.uint8)
                img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            elif SUB_IMAGE_TYPE == 'raw':
                img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            with self.lock:
                self.latest_rgb, self.latest_rgb_msg = img, msg
        except Exception as e:
            self.get_logger().error(f"RGB conversion error: {e}")

    def depth_callback(self, msg):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
            with self.lock:
                self.latest_depth = depth
        except Exception as e:
            self.get_logger().error(f"Depth conversion error: {e}")

    def transform_to_map(self, pt_camera: PointStamped, class_name: str):
        try:
            pt_map = self.tf_buffer.transform(pt_camera, 'map', timeout=rclpy.duration.Duration(seconds=0.5))
            x, y, z = pt_map.point.x, pt_map.point.y, pt_map.point.z
            self.get_logger().info(f"[TF] {class_name} → map: (x={x:.2f}, y={y:.2f}, z={z:.2f})")
            return x, y, z
        except Exception as e:
            self.get_logger().warn(f"[TF] class={class_name} 변환 실패: {e}")
            return float('nan'), float('nan'), float('nan')

    def publish_marker(self, x, y, z, label):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns, marker.id = 'detected_objects', self.marker_id
        self.marker_id += 1
        marker.type, marker.action = Marker.SPHERE, Marker.ADD
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = x, y, z
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.2
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 1.0, 0.0, 1.0
        marker.lifetime.sec = 5
        self.marker_pub.publish(marker)

    def inference_loop(self):
        self.get_logger().info("Inference loop started. Waiting for images...")
        while rclpy.ok() and not self.should_shutdown:
            with self.lock:
                rgb, depth, K, rgb_msg = self.latest_rgb, self.latest_depth, self.K, self.latest_rgb_msg

            if any(v is None for v in (rgb, depth, K, rgb_msg)):
                time.sleep(0.005)
                continue

            frame = rgb.copy()
            results = self.model(frame, verbose=False)
            object_count = 0

            for result in results:
                if result.boxes is None:
                    continue
                for box in result.boxes:
                    u, v = map(int, box.xywh[0][:2].cpu().numpy())
                    if not (0 <= v < depth.shape[0] and 0 <= u < depth.shape[1]):
                        continue
                    
                    conf = math.ceil(box.conf[0] * 100) / 100
                    if conf < CONFIDENCE_THRESHOLD:
                        continue  # 신뢰도 낮으면 건너뜀

                    z = float(depth[v, u]) / 1000.0
                    fx, fy = K[0, 0], K[1, 1]
                    cx, cy = K[0, 2], K[1, 2]
                    x = (u - cx) * z / fx
                    y = (v - cy) * z / fy

                    x1, y1, x2, y2 = map(int, box.xyxy[0].cpu().numpy())
                    conf = float(box.conf[0])
                    cls = int(box.cls[0])
                    label = self.classNames[cls] if cls < len(self.classNames) else f'class_{cls}'

                    pt_camera = PointStamped()
                    pt_camera.header.frame_id = rgb_msg.header.frame_id
                    pt_camera.header.stamp = rclpy.time.Time().to_msg()
                    pt_camera.point.x, pt_camera.point.y, pt_camera.point.z = x, y, z

                    map_x, map_y, map_z = self.transform_to_map(pt_camera, label)
                    if not np.isnan(map_x):
                        self.publish_marker(map_x, map_y, map_z, label)

                    cv2.rectangle(frame, (x1, y1), (x2, y2), color_dict[label], 2)
                    cv2.circle(frame, (u, v), 4, color_dict[label], -1)
                    # cv2.putText(frame, f"{label} {conf:.2f} {z:.2f}m", (x1, y1-5),
                    #             cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_dict[label], 2)
                    cv2.putText(frame, f"{label} {conf:.2f}", (x1, y1 - 35),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_dict[label], 2)
                    cv2.putText(frame, f"{z:.2f}m", (x1, y1 - 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_dict[label], 2)
                    cv2.putText(frame, f"({map_x:.2f}, {map_y:.2f}, {map_z:.2f})", (x1, y1 -5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_dict[label], 2)

                    self.csv_output.append([label, conf, u, v, x, y, z, map_x, map_y, map_z])
                    self.confidences.append(conf)
                    object_count += 1

            self.max_object_count = max(self.max_object_count, object_count)
            with self.lock:
                self.processed_frame = frame
            time.sleep(0.005)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--model', type=str, required=False, default=YOLO_PATH, help='Path to model file (.pt, .onnx, .engine)')
    args, _ = parser.parse_known_args()

    model_path = args.model

    if not os.path.exists(model_path):
        print(f"Model file not found: {model_path}")
        sys.exit(1)

    suffix = Path(model_path).suffix.lower()
    model = YOLO(model_path, task='detect') if suffix in ['.onnx', '.engine'] else YOLO(model_path)
    model.to('cuda' if torch.cuda.is_available() else 'cpu')
    print("Using GPU for inference." if torch.cuda.is_available() else "Using CPU.")

    output_dir = './output'
    if os.path.exists(output_dir):
        shutil.rmtree(output_dir)
    os.mkdir(output_dir)

    rclpy.init()
    node = YoloDepthToMap(model, output_dir)

    try:
        threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
        threading.Thread(target=node.inference_loop, daemon=True).start()

        while rclpy.ok() and not node.should_shutdown:
            with node.lock:
                frame = node.processed_frame.copy() if node.processed_frame is not None else None
            if frame is not None:
                cv2.imshow("YOLO + Depth + Map", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                node.get_logger().info("Shutdown requested by user.")
                node.should_shutdown = True
                break
            time.sleep(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()
        print("Shutdown complete.")
        sys.exit(0)

if __name__ == '__main__':
    main()