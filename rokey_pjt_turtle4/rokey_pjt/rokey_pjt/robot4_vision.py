# robot4_vision.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from rokey_interfaces.srv import ObjectPosition
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

# raw, compressed 모드 설정
SUB_IMAGE_TYPE = 'compressed'
# SUB_IMAGE_TYPE = 'raw'

# YOLO confidence, model path
CONFIDENCE_THRESHOLD = 0.8
YOLO_PATH = '/home/hongha/YOLO Dataset_turtlebot4/runs/detect/yolov8s/weights/best.pt'

# 인식된 객체의 시각화 색상
color_dict = {
    "bandage": (0, 255, 255),   # cyan (BGR)
    "codaewon": (0, 255, 0),    # lime/yellow-green (BGR)
    "cup": (0, 0, 255),         # red (BGR)
    "tylenol": (255, 0, 255)    # purple (BGR)
}


'''객체를 탐지하고 map 좌표를 추정한 후, navigation node에 서비스 response를 반환하는 클래스'''
class Robot4Vision(Node):
    def __init__(self):
        super().__init__('robot4_vision')

        self.get_logger().info("📷 Vision Node 시작!")

        # YOLO 객체 탐지 모델 초기화
        self.model = YOLO(YOLO_PATH)
        self.model.to('cuda' if torch.cuda.is_available() else 'cpu')
        self.get_logger().info("Using GPU for inference." if torch.cuda.is_available() else "Using CPU.")
        self.classNames = getattr(self.model, 'names', [])

        # CvBridge 설정
        self.bridge = CvBridge()

        # 내부 상태 변수 정의
        self.K = None
        self.latest_rgb = self.latest_depth = self.latest_rgb_msg = self.processed_frame = None
        self.lock = threading.Lock()
        self.should_shutdown = False

        # TF2 초기화
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 마커 publisher 설정
        self.marker_pub = self.create_publisher(Marker, MARKER_TOPIC, 10)
        self.marker_id = 0

        self.get_logger().info("📷 카메라 대기중 ...")

        # 센서 토픽 subscription 설정
        self.create_subscription(Image, DEPTH_TOPIC, self.depth_callback, 1)
        self.create_subscription(CameraInfo, CAMERA_INFO_TOPIC, self.camera_info_callback, 1)

        # RGB 이미지 토픽 subscription 설정
        if SUB_IMAGE_TYPE == 'compressed':
            self.create_subscription(CompressedImage, RGB_TOPIC_COMPRESSED, self.rgb_callback, 1)
            self.get_logger().info(f"Subscribing to topics:\n  RGB: {RGB_TOPIC_COMPRESSED}\n  Depth: {DEPTH_TOPIC}\n  CameraInfo: {CAMERA_INFO_TOPIC}\n  MarkerPub: {MARKER_TOPIC}")
        elif SUB_IMAGE_TYPE == 'raw':
            self.create_subscription(Image, RGB_TOPIC, self.rgb_callback, 1)
            self.get_logger().info(f"Subscribing to topics:\n  RGB: {RGB_TOPIC}\n  Depth: {DEPTH_TOPIC}\n  CameraInfo: {CAMERA_INFO_TOPIC}\n  MarkerPub: {MARKER_TOPIC}")

        self.get_logger().info("📷 카메라 시작!")

        # GUI에 이미지를 퍼블리시하는 publisher 생성
        self.detect_img_publisher = self.create_publisher(CompressedImage, '/robot4/detect_img', 1)

        # 서비스 서버 시작
        self.srv = self.create_service(ObjectPosition, '/object_position', self.object_position_callback)
        self.get_logger().info("✅ /object_position 서비스 서버 시작")

        # 객체 위치 서비스 관련 변수 초기화
        self.detect_mode = False
        self.detect_done = False
        self.detect_start_time = 0.0
        self.detect_time = 5.0

        # 라벨, 라벨 좌표 변수 초기화
        self.label = ''
        self.x = None
        self.y = None


    '''클라이언트에서 label을 request를 받으면, 객체 탐지 후 객체의 좌표를 response하는 함수'''
    def object_position_callback(self, request, response):
        # depth subscribtion 시작
        # self.create_depth_subscription()

        # 요청받은 객체 라벨 저장
        self.label = request.label.lower().strip()
        self.get_logger().info(f"📥 [서비스 요청 수신] label='{self.label}'")

        self.get_logger().info(f"[INFO] YOLO 모델 시작!")

        # YOLO 인식 루프 트리거, 일정 시간이 지난 후 YOLO 종료하도록 제어
        self.detect_mode = True
        self.detect_start_time = time.time()
        
        # YOLO 인식 완료 대기
        while not self.detect_done:
            time.sleep(0.03)
            continue
        
        # 결과를 응답 메시지에 설정
        response.x = self.x
        response.y = self.y
        self.detect_done = False

        # 응답 반환 및 로그 출력
        self.get_logger().info(f"📤 [서비스 응답 반환] x={response.x} y={response.y}")

        return response


    '''내부 카메라 캘리브레이션 정보를 수신하는 함수'''
    def camera_info_callback(self, msg):
        if self.K is None:
            self.K = np.array(msg.k).reshape(3, 3)
            self.get_logger().info(f"📷 CameraInfo: fx={self.K[0,0]:.2f}, fy={self.K[1,1]:.2f}, cx={self.K[0,2]:.2f}, cy={self.K[1,2]:.2f}")


    '''RGB 이미지를 ROS 메시지에서 OpenCV 이미지로 변환하는 함수'''
    def rgb_callback(self, msg):
        try:
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


    '''ROS 토픽에서 수신된 깊이 이미지 메시지를 OpenCV 형식으로 변환하는 함수'''
    def depth_callback(self, msg):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
            with self.lock:
                self.latest_depth = depth
        except Exception as e:
            self.get_logger().error(f"Depth conversion error: {e}")


    '''TF2를 활용하여 카메라 좌표계에 있는 3D 포인트를 map 좌표계로 변환하는 함수'''
    def transform_to_map(self, pt_camera: PointStamped, class_name: str):
        try:
            pt_map = self.tf_buffer.transform(pt_camera, 'map', timeout=rclpy.duration.Duration(seconds=0.5))
            x, y, z = pt_map.point.x, pt_map.point.y, pt_map.point.z
            self.get_logger().info(f"[TF] {class_name} → map: (x={x:.2f}, y={y:.2f}, z={z:.2f})")
            return x, y, z
        except Exception as e:
            self.get_logger().warn(f"[TF] class={class_name} 변환 실패: {e}")
            return float('nan'), float('nan'), float('nan')


    '''Marker 메시지를 생성하고, 이를 특정 위치에 시각화 목적으로 퍼블리시하는 함수'''
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


    '''실시간 이미지에 대해 YOLO로 객체를 인식하고, 인식된 객체의 좌표를 추정하는 함수'''
    def inference_loop(self):
        self.get_logger().info("Inference loop started. Waiting for images...")
        while rclpy.ok() and not self.should_shutdown:
            # 이미지 및 카메라 행렬 정보 획득
            with self.lock:
                rgb, depth, K, rgb_msg = self.latest_rgb, self.latest_depth, self.K, self.latest_rgb_msg

            # 아직 이미지 또는 정보가 준비되지 않았으면 대기
            if any(v is None for v in (rgb, depth, K, rgb_msg)):
                time.sleep(0.005)
                continue
            
            # 원본 이미지 복사
            frame = rgb.copy()

            # 인식 모드일 때 YOLO 추론 실행
            if self.detect_mode and self.model:
                results = self.model(frame, verbose=False)

                for result in results:
                    if result.boxes is None:
                        continue
                    for box in result.boxes:
                        # 바운딩 박스 중심 좌표 (픽셀 기준)
                        u, v = map(int, box.xywh[0][:2].cpu().numpy())
                        if not (0 <= v < depth.shape[0] and 0 <= u < depth.shape[1]):
                            continue
                        
                        # 신뢰도 확인
                        conf = math.ceil(box.conf[0] * 100) / 100
                        if conf < CONFIDENCE_THRESHOLD:
                            continue
                        
                        # 깊이 정보를 사용해 3D 위치 추정 (camera 기준)
                        # === [여기 수정] 주변 평균으로 z 계산 ===
                        # v_min = max(0, v - 1)
                        # v_max = min(depth.shape[0], v + 2)
                        # u_min = max(0, u - 1)
                        # u_max = min(depth.shape[1], u + 2)

                        # patch = depth[v_min:v_max, u_min:u_max]
                        # z = np.nanmean(patch) / 1000.0

                        # if z <= 0 or np.isnan(z) or np.isinf(z):
                        #     continue
                        # === [수정 끝] 주변 평균으로 z 계산 ===

                        z = float(depth[v, u]) / 1000.0
                        fx, fy = K[0, 0], K[1, 1]
                        cx, cy = K[0, 2], K[1, 2]
                        x = (u - cx) * z / fx
                        y = (v - cy) * z / fy

                        # 라벨, 바운딩박스 좌표 추출
                        x1, y1, x2, y2 = map(int, box.xyxy[0].cpu().numpy())
                        conf = float(box.conf[0])
                        cls = int(box.cls[0])
                        label = self.classNames[cls] if cls < len(self.classNames) else f'class_{cls}'

                        # camera → map 좌표계 변환
                        pt_camera = PointStamped()
                        pt_camera.header.frame_id = rgb_msg.header.frame_id
                        pt_camera.header.stamp = rgb_msg.header.stamp
                        # pt_camera.header.stamp = rclpy.time.Time().to_msg()
                        pt_camera.point.x, pt_camera.point.y, pt_camera.point.z = x, y, z
                        map_x, map_y, map_z = self.transform_to_map(pt_camera, label)
                        
                        # RViz 마커로 시각화
                        if not np.isnan(map_x):
                            self.publish_marker(map_x, map_y, map_z, label)

                        # 이미지에 시각화 정보 그리기
                        cv2.rectangle(frame, (x1, y1), (x2, y2), color_dict[label], 2)
                        cv2.circle(frame, (u, v), 4, color_dict[label], -1)
                        cv2.putText(frame, f"{label} {conf:.2f}", (x1, y1 - 35), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_dict[label], 2)
                        cv2.putText(frame, f"{z:.2f}m", (x1, y1 - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_dict[label], 2)
                        cv2.putText(frame, f"({map_x:.2f}, {map_y:.2f}, {map_z:.2f})", (x1, y1 -5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_dict[label], 2)

                        # request 받은 객체라면 좌표 저장
                        if label == self.label:
                            self.x = map_x
                            self.y = map_y

                # 일정 시간 경과 후 YOLO 모델 종료
                elapsed = time.time() - self.detect_start_time
                if elapsed > self.detect_time:
                    self.get_logger().info(f"[INFO] YOLO 모델 {self.detect_time}초 경과, 메모리 해제 중...")
                    self.model = None
                    time.sleep(0.5)
                    self.detect_done = True
                    self.detect_mode = False
                    self.get_logger().info("[INFO] 인식 종료, YOLO 모델 메모리 해제 완료!")

            # frame (RGB) → CompressedImage 퍼블리시
            try:
                msg = CompressedImage()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.format = 'jpeg'
                # ret, jpeg = cv2.imencode('.jpg', cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
                ret, jpeg = cv2.imencode('.jpg', frame)
                if ret:
                    msg.data = jpeg.tobytes()
                    self.detect_img_publisher.publish(msg)
            except Exception as e:
                self.get_logger().error(f"이미지 퍼블리시 오류: {e}")

            # 결과 프레임 업데이트
            with self.lock:
                self.processed_frame = frame
            time.sleep(0.005)


def main():
    rclpy.init()
    node = Robot4Vision()

    try:
        threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
        threading.Thread(target=node.inference_loop, daemon=True).start()

        while rclpy.ok() and not node.should_shutdown:
            with node.lock:
                frame = node.processed_frame.copy() if node.processed_frame is not None else None
            # if frame is not None:
            #     cv2.imshow("robot4_vision frame", frame)
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
        sys.exit(0)

if __name__ == '__main__':
    main()