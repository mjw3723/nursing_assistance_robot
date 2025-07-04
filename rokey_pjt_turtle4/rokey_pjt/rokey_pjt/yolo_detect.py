import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import math
import os
import sys
from ultralytics import YOLO
import numpy as np

# ========================
# 상수 정의
# ========================
# MODEL_PATH = '/home/mi/rokey_ws/model/yolov8n.pt'  # YOLO 모델 경로
RGB_TOPIC = '/robot4/oakd/rgb/image_raw'  # 구독할 이미지 토픽 이름
RGB_TOPIC_COMPRESSED = '/robot4/oakd/rgb/image_raw/compressed'  # 구독할 이미지 토픽 이름
TARGET_CLASS_ID = 0  # 탐지할 클래스 ID (예: car = 0)

MODEL_PATH = '/home/hongha/YOLO Dataset_turtlebot4/runs/detect/yolov8s/weights/best.pt'  # YOLO 모델 경로 경로
CONFIDENCE_THRESHOLD = 0.85

SUB_IMAGE_TYPE = 'compressed'
# SUB_IMAGE_TYPE = 'raw'

# 인식된 객체의 시각화 색상
color_dict = {
    "bandage": (0, 255, 255),   # cyan (BGR)
    "codaewon": (0, 255, 0),    # lime/yellow-green (BGR)
    "cup": (0, 0, 255),         # red (BGR)
    "tylenol": (255, 0, 255)    # purple (BGR)
}

# ========================
# YOLO 객체 인식 노드 정의
# ========================
class YOLOViewerNode(Node):
    def __init__(self):
        super().__init__('yolo_viewer_node')

        # YOLO 모델 로딩
        if not os.path.exists(MODEL_PATH):
            self.get_logger().error(f"Model not found: {MODEL_PATH}")
            sys.exit(1)
        self.model = YOLO(MODEL_PATH)
        self.class_names = getattr(self.model, 'names', [])  # 클래스 이름 리스트

        # 이미지 메시지 → OpenCV 이미지 변환용
        self.bridge = CvBridge()

        # ROS2 이미지 토픽 구독 설정
        # self.subscription = self.create_subscription(Image, RGB_TOPIC, self.image_callback, 10)

        if SUB_IMAGE_TYPE == 'compressed':
            self.subscription = self.create_subscription(CompressedImage, RGB_TOPIC_COMPRESSED, self.image_callback, 10)
        elif SUB_IMAGE_TYPE == 'raw':
            self.subscription = self.create_subscription(Image, RGB_TOPIC, self.image_callback, 10)

        # 종료 요청 여부를 나타내는 플래그 (OpenCV 창에서 'q' 누르면 True)
        self.should_shutdown = False

        # OpenCV 창 이름
        self.window_name = "YOLO Detection"

    def image_callback(self, msg):
        # ROS 이미지 메시지를 OpenCV 이미지로 변환
        # img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        if SUB_IMAGE_TYPE == 'compressed':
            np_arr = np.frombuffer(msg.data, np.uint8)
            img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        elif SUB_IMAGE_TYPE == 'raw':
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # YOLO 추론 실행 (stream=True로 반복 객체 결과 얻기)
        results = self.model(img, stream=True)
        object_count = 0

        # 결과 순회
        for r in results:
            for box in r.boxes:
                cls = int(box.cls[0])
                # if cls != TARGET_CLASS_ID:
                #     continue  # 대상 클래스가 아니면 건너뜀

                conf = math.ceil(box.conf[0] * 100) / 100
                if conf < CONFIDENCE_THRESHOLD:
                    continue  # 신뢰도 낮으면 건너뜀

                # 바운딩 박스 좌표 및 confidence 가져오기
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                label = self.class_names[cls] if cls < len(self.class_names) else f"class_{cls}"

                # 바운딩 박스와 라벨을 이미지에 표시
                cv2.rectangle(img, (x1, y1), (x2, y2), color_dict[label], 2)
                cv2.putText(img, f"{label}: {conf}", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color_dict[label], 2)
                object_count += 1

        # 전체 탐지 개수도 이미지에 표시
        cv2.putText(img, f"Objects: {object_count}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        # 시각화를 위해 2배 확대 후 OpenCV로 표시
        # display_img = cv2.resize(img, (img.shape[1]*3, img.shape[0]*3))
        display_img = cv2.resize(img, (img.shape[1], img.shape[0]))
        cv2.imshow(self.window_name, display_img)

        # 키 입력 대기 ('q' 누르면 종료 플래그 설정)
        key = cv2.waitKey(1)
        if key == ord('q'):
            self.should_shutdown = True
            self.get_logger().info("Q pressed. Shutting down...")

# ========================
# 메인 함수
# ========================
def main():
    rclpy.init()
    node = YOLOViewerNode()

    try:
        # ROS2 이벤트 처리 루프 (OpenCV와 병행 실행을 위해 spin_once 사용)
        while rclpy.ok() and not node.should_shutdown:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        # 자원 정리 및 안전 종료
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()
        print("Shutdown complete.")
        sys.exit(0)

if __name__ == '__main__':
    main()
