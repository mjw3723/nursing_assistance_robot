import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import tf2_ros
import numpy as np # ArUco 포즈 추정에 필요
from visualization_msgs.msg import Marker
from std_msgs.msg import Bool 
from std_msgs.msg import Float64
from geometry_msgs.msg import Point
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import pickle
import os
from rokey_interfaces.msg import Aruco_Marker
import subprocess
RGB_TOPIC = '/robot1/oakd/rgb/preview/image_raw' # RGB 이미지 토픽
CALIBRATION_FILE_PATH = 'camera_calibration.pkl' # 캘리브레이션 데이터 파일 경로
MARKER_SIZE = 0.05  # ArUco 마커 크기 (미터 단위, 예: 5cm) - 실제 마커 크기와 정확히 일치해야 합니다!
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
            RGB_TOPIC,
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
        self.aruco_start = self.create_publisher(
            
        )
        self.detected_marker_id_2 = False  # 2번 마커 트리거 중복 방지용
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.person_published = False
        self.person_cleared_published = False
        self.person_pub = self.create_publisher(Bool, '/person_detected', qos_profile)
        self.cleared_pub = self.create_publisher(Bool, '/person_cleared', qos_profile)
        self.marker_pub = self.create_publisher(Aruco_Marker, '/aruco_marker', 10)
        self.no_person_frame_count = 0
        self.no_person_frame_threshold = 5 
        self.distance_m = None

        self.face_trigger_pub = self.create_publisher(Bool, '/face_detection_start', 10)
        self.face_triggered = False
        
    def init_aruco(self):
        # Aruco
        self.camera_matrix = None
        self.dist_coeffs = None
        try:
            script_dir = os.path.dirname(os.path.abspath(__file__))
            full_calibration_path = os.path.join(script_dir, CALIBRATION_FILE_PATH)
            with open(full_calibration_path, 'rb') as f:
                calibration_data = pickle.load(f)
            self.camera_matrix = calibration_data['camera_matrix']
            self.dist_coeffs = calibration_data['dist_coeffs']
            self.get_logger().info("ArUco: Calibration data loaded successfully.")
        except FileNotFoundError:
            self.get_logger().error(f"ArUco: Error: Camera calibration file not found at {full_calibration_path}")
            self.get_logger().error("ArUco: Please ensure 'camera_calibration.pkl' exists in the same directory as this script, or provide the full path.")
        except Exception as e:
            self.get_logger().error(f"ArUco: Error loading calibration data: {e}")  
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
        self.aruco_params = cv2.aruco.DetectorParameters() 
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        self.get_logger().info("ArUco: Detector initialized with DICT_5X5_250.")

        half_size = MARKER_SIZE / 2.0
        self.obj_points = np.array([
            [-half_size,  half_size, 0],  # Top-left
            [ half_size,  half_size, 0],  # Top-right
            [ half_size, -half_size, 0],  # Bottom-right
            [-half_size, -half_size, 0]   # Bottom-left
        ], dtype=np.float32)

    
    def listener_callback(self, msg):
        person_detected_now = False
        # ROS 이미지 → OpenCV 이미지
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.frame_processed = cv_image.copy()
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
                    if self.distance_m <= 3.0:
                        person_detected_now = True
                        if not self.person_published:
                            self.publish_person_detect()
                    else:
                        person_detected_now = False
        self.aruco_run()
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
    
    def aruco_run(self):
        if self.frame_processed is not None:
            frame_undistorted = cv2.undistort(self.frame_processed, self.camera_matrix, self.dist_coeffs)
            corners, ids, _ = self.detector.detectMarkers(frame_undistorted)
            if ids is not None:
                for i in range(len(ids)):
                    ret, rvec, tvec = cv2.solvePnP(
                        self.obj_points, corners[i],
                        self.camera_matrix, self.dist_coeffs
                    )
                    if not ret:
                        continue
                    marker_id = int(ids[i][0])  # <-- 마커 ID 가져오기
                    if marker_id == 2:
                        distance = float(tvec[2])
                        self.get_logger().info(f"🎯 ArUco ID=2 거리: {distance:.2f}m")
                        
                        if 0.5 < distance < 1.0:  # 거리가 너무 멀면...
                            self.forward_slightly()  # 전진 명령

                        elif distance <= 0.5 and not self.face_triggered:
                            self.get_logger().info("📏 거리 0.5m 이하 → 얼굴 인식 시작 트리거")
                            msg = Bool()
                            msg.data = True
                            self.face_trigger_pub.publish(msg)
                            self.face_triggered = True
                    # 회전 벡터 → 오일러 변환
                    rot_matrix, _ = cv2.Rodrigues(rvec)
                    try:
                        euler_angles = cv2.RQDecomp3x3(rot_matrix)[0]
                    except cv2.error:
                        self.get_logger().warn(f"ArUco: Rotation 실패 - ID {ids[i][0]}")
                        continue
                    marker_msg = Aruco_Marker()
                    marker_msg.id = int(ids[i][0])
                    marker_msg.pos_x = float(tvec[0])
                    marker_msg.pos_y = float(tvec[1])
                    marker_msg.pos_z = float(tvec[2])
                    marker_msg.rot_x = float(euler_angles[0])
                    marker_msg.rot_y = float(euler_angles[1])
                    marker_msg.rot_z = float(euler_angles[2])
                    self.marker_pub.publish(marker_msg)

    def distance_callback(self,msg:Float64):
        self.distance_m = msg.data
        
    def trigger_face_detection(self):
        self.get_logger().info("🎯 얼굴 인식 및 심박수 루틴 트리거!")

        # 토픽 퍼블리시 (선택)
        msg = Bool()
        msg.data = True
        self.face_trigger_pub.publish(msg)

        # 💡 vital_check_node2.py 실행
        try:
            subprocess.Popen(["ros2", "run", "rokey_pjt", "vital"])
            self.get_logger().info("🩺 vital_check_node2 노드 실행됨")
        except Exception as e:
            self.get_logger().error(f"실행 실패: {e}")

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
