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
from rokey_interfaces.msg import ArucoMarker
import subprocess
from geometry_msgs.msg import Twist
import time  # ← 이 줄을 꼭 추가하세요

###############아르코마커#########################아르코마커#######################아르코마커########################아르코마커#########################아르코마커#####################################
#################아르코마커########################################아르코마커###################################################아르코마커########################################아르코마커##########
###############아르코마커#########################아르코마커#######################아르코마커########################아르코마커#########################아르코마커#####################################
#################아르코마커########################################아르코마커###################################################아르코마커########################################아르코마커##########
###############아르코마커#########################아르코마커#######################아르코마커########################아르코마커#########################아르코마커#####################################
#################아르코마커########################################아르코마커###################################################아르코마커########################################아르코마커##########
RGB_TOPIC ='/robot1/oakd/rgb/preview/image_raw'
CALIBRATION_FILE_PATH = 'camera_calibration.pkl' # 캘리브레이션 데이터 파일 경로
MARKER_SIZE = 0.05  # ArUco 마커 크기 (미터 단위, 예: 5cm) - 실제 마커 크기와 정확히 일치해야 합니다!
class YoloSubscriber(Node):
    def __init__(self):
        super().__init__('yolo_subscriber')
        self.bridge = CvBridge()
        self.model = YOLO('/home/rokey/rokey_ws/src/rokey_pjt/rokey_pjt/best.pt',verbose=False)
        self.init_aruco()
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
        self.detected_marker_id_2 = False  # 2번 마커 트리거 중복 방지용
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.person_published = False
        self.person_cleared_published = False
        self.person_pub = self.create_publisher(Bool, '/person_detected', qos_profile)
        self.cleared_pub = self.create_publisher(Bool, '/person_cleared', qos_profile)
        self.marker_pub = self.create_publisher(ArucoMarker, '/aruco_marker', 10)
        self.no_person_frame_count = 0
        self.no_person_frame_threshold = 5 
        self.distance_m = None
        self.cmd_vel_pub = self.create_publisher(Twist, '/robot1/cmd_vel', 10)
        self.face_trigger_pub = self.create_publisher(Bool, '/face_detection_start', 10)
        self.face_triggered = False

       

    def init_aruco(self):
        # Aruco
        self.camera_matrix = None
        self.dist_coeffs = None
        try:
            # script_dir = os.path.dirname(os.path.abspath(__file__))
            # self.get_logger().info(script_dir)
            full_calibration_path = '/home/moon/nursing_assistance_robot/src/nursing_assistance_robot/nursing_assistance_robot/camera_calibration.pkl'
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
        self.get_logger().info(f"YOLO 시작중")
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

                    marker_id = int(ids[i][0])

                    if marker_id == 2:
                        distance = float(tvec[2])
                        self.get_logger().info(f"🎯 ArUco ID=2 거리: {distance:.2f}m")

                        # ✅ 1m 기준으로 변경하고, 1m 이상이면 반복적으로 전진
                        if distance > 1.0:
                            self.get_logger().info("➡️ 거리 1m 이상 → 전진 중...")
                            self.forward_slightly()
                            self.face_triggered = False  # 계속 초기화하여 반복 가능하게 함
                        elif distance <= 1.0 and not self.face_triggered:
                            self.get_logger().info(f"📏 얼굴 인식 시작 (거리={distance:.2f}m)")
                            self.trigger_face_detection()
                            self.face_triggered = True

                    # 마커 메시지 퍼블리시
                    rot_matrix, _ = cv2.Rodrigues(rvec)
                    try:
                        euler_angles = cv2.RQDecomp3x3(rot_matrix)[0]
                    except cv2.error:
                        self.get_logger().warn(f"ArUco: Rotation 실패 - ID {ids[i][0]}")
                        continue
                    marker_msg = ArucoMarker()
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
        # 예: /face_detection_start 토픽 발행 또는 서비스 호출
        msg = Bool()
        msg.data = True
        self.face_trigger_pub.publish(msg)  # 퍼블리셔 선언 필요!

    def forward_slightly(self):
        twist = Twist()
        twist.linear.x = 0.07  # 아주 천천히
        start_time = time.time()
        while time.time() - start_time < 5:
            self.cmd_vel_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.1)
        twist.linear.x = 0.0
        self.cmd_vel_pub.publish(twist)

    

        # 💡 vital_check_node2.py 실행
        try:
        # ROS2 환경을 소싱한 후 vital 노드 실행
            subprocess.Popen([
                'bash', '-c',
                'source /opt/ros/humble/setup.bash && '
                'source ~/nursing_assistance/install/setup.bash && '
                'ros2 run nursing_assistance vital'
            ])
            print("🩺 vital_check_node2 실행됨")
        except Exception as e:
            print(f"❌ vital_check_node2 실행 실패: {e}")

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
    def transform_aruco_point_to_map_x(self, aruco_pos_x: float, aruco_pos_y: float, aruco_pos_z: float) -> float | None:
        """
        아루코 마커의 카메라 좌표계 위치를 map 좌표계로 변환하고,
        변환된 점의 x 좌표만 반환합니다.
        """
        aruco_point_camera = PointStamped()
        aruco_point_camera.header.stamp = self.get_clock().now().to_msg()
        # 이 부분이 중요합니다. '/robot1/oakd/rgb/preview/image_raw' 토픽의 frame_id를 확인하여 정확히 입력하세요.
        # 일반적인 OAK-D ROS 드라이버의 convention에 따라 'robot1_oakd_rgb_camera_optical_frame' 또는 유사한 이름이 될 수 있습니다.
        # 정확한 이름은 `ros2 topic echo /robot1/oakd/rgb/preview/image_raw`를 실행하여 메시지의 `header.frame_id`를 확인하거나,
        # `ros2 run tf2_tools view_frames` 명령으로 TF 트리를 확인하여 찾을 수 있습니다.
        aruco_point_camera.header.frame_id = 'robot1_oakd_rgb_camera_optical_frame' # <-- 이곳을 실제 카메라 프레임 ID로 변경!

        aruco_point_camera.point.x = aruco_pos_x
        aruco_point_camera.point.y = aruco_pos_y
        aruco_point_camera.point.z = aruco_pos_z

        try:
            point_map = self.tf_buffer.transform(
                aruco_point_camera,
                'map', # 목표 좌표계
                timeout=rclpy.duration.Duration(seconds=1.0) # 변환 대기 시간
            )
            self.get_logger().info(f"ArUco 원본 ({aruco_point_camera.point.x:.2f},{aruco_point_camera.point.y:.2f},{aruco_point_camera.point.z:.2f}) "
                                   f"를 Map으로 변환: ({point_map.point.x:.2f}, {point_map.point.y:.2f}, {point_map.point.z:.2f})")
            return point_map.point.x # 변환된 점의 x 좌표만 반환

        except tf2_ros.TransformException as ex:
            self.get_logger().warn(f"ArUco: map으로의 TF 변환 실패: {ex}. Frame ID ('{aruco_point_camera.header.frame_id}') 또는 'map' 프레임이 TF 트리에 없는지 확인하세요.")
            return None
        except Exception as e:
            self.get_logger().error(f"ArUco: 예상치 못한 오류 발생: {e}")
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
