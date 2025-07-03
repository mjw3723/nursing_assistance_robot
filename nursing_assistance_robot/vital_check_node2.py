#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import os

import cv2
import numpy as np
import time
from scipy.signal import butter, lfilter, find_peaks

# from utils import kcf
# from utils import skinsegment

from nursing_assistance_robot.utils import kcf
from nursing_assistance_robot.utils import skinsegment
from ament_index_python.packages import get_package_share_directory


class RPPGChromNode(Node):
    def __init__(self):
        super().__init__('rppg_chrom_node')
        self.get_logger().info("🎥 rPPG-Chrom Node Started")
        self.publisher_ = self.create_publisher(Float32, '/bpm', 10)
        self.spo2_publisher = self.create_publisher(Float32, '/spo2', 10)
        self.sbp_publisher = self.create_publisher(Float32, '/sbp', 10)
        self.dbp_publisher = self.create_publisher(Float32, '/dbp', 10)


        # 얼굴 검출 모델 로드
        model_dir = os.path.join(get_package_share_directory('nursing_assistance_robot'), 'model')
        model_pb = os.path.join(model_dir, 'face_detector.pb')
        model_pbtxt = os.path.join(model_dir, 'face_detector.pbtxt')
        self.detector = cv2.dnn.readNetFromTensorflow(model_pb, model_pbtxt)
        self.tracker = kcf.KCFTracker()
        self.is_tracking = False

        # 웹캠
        self.cap = cv2.VideoCapture(0)
        self.cap.set(3, 640)
        self.cap.set(4, 480)

        # 설정값
        self.buffer_size = 300
        self.fps = 30
        self.frame_buffer = []
        self.last_bpm_time = time.time()
        self.detect_th = 0.5

        # 주기적 콜백
        self.timer = self.create_timer(1.0 / self.fps, self.process_frame)
        # 클래스 변수로 추가
        self.latest_bpm = None
        self.latest_spo2 = None
        self.latest_sbp = None
        self.latest_dbp = None

        self.measure_start_time = time.time()
        self.bpm_list = []
        self.spo2_list = []
        self.sbp_list = []
        self.dbp_list = []
        self.published_once = False



    def bandpass_filter(self, signal, fs, low=0.7, high=4.0):
        nyq = 0.5 * fs
        low /= nyq
        high /= nyq
        b, a = butter(3, [low, high], btype='band')
        return lfilter(b, a, signal)

    def chrom_method(self, r, g, b):
        r = np.array(r)
        g = np.array(g)
        b = np.array(b)
        Xs = 3 * r - 2 * g
        Ys = 1.5 * r + g - 1.5 * b
        S = Xs / (Ys + 1e-6)
        return S - np.mean(S)

    def estimate_bpm(self, ppg_signal):
        filtered = self.bandpass_filter(ppg_signal, self.fps)
        peaks, _ = find_peaks(filtered, distance=self.fps/2)
        if len(peaks) >= 2:
            peak_intervals = np.diff(peaks) / self.fps
            avg_interval = np.mean(peak_intervals)
            return 60.0 / avg_interval
        return None
    
    def estimate_bp(self, bpm):
        # 아주 단순한 경험적 추정 모델 (정밀하지 않음)
        sbp = 0.5 * bpm + 90
        dbp = 0.3 * bpm + 50
        return sbp, dbp
    
    def estimate_spo2(self, r_vals, g_vals, b_vals):
        r = np.array(r_vals)
        g = np.array(g_vals)

        # DC (mean) + AC (std)
        r_dc = np.mean(r)
        g_dc = np.mean(g)
        r_ac = np.std(r)
        g_ac = np.std(g)

        # 비율 기반 추정 (간단한 모델)
        ratio = (r_ac / r_dc) / (g_ac / g_dc + 1e-6)
        spo2 = 110 - 15 * ratio  # 대략적인 경험적 추정 공식

        return np.clip(spo2, 70, 100)

    def detect_face(self, frame):
        blob = cv2.dnn.blobFromImage(cv2.resize(frame, (300, 300)),
                                    1.0, (300, 300), [104., 117., 123.], False, True)
        self.detector.setInput(blob)
        detections = self.detector.forward()
        h, w = frame.shape[:2]
        for i in range(detections.shape[2]):
            confidence = detections[0, 0, i, 2]
            if confidence > self.detect_th:
                box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
                x1, y1, x2, y2 = box.astype("int")
                return [x1, y1, x2 - x1, y2 - y1]
        return None

    def process_frame(self):
        if self.published_once:
            self.cap.release()
            cv2.destroyAllWindows()
            rclpy.shutdown()
            return

        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("❌ Webcam not ready")
            return

        if not self.is_tracking:
            bbox = self.detect_face(frame)
            if bbox:
                self.tracker.init(frame, bbox)
                self.is_tracking = True
            else:
                return  # 얼굴 못 찾으면 프레임 처리 중단
        else:
            ok, bbox = self.tracker.update(frame)
            if not ok or bbox is None:
                self.is_tracking = False
                return

        x, y, w, h = [int(i) for i in bbox]
        roi = frame[y:y+h, x:x+w]
        skin_mask = skinsegment.create_skin_mask(roi)
        if skin_mask is None:
            return

        r = np.mean(roi[:, :, 2][skin_mask == 1])
        g = np.mean(roi[:, :, 1][skin_mask == 1])
        b = np.mean(roi[:, :, 0][skin_mask == 1])

        if np.isnan(r) or np.isnan(g) or np.isnan(b):
            return

        self.frame_buffer.append([r, g, b])
        if len(self.frame_buffer) > self.buffer_size:
            self.frame_buffer.pop(0)

        bpm = None
        spo2 = None
        sbp = None
        dbp = None
        
        if len(self.frame_buffer) >= 100:
            r_vals = [x[0] for x in self.frame_buffer]
            g_vals = [x[1] for x in self.frame_buffer]
            b_vals = [x[2] for x in self.frame_buffer]

            # [여기서 정규화 추가]
            r_mean = np.mean(r_vals)
            g_mean = np.mean(g_vals)
            b_mean = np.mean(b_vals)

            r_vals = r_vals / (r_mean + 1e-6)
            g_vals = g_vals / (g_mean + 1e-6)
            b_vals = b_vals / (b_mean + 1e-6)

            ppg = self.chrom_method(r_vals, g_vals, b_vals)
            bpm = self.estimate_bpm(ppg)

        # if bpm and 40 < bpm < 180 and (time.time() - self.last_bpm_time) > 1:
        #     self.publisher_.publish(Float32(data=bpm))
        #     self.get_logger().info(f'BPM: {bpm:.2f}')
        #     self.latest_bpm = bpm
        #     self.last_bpm_time = time.time()

        if len(self.frame_buffer) >= 100:
            r_vals = [x[0] for x in self.frame_buffer]
            g_vals = [x[1] for x in self.frame_buffer]
            b_vals = [x[2] for x in self.frame_buffer]
            ppg = self.chrom_method(r_vals, g_vals, b_vals)
            bpm = self.estimate_bpm(ppg)

            if bpm and 40 < bpm < 180:
                spo2 = self.estimate_spo2(r_vals, g_vals, b_vals)
                sbp, dbp = self.estimate_bp(bpm)

                self.bpm_list.append(bpm)
                self.spo2_list.append(spo2)
                self.sbp_list.append(sbp)
                self.dbp_list.append(dbp)

                # 5초 측정 후 한 번만 publish
                if time.time() - self.measure_start_time > 5 and not self.published_once:
                    bpm_med = float(np.median(self.bpm_list))
                    spo2_med = float(np.median(self.spo2_list))
                    sbp_med = float(np.median(self.sbp_list))
                    dbp_med = float(np.median(self.dbp_list))

                    self.publisher_.publish(Float32(data=bpm_med))
                    self.spo2_publisher.publish(Float32(data=spo2_med))
                    self.sbp_publisher.publish(Float32(data=sbp_med))
                    self.dbp_publisher.publish(Float32(data=dbp_med))

                    self.get_logger().info(f"📡 Published once - BPM: {bpm_med:.1f}, SpO2: {spo2_med:.1f}%, BP: {sbp_med:.0f}/{dbp_med:.0f}")

                    self.latest_bpm = bpm_med
                    self.latest_spo2 = spo2_med
                    self.latest_sbp = sbp_med
                    self.latest_dbp = dbp_med


                    self.published_once = True  # 다시 publish하지 않도록


        # SpO2 계산
            # spo2 = self.estimate_spo2(r_vals, g_vals, b_vals)
            # if spo2:
            #     self.latest_spo2 = spo2
            #     self.spo2_publisher.publish(Float32(data=spo2))
            #     self.get_logger().info(f'SpO₂: {spo2:.2f}%')

            # # 혈압 계산
            # sbp, dbp = self.estimate_bp(bpm)
            # self.latest_sbp = sbp
            # self.latest_dbp = dbp
            # self.sbp_publisher.publish(Float32(data=sbp))
            # self.dbp_publisher.publish(Float32(data=dbp))
            # self.get_logger().info(f'BP: {sbp:.0f}/{dbp:.0f} mmHg')

        # 화면 표시
        if self.latest_bpm:
            text_x = x + w + 10
            text_y = y + 30
            spacing = 30
            bpm_text = f"{self.latest_bpm:.1f} BPM"
            cv2.putText(frame, bpm_text, (text_x, text_y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            if self.latest_spo2 is not None:
                spo2_text = f"SpO2: {self.latest_spo2:.1f}%"
                cv2.putText(frame, spo2_text, (text_x, text_y + spacing),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

            if self.latest_sbp is not None and self.latest_dbp is not None:
                bp_text = f"BP: {self.latest_sbp:.0f}/{self.latest_dbp:.0f} mmHg"
                cv2.putText(frame, bp_text, (text_x, text_y + spacing * 2),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)



        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.imshow("rPPG Chrom", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.cap.release()
            cv2.destroyAllWindows()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = RPPGChromNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()