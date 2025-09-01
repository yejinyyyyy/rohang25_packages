#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time
import cv2
import torch
import numpy as np
from sensor_msgs.msg import CompressedImage
from yolov11_msgs.msg import Detection
from std_msgs.msg import Bool
from ultralytics import YOLO
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy

class YOLOv11(Node):
    def __init__(self):
        super().__init__('yolov11_node')

        # QoS 설정
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        # 파라미터로 카메라 소스 선택 (압축 이미지 토픽)
        self.declare_parameter('video_source', 'sim_camera')
        camera_select = self.get_parameter('video_source').value
        if camera_select == 'real_camera':
            self.get_logger().info('Using REAL(ZR10) compressed image topic.')
            cam_topic = '/camera/image_raw/compressed'
        elif camera_select == 'sim_camera':
            self.get_logger().info('Using GAZEBO compressed image topic.')
            cam_topic = '/camera2/image_raw/compressed'
        else:
            self.get_logger().warn(f'Unknown video_source "{camera_select}", defaulting to GAZEBO compressed image.')
            cam_topic = '/camera2/image_raw/compressed'

        # 제어 플래그
        self.control_flag = True
        self.log_counter = 0

        # CvBridge 및 퍼블리셔
        self.bridge = CvBridge()
        self.pub_detection = self.create_publisher(Detection, '/yolov11/detection', 10)
        self.pub_result_img = self.create_publisher(CompressedImage, '/yolov11/result_img/compressed', 10)

        # 구독자: 압축 이미지 & 제어 플래그
        self.create_subscription(CompressedImage, cam_topic, self.image_callback, qos_profile)
        self.create_subscription(Bool, '/vision_nodes_control_flag', self.flag_callback, qos_profile)

        # YOLO 모델 로드
        self.model = YOLO('/home/yejin/weights.pt')
        if torch.cuda.is_available():
            self.get_logger().info("✅ CUDA is available. Using GPU.")
            self.model.to(0)
        else:
            self.get_logger().warn("❌ CUDA is NOT available. Using CPU.")

        # FPS 계산용
        self.frame_count = 0
        self.start_time = time.time()
        self.fps_text = "FPS: 0"

        self.get_logger().info('\n -- YOLO compressed node initiated -- ')

    def flag_callback(self, msg: Bool):
        self.control_flag = msg.data

    def image_callback(self, msg: CompressedImage):
        if not self.control_flag:
            return

        try:
            # 압축 이미지 → OpenCV BGR 이미지
            frame = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"cv_bridge 변환 오류: {e}")
            return

        # 해상도 축소
        frame = cv2.resize(frame, (640, 640))

        # 추론
        results = self.model(frame, show=False, conf=0.8)
        self.detect_show(results, frame)

    def detect_show(self, results, frame):
        self.frame_count += 1
        detection = Detection()
        published = False

        for result in results[0].boxes:
            cls_id = int(result.cls.item())
            conf = result.conf[0].item()
            x, y, w, h = result.xywh[0]

            detection.class_id = results[0].names[cls_id]
            detection.score = conf
            detection.center_x = round(x.item())
            detection.center_y = round(y.item())
            self.pub_detection.publish(detection)
            published = True

            # 바운딩 박스 시각화
            x1 = int(x - w / 2)
            y1 = int(y - h / 2)
            x2 = int(x + w / 2)
            y2 = int(y + h / 2)
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame,
                        f"{results[0].names[cls_id]} {conf:.2f}",
                        (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 255, 0),
                        2)

            if self.log_counter % 5 == 0:
                self.get_logger().info(
                    f'\n--------------------'
                    f'\nclass_id: {detection.class_id}'
                    f'\ncenter x: {detection.center_x}'
                    f'\ncenter y: {detection.center_y}'
                    f'\n--------------------'
                )
            self.log_counter += 1

        # 검출 없을 때 빈 메시지 발행
        if not published:
            detection.class_id = "0"
            detection.score = 0.0
            detection.center_x = 0
            detection.center_y = 0
            self.pub_detection.publish(detection)

        # 1초마다 FPS 계산
        if time.time() - self.start_time >= 1:
            fps = self.frame_count / (time.time() - self.start_time)
            self.fps_text = f"FPS: {fps:.2f}"
            self.frame_count = 0
            self.start_time = time.time()

        # FPS 표시
        cv2.putText(frame,
                    self.fps_text,
                    (20, 50),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 230, 0),
                    2,
                    cv2.LINE_AA)

        # 압축 이미지 퍼블리시
        self.publish_image(frame)

    def publish_image(self, frame_raw):
        try:
            comp_msg = self.bridge.cv2_to_compressed_imgmsg(frame_raw, dst_format='jpeg')
            self.pub_result_img.publish(comp_msg)
        except Exception as e:
            self.get_logger().error(f"이미지 퍼블리시 오류: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = YOLOv11()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

