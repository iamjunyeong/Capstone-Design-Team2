#!/usr/bin/env python3
import rclpy, os, cv2
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Int8
from message_filters import Subscriber, ApproximateTimeSynchronizer
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory

# 클래스 이름 및 필터링 기준
NAMES = ['pole','person','bollard','bicycle','motorcycle','bench','barricade','scooter']
STATIC_LABELS = {'pole','bollard','bench','barricade'}
DYNAMIC_LABELS = {'person','bicycle','motorcycle','scooter'}
RELEVANT = STATIC_LABELS | DYNAMIC_LABELS

DISTANCE_THRESHOLD = 5.0  # meter

class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector')
        self.bridge = CvBridge()

        # 모델 로드
        # pkg_share = get_package_share_directory('vision_bringup')
        # model_path = os.path.join(pkg_share, 'model', 'obstacle_detector.pt')
        model_path = '/home/ubuntu/capstone_ws/src/Capstone-Design-Team2/vision_bringup/model/obstacle_detector.pt'
        self.model = YOLO(model_path)

        # 퍼블리셔
        self.pub_info  = self.create_publisher(Int8 , '/obstacle_info'          , 1)
        self.pub_debug = self.create_publisher(Image, '/obstacle_debug/image_raw', 1)

        # 동기화 구독자 설정
        qos = rclpy.qos.QoSProfile(depth=10)
        sub_rgb   = Subscriber(self, Image, '/camera/camera/color/image_raw', qos_profile=qos)
        sub_depth = Subscriber(self, Image, '/camera/camera/aligned_depth_to_color/image_raw', qos_profile=qos)
        ats = ApproximateTimeSynchronizer([sub_rgb, sub_depth], queue_size=10, slop=0.1)
        ats.registerCallback(self.cb_synced)

        self.get_logger().info('ObstacleDetector with depth filtering ready.')
    def cb_synced(self, img_msg, depth_msg):
        try:
            # 원본 encoding 그대로 변환
            color = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='passthrough')
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')

            # rgb → bgr로 변환 (YOLO는 일반적으로 BGR을 기대)
            if img_msg.encoding.lower() == 'rgb8':
                color = cv2.cvtColor(color, cv2.COLOR_RGB2BGR)

            # depth 타입 확인
            if depth.dtype != np.uint16:
                self.get_logger().warn(f"Expected uint16 depth, got {depth.dtype}")
                return

        except Exception as e:
            self.get_logger().error(f'Image conversion failed: {e}')
            return

        # YOLO inference
        res = self.model(color)[0]
        boxes, classes = res.boxes.xyxy, res.boxes.cls
        near_s, near_d = 0, 0
        annotated = res.plot()

        h, w = depth.shape
        for (x1, y1, x2, y2), cid in zip(boxes.cpu().numpy(), classes.cpu().numpy()):
            label = NAMES[int(cid)] if int(cid) < len(NAMES) else None
            if label not in RELEVANT: continue

            # 바운딩박스 안의 depth 값 추출
            x1, y1, x2, y2 = map(int, [x1, y1, x2, y2])
            x1, y1 = max(0, x1), max(0, y1)
            x2, y2 = min(w-1, x2), min(h-1, y2)
            box_depth = depth[y1:y2, x1:x2]
            valid = box_depth[box_depth > 0]
            if len(valid) == 0: continue
            median_m = np.median(valid) * 1e-3  # mm → m
            if median_m > DISTANCE_THRESHOLD: continue

            # 클래스별 카운트
            if label in DYNAMIC_LABELS: near_d += 1
            else: near_s += 1

            # 거리 정보 디버그 이미지에 표시 (바운딩박스 오른쪽 위)
            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 255), 2)
            text = f"{label}: {median_m:.2f}m"
            text_size, _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
            text_x = x2 - text_size[0]  # 오른쪽 정렬
            text_y = y1 + text_size[1]
            cv2.putText(annotated, text,
                        (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (0, 0, 255), 2, cv2.LINE_AA)

        # 코드 결정 및 발행
        code = 0 if near_s==0 and near_d==0 else 1 if near_s>0 and near_d==0 else 2 if near_d>0 and near_s==0 else 3
        self.pub_info.publish(Int8(data=code))

        # 디버깅 이미지 왼쪽 상단에 obstacle code 표시
        cv2.putText(annotated, f"Obstacle Code: {code}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                    (0, 255, 0), 2, cv2.LINE_AA)

        # 디버깅 이미지 발행
        dbg_msg = self.bridge.cv2_to_imgmsg(annotated, 'bgr8')
        dbg_msg.header = img_msg.header
        self.pub_debug.publish(dbg_msg)


def main():
    rclpy.init()
    node = ObstacleDetector()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
