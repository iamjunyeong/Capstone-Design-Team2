#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int8
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

names = ['car', 'truck', 'bicycle', 'motorcycle', 'bus', 'scooter']
height_thresholds = {
    'car':        60,
    'truck':      80,
    'bicycle':    50,
    'motorcycle': 50,
    'bus':        80,
    'scooter':    40,
}
dynamic_labels = set(names)

FULL_FRAME_RATIO = 0.95
MOVE_RATIO_THRESHOLD = 0.10
EPS = 1e-6

class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector_crosswalk')

        self.declare_parameter('camera_topics', ['/usb_cam_0/image_raw', '/usb_cam_1/image_raw'])
        camera_topics = self.get_parameter('camera_topics').value

        self.pub_info = self.create_publisher(Int8, '/obstacle_crosswalk_info', 1)
        self.bridge = CvBridge()
        self.model = YOLO('/home/loe/workspace/github/Capstone-Design-Team2/vision_bringup/model/obstacle_detector_crosswalk.pt')

        # 퍼블리셔 및 상태 초기화
        self.debug_publishers = {}
        self.prev_max_height = {}
        self.last_codes = {}

        for i, topic in enumerate(camera_topics):
            debug_topic = f'/obstacle_crosswalk_debug_{i}'
            self.debug_publishers[topic] = self.create_publisher(Image, debug_topic, 1)
            self.prev_max_height[topic] = 0.0
            self.last_codes[topic] = 1
            self.create_subscription(Image, topic, self.make_callback(topic), 1)
            self.get_logger().info(f'Subscribing to {topic}, publishing debug to {debug_topic}')

    def make_callback(self, topic_name):
        def callback(msg: Image):
            try:
                img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            except Exception as e:
                self.get_logger().error(f'Image conversion failed: {e}')
                return

            h_frame = img.shape[0]
            res = self.model(img)[0]
            boxes = res.boxes.xyxy
            cls_ids = res.boxes.cls

            # 디버깅: YOLO 감지 결과 출력
            xyxy = boxes.cpu().numpy() if hasattr(boxes, 'cpu') else boxes
            ids  = cls_ids.cpu().numpy() if hasattr(cls_ids, 'cpu') else cls_ids
            conf = res.boxes.conf.cpu().numpy() if hasattr(res.boxes.conf, 'cpu') else res.boxes.conf

            self.get_logger().info(f'[YOLO] from {topic_name}: {len(ids)} objects')
            for i, ((x1, y1, x2, y2), cid, c) in enumerate(zip(xyxy, ids, conf)):
                if i >= 5:
                    self.get_logger().info('... (more objects omitted)')
                    break
                label = names[int(cid)] if int(cid) < len(names) else 'unknown'
                self.get_logger().info(
                    f'  [{i}] {label:<10} | conf: {c:.2f} | box: ({int(x1)}, {int(y1)}, {int(x2)}, {int(y2)})'
                )

            # 최대 높이 계산
            max_h = 0.0
            for (x1, y1, x2, y2), cid in zip(xyxy, ids):
                label = names[int(cid)] if int(cid) < len(names) else None
                if label not in dynamic_labels:
                    continue
                box_h = float(y2 - y1)
                if box_h < height_thresholds.get(label, float('inf')):
                    continue
                max_h = max(max_h, box_h)

            prev_h = self.prev_max_height[topic_name]
            ratio = abs(max_h - prev_h) / prev_h if prev_h > EPS else 0.0
            sensor_code = 0 if prev_h > EPS and ratio >= MOVE_RATIO_THRESHOLD else 1
            if prev_h <= EPS and max_h < h_frame * FULL_FRAME_RATIO:
                sensor_code = 0

            self.prev_max_height[topic_name] = max_h
            self.last_codes[topic_name] = sensor_code

            final_code = 1 if all(c == 1 for c in self.last_codes.values()) else 0
            self.pub_info.publish(Int8(data=final_code))

            # 디버깅 이미지 생성
            debug_img = img.copy()
            for (x1, y1, x2, y2), cid in zip(xyxy, ids):
                label = names[int(cid)] if int(cid) < len(names) else 'unknown'
                if label not in dynamic_labels:
                    continue
                box_h = float(y2 - y1)
                if box_h < height_thresholds.get(label, float('inf')):
                    continue
                cv2.rectangle(debug_img, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                cv2.putText(debug_img, label, (int(x1), int(y1) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

            cv2.putText(debug_img, f"Obstacle Code: {final_code}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)

            debug_msg = self.bridge.cv2_to_imgmsg(debug_img, 'bgr8')
            debug_msg.header = msg.header
            self.debug_publishers[topic_name].publish(debug_msg)

        return callback

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
