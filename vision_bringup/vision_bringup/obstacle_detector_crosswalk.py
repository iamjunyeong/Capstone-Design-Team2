#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int8
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

DYNAMIC_CLASSES = {'car', 'truck', 'bicycle', 'motorcycle', 'bus', 'scooter'}
HEIGHT_THRESHOLDS = {
    'car':        30,
    'truck':      80,
    'bicycle':    50,
    'motorcycle': 50,
    'bus':        80,
    'scooter':    40,
}

#FULL_FRAME_RATIO = 0.95
MOVE_RATIO_THRESHOLD = 0.05
EPS = 1e-6

class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector_crosswalk')

        self.declare_parameter('camera_topics', ['/usb_cam_0/image_raw', '/usb_cam_1/image_raw'])
        camera_topics = self.get_parameter('camera_topics').value

        self.pub_info = self.create_publisher(Int8, '/obstacle_crosswalk_info', 1)

        self.bridge = CvBridge()
        self.model = YOLO('/home/ubuntu/capstone_ws/src/Capstone-Design-Team2/vision_bringup/model/obstacle_detector_crosswalk.pt')

        self.prev_max = {t: 0.0 for t in camera_topics}
        self.last_codes = {t: 1   for t in camera_topics}
        self.debug_publishers = {}

        for i, topic in enumerate(camera_topics):
            self.debug_publishers[topic] = self.create_publisher(Image, f'/obstacle_crosswalk_debug_{i}', 1)
            self.create_subscription(Image, topic, self.make_cb(topic), 1)
            self.get_logger().info(f'Subscribing to {topic}')

    def make_cb(self, topic_name):
        def callback(msg: Image):
            try:
                img_yuv422 = self.bridge.imgmsg_to_cv2(msg, 'passthrough')

                # 안전하게 채널 수 검사
                if img_yuv422.ndim == 3 and img_yuv422.shape[2] == 2:
                    img = cv2.cvtColor(img_yuv422, cv2.COLOR_YUV2BGR_YUY2)
                elif img_yuv422.ndim == 3 and img_yuv422.shape[2] == 3:
                    self.get_logger().warn('Image already 3 channels. Skipping YUV2BGR_YUY2 conversion.')
                    img = img_yuv422
                else:
                    self.get_logger().error(f'Unexpected image shape: {img_yuv422.shape}')
                    return

                # 이후 encoding 조건은 유지
                if msg.encoding == 'rgb8':
                    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                elif msg.encoding in ('mono8', '8UC1'):
                    img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

            except Exception as e:
                self.get_logger().error(f'Image conversion failed: {e}')
                return


            #h_frame = img.shape[0]
            print(f"image shape: {img.shape}, encoding: {msg.encoding}")

            res = self.model(img)[0]
            boxes = res.boxes.xyxy
            cls_ids = res.boxes.cls

            max_h = 0.0
            if boxes is not None and len(boxes) > 0:
                arr_xyxy = boxes.cpu().numpy() if hasattr(boxes, 'cpu') else boxes
                arr_cls  = cls_ids.cpu().numpy() if hasattr(cls_ids, 'cpu') else cls_ids
                for coord, cid in zip(arr_xyxy, arr_cls):
                    label = self.model.names[int(cid)]
                    if label not in DYNAMIC_CLASSES:
                        continue
                    box_h = float(coord[3] - coord[1])
                    if box_h < HEIGHT_THRESHOLDS.get(label, float('inf')):
                        continue
                    max_h = max(max_h, box_h)


            if boxes is None or len(boxes) == 0:
                code = 1
                current_h = 0.0
                self.prev_max[topic_name] = 0.0 
            else:
                prev_h = self.prev_max[topic_name]
                current_h = max_h

                if prev_h < EPS:
                    ratio = 0.0
                else:
                    ratio = (current_h - prev_h) / prev_h

                if ratio >= MOVE_RATIO_THRESHOLD:
                    code = 0
                else:
                    code = 1

            self.prev_max[topic_name] = current_h
            self.last_codes[topic_name] = code
            final_code = 1 if all(c == 1 for c in self.last_codes.values()) else 0
            self.pub_info.publish(Int8(data=final_code))

            dbg = img.copy()
            if boxes is not None and len(boxes) > 0:
                for coord, cid in zip(arr_xyxy, arr_cls):
                    label = self.model.names[int(cid)]
                    if label not in DYNAMIC_CLASSES:
                        continue
                    box_h = float(coord[3] - coord[1])
                    if box_h < HEIGHT_THRESHOLDS.get(label, float('inf')):
                        continue
                    x1, y1, x2, y2 = map(int, coord)
                    cv2.rectangle(dbg, (x1, y1), (x2, y2), (0,255,0), 2)
                    cv2.putText(dbg, label, (x1, y1-5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,0,0), 2)
            cv2.putText(dbg, f"Code: {final_code}", (10,30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0,0,255), 2)
            dbg_msg = self.bridge.cv2_to_imgmsg(dbg, 'bgr8')
            dbg_msg.header = msg.header
            self.debug_publishers[topic_name].publish(dbg_msg)

        return callback

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()