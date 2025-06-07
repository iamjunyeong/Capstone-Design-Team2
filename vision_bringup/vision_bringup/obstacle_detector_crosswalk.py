# detect_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int8
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO


names = ['car', 'truck', 'bicycle', 'motorcycle', 'bus', 'scooter']
height_thresholds = {
    'car':       60,
    'truck':     80,
    'bicycle':   50,
    'motorcycle':50,
    'bus':       80,
    'scooter':   40,
}

dynamic_labels = set(names)


FULL_FRAME_RATIO = 0.95
MOVE_RATIO_THRESHOLD = 0.10
EPS = 1e-6


class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector_crosswalk')

        self.declare_parameter('camera_topics', ['/usb_cam0', '/usb_cam1'])
        camera_topics = self.get_parameter('camera_topics').value

        self.pub = self.create_publisher(Int8, '/obstacle_info', 1)

        self.bridge = CvBridge()
        model_path = '/home/loe/workspace/github/Capstone-Design-Team2/vision_bringup/model/obstacle_detector_crosswalk.pt'
        self.model = YOLO(model_path)

        self.prev_max_height = {topic: 0.0 for topic in camera_topics}
        self.last_codes       = {topic: 1   for topic in camera_topics}

        for topic in camera_topics:
            self.create_subscription(Image, topic, self.make_callback(topic), 1)
            self.get_logger().info(f'Subscribing to {topic}')

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

            # 프레임 동적 클래스 최대 높이 계산
            max_h = 0.0
            if boxes is not None and len(boxes) > 0:
                xyxy = boxes.cpu().numpy() if hasattr(boxes, 'cpu') else boxes
                ids  = cls_ids.cpu().numpy() if hasattr(cls_ids, 'cpu') else cls_ids
                for (x1, y1, x2, y2), cid in zip(xyxy, ids):
                    label = names[int(cid)] if int(cid) < len(names) else None
                    if label not in dynamic_labels:
                        continue
                    box_h = float(y2 - y1)
                    if box_h < height_thresholds.get(label, float('inf')):
                        continue
                    max_h = max(max_h, box_h)

            prev_h = self.prev_max_height[topic_name]
            if prev_h > EPS:
                diff  = abs(max_h - prev_h)
                ratio = diff / prev_h
            else:
                ratio = 0.0

            if prev_h > EPS:
                sensor_code = 0 if ratio >= MOVE_RATIO_THRESHOLD else 1
            else:
                sensor_code = 1 if max_h >= h_frame * FULL_FRAME_RATIO else 1

            self.prev_max_height[topic_name] = max_h
            self.last_codes[topic_name]       = sensor_code

            final_code = 1 if all(c == 1 for c in self.last_codes.values()) else 0

            out = Int8()
            out.data = final_code
            self.pub.publish(out)

        return callback

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

