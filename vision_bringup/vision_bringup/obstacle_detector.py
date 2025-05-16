import rclpy
import os
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int8
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory

# Class and threshold
names = ['pole','person','bollard','bicycle','motorcycle','bench','barricade','scooter']
thresholds = {
    'person':10000,'bicycle':2000,'motorcycle':2000,
    'scooter':800,'pole':1500,'bollard':1000,'bench':2500, 'barricade':1500
}
static_labels = {'pole','bollard','bench','barricade'}
dynamic_labels = {'person','bicycle','motorcycle','scooter'}
relevant = static_labels|dynamic_labels

def detect_obstacles(frame, model):
    res = model(frame)[0]
    near_s, near_d = 0, 0
    boxes, cls = res.boxes.xyxy, res.boxes.cls
    if boxes is not None and len(boxes):
        xyxy = boxes.cpu().numpy() if hasattr(boxes,'cpu') else boxes
        ids  = cls.cpu().numpy() if hasattr(cls,'cpu') else cls
        for (x1,y1,x2,y2), cid in zip(xyxy,ids):
            label = names[int(cid)] if cid < len(names) else None
            if label not in relevant: continue
            area = (x2-x1)*(y2-y1)
            if area < thresholds.get(label,1000): continue
            if label in dynamic_labels: near_d+=1
            else: near_s+=1
    if   near_s==0 and near_d==0: code=0
    elif near_s>0 and near_d==0: code=1
    elif near_s==0 and near_d>0: code=2
    else: code=3
    return code


class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector')
        self.declare_parameter('camera_topics', ['/camera/camera/color/image_raw','/usb_cam1','/usb_cam2'])
        topics = self.get_parameter('camera_topics').value

        # 모델 경로: 패키지 공유 디렉터리 내 model 폴더
        pkg_share = get_package_share_directory('vision_bringup')
        model_path = os.path.join(pkg_share, 'model', 'obstacle_detector.pt')
        self.model = YOLO(model_path)

        self.pub = self.create_publisher(Int8, '/obstacle_info', 1)
        self.debug_pub = self.create_publisher(Image, '/obstacle_debug/image_raw', 1)
        self.bridge = CvBridge()

        for t in topics:
            self.create_subscription(Image, t, self.cb, 1)
            self.get_logger().info(f'Sub to {t}')

    def cb(self, msg: Image):
        try:
            # ① 원본 그대로 받아온다
            img = self.bridge.imgmsg_to_cv2(msg, 'passthrough')

            # ② 필요하면 BGR 로 변환
            if msg.encoding == 'rgb8':
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            elif msg.encoding in ('mono8', '8UC1'):
                img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
            elif msg.encoding != 'bgr8':
                self.get_logger().warn(f'Unhandled encoding {msg.encoding}')
                return
            if img.size == 0:
                self.get_logger().warn('Empty image received, skip frame')
                return
        except Exception as e:
            self.get_logger().error(f'Image conversion failed: {e}')
            return

        # YOLO 결과 및 코드 판단
        res = self.model(img)[0]
        code = detect_obstacles(img, self.model)

        out = Int8()
        out.data = code
        self.pub.publish(out)

        # 디버깅 이미지 퍼블리시
        img_plot = res.plot()
        img_bgr = cv2.cvtColor(img_plot, cv2.COLOR_RGB2BGR)
        cv2.putText(
            img_bgr,
            f"Obstacle Code: {code}",
            (20, 40),                    # 좌표 (x, y)
            cv2.FONT_HERSHEY_SIMPLEX,   # 폰트
            1.2,                        # 크기
            (0, 0, 255),                # 색상 (빨강)
            2,                          # 두께
            cv2.LINE_AA
        )
        debug_msg = self.bridge.cv2_to_imgmsg(img_bgr, encoding='bgr8')
        debug_msg.header = msg.header
        self.debug_pub.publish(debug_msg)

def main():
    rclpy.init()
    node=ObstacleDetector(); rclpy.spin(node)
    node.destroy_node(); rclpy.shutdown()

if __name__=='__main__': main()
