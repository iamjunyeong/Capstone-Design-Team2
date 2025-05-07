import rclpy
import os
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int8
from cv_bridge import CvBridge
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
        self.declare_parameter('camera_topics', ['/camera','/usb_cam1','/usb_cam2'])
        topics = self.get_parameter('camera_topics').value

        # 모델 경로: 패키지 공유 디렉터리 내 model 폴더
        pkg_share = get_package_share_directory('vision_bringup')
        model_path = os.path.join(pkg_share, 'model', 'obstacle_detector.pt')
        self.model = YOLO(model_path)

        self.pub = self.create_publisher(Int8, '/obstacle_info', 1)
        self.bridge = CvBridge()

        for t in topics:
            self.create_subscription(Image, t, self.cb, 1)
            self.get_logger().info(f'Sub to {t}')

    def cb(self, msg: Image):
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        code = detect_obstacles(img, self.model)
        out = Int8()
        out.data = code
        self.pub.publish(out)


def main():
    rclpy.init()
    node=ObstacleDetector(); rclpy.spin(node)
    node.destroy_node(); rclpy.shutdown()

if __name__=='__main__': main()
