#!/usr/bin/env python3
import rclpy, cv2, numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from message_filters import Subscriber, ApproximateTimeSynchronizer
from fast_slic.avx2 import SlicAvx2 as Slic

def fastslic_bgr(bgr: np.ndarray, num_components=1600, compactness=10):
    """BGR 이미지를 fast-slic으로 슈퍼픽셀 라벨링"""
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    slic = Slic(num_components=num_components, compactness=compactness)
    return slic.iterate(rgb)


class SlicNode(Node):
    def __init__(self):
        super().__init__('slic_node')
        self.bridge = CvBridge()
        # 원하는 스케일 (이 값은 downstream 노드의 scale 파라미터와 일치시킵니다.)
        self.scale = 0.4

        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=5)
        # Raw camera topics
        sub_c = Subscriber(self, Image,      '/camera/camera/color/image_raw', qos_profile=qos)
        sub_d = Subscriber(self, Image,      '/camera/camera/aligned_depth_to_color/image_raw', qos_profile=qos)
        sub_i = Subscriber(self, CameraInfo, '/camera/camera/color/camera_info', qos_profile=qos)

        ats = ApproximateTimeSynchronizer([sub_c, sub_d, sub_i], queue_size=5, slop=0.03)
        ats.registerCallback(self.callback)

        # Scaled topics
        self.pub_labels = self.create_publisher(Image,      '/slic/labels',       10)
        self.pub_color  = self.create_publisher(Image,      '/slic/color',        10)
        self.pub_depth  = self.create_publisher(Image,      '/slic/depth',        10)
        self.pub_info   = self.create_publisher(CameraInfo, '/slic/camera_info', 10)

        self.get_logger().info('SlicNode initialized with scale={:.2f}'.format(self.scale))

    def callback(self, img_msg, depth_msg, info_msg):
        # Raw images to cv2
        color_raw = self.bridge.imgmsg_to_cv2(img_msg,   'bgr8')
        depth_raw = self.bridge.imgmsg_to_cv2(depth_msg, '16UC1')

        # 1) 스케일 적용
        h, w = color_raw.shape[:2]
        h_s, w_s = int(h * self.scale), int(w * self.scale)
        color = cv2.resize(color_raw, (w_s, h_s), interpolation=cv2.INTER_LINEAR)
        depth = cv2.resize(depth_raw, (w_s, h_s), interpolation=cv2.INTER_NEAREST)

        # 2) 카메라 내재 행렬 업데이트 및 발행
        K = np.asarray(info_msg.k, np.float32).reshape(3,3)
        Ks = K.copy()
        Ks[0,0] *= self.scale; Ks[1,1] *= self.scale
        Ks[0,2] *= self.scale; Ks[1,2] *= self.scale
        K3 = Ks.reshape(3, 3)[:3, :3]
        info_s = CameraInfo()
        info_s.header = info_msg.header
        info_s.height = h_s
        info_s.width  = w_s
        info_s.distortion_model = info_msg.distortion_model
        info_s.d = list(info_msg.d)
        info_s.r = list(info_msg.r)
        info_s.p = list(info_msg.p)
        info_s.k = [float(val) for val in K3.flatten()]
        self.pub_info.publish(info_s)

        # 3) SLIC 레이블링
        num_comp = int((h_s * w_s) / (16 ** 2))
        labels = fastslic_bgr(color, num_components=num_comp, compactness=10.0)

        # 4) 메시지 변환 및 발행
        label_msg = self.bridge.cv2_to_imgmsg(labels.astype(np.int32), encoding='32SC1')
        label_msg.header = img_msg.header
        color_msg = self.bridge.cv2_to_imgmsg(color, 'bgr8')
        color_msg.header = img_msg.header
        depth_msg_s = self.bridge.cv2_to_imgmsg(depth, '16UC1')
        depth_msg_s.header = depth_msg.header

        self.pub_labels.publish(label_msg)
        self.pub_color.publish(color_msg)
        self.pub_depth.publish(depth_msg_s)

        self.get_logger().debug(f'Published scaled SLIC ({w_s}x{h_s}) and camera_info at {img_msg.header.stamp}')


def main(args=None):
    rclpy.init(args=args)
    node = SlicNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
