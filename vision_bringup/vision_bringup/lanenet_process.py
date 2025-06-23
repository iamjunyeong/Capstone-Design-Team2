#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image, PointCloud2, PointField, CameraInfo
from cv_bridge import CvBridge
import message_filters
import sensor_msgs_py.point_cloud2 as pc2

import os
import sys
import cv2
import torch
import numpy as np
import struct
from PIL import Image as PilImage
from torchvision import transforms
from ament_index_python.packages import get_package_share_directory
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'lanenet_model'))
from lanenet.LaneNet import LaneNet  # ✅ 변경된 import

def rgb_to_float(r, g, b):
    return struct.unpack('f', struct.pack('I', (r << 16) | (g << 8) | b))[0]

class CenterLineContourNode(Node):
    def __init__(self):
        super().__init__('center_line_contour_node')

        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.model = LaneNet(arch='ENet').to(self.device).eval()
        model_path = os.path.join(
            get_package_share_directory('vision_bringup'),
            'model', 'best_model.pth'
        )
        self.model.load_state_dict(torch.load(model_path, map_location=self.device))
        self.get_logger().info('LaneNet 모델 로드 완료')

        self.transform = transforms.Compose([
            transforms.Resize((256, 512)),
            transforms.ToTensor(),
            transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])
        ])
        self.bridge = CvBridge()

        self.fx = self.fy = self.cx = self.cy = None
        self.camera_info_received = False

        self.create_subscription(
            CameraInfo,
            '/camera/camera/color/camera_info',
            self.camera_info_callback,
            qos_profile_sensor_data
        )

        color_sub = message_filters.Subscriber(
            self, Image, '/camera/camera/color/image_raw', qos_profile=qos_profile_sensor_data)
        depth_sub = message_filters.Subscriber(
            self, Image, '/camera/camera/aligned_depth_to_color/image_raw', qos_profile=qos_profile_sensor_data)

        self.ts = message_filters.ApproximateTimeSynchronizer([color_sub, depth_sub], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.callback)

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE,
                         depth=10)
        self.pub = self.create_publisher(PointCloud2, '/center_line_point', qos)
        self.overlay_pub = self.create_publisher(Image, '/lanenet/overlay_image', qos)

    def camera_info_callback(self, msg):
        if not self.camera_info_received:
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]
            self.camera_info_received = True
            self.get_logger().info(
                f'Camera 파라미터: fx={self.fx:.2f}, fy={self.fy:.2f}, cx={self.cx:.2f}, cy={self.cy:.2f}'
            )

    def callback(self, color_msg, depth_msg):
        if not self.camera_info_received:
            self.get_logger().warn('카메라 정보 대기 중...')
            return

        try:
            color_img = self.bridge.imgmsg_to_cv2(color_msg, 'bgr8')
            depth_img = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f'이미지 변환 실패: {e}')
            return

        input_tensor = self.preprocess(color_img)
        with torch.no_grad():
            out = self.model(input_tensor)['binary_seg_pred']

        mask = (out.squeeze().cpu().numpy().astype(np.uint8) * 255)
        mask = cv2.resize(mask, (color_img.shape[1], color_img.shape[0]))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) < 2:
            self.get_logger().warn('contour 2개 이하')
            return

        contours = sorted(contours, key=cv2.contourArea, reverse=True)[:2]
        contours = sorted(contours, key=lambda c: cv2.boundingRect(c)[0])

        points = []
        overlay_img = color_img.copy()

        for v in range(2, color_img.shape[0]-2, 10):
            x1 = x2 = -1
            for c in contours:
                for pt in c[:, 0, :]:
                    if abs(int(pt[1]) - v) < 5:
                        if x1 < 0:
                            x1 = int(pt[0])
                        else:
                            x2 = int(pt[0])
                            break
            if x1 >= 0 and x2 >= 0:
                u = (x1 + x2) / 2.0

                if 2 <= int(u) < depth_img.shape[1] - 2:
                    patch = depth_img[v-2:v+3, int(u)-2:int(u)+3]
                    valid = patch[patch > 0]
                    if len(valid) == 0:
                        continue
                    z = np.median(valid) * 0.001
                    if np.std(valid) * 0.001 > 0.02:
                        continue

                    x = (u - self.cx) * z / self.fx
                    y = (v - self.cy) * z / self.fy
                    rgb = rgb_to_float(255, 0, 0)  # 빨간색
                    points.append((x, y, z, rgb))

                if 0 <= v < overlay_img.shape[0] and 0 <= int(u) < overlay_img.shape[1]:
                    cv2.circle(overlay_img, (int(x1), v), 3, (0, 255, 0), -1)  # 왼쪽 초록
                    cv2.circle(overlay_img, (int(x2), v), 3, (0, 255, 0), -1)  # 오른쪽 초록
                    cv2.circle(overlay_img, (int(u), v), 3, (0, 0, 255), -1)  # 중앙 빨강

        if not points:
            self.get_logger().warn('3D point 없음')
            return

        header = depth_msg.header
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        cloud = pc2.create_cloud(header, fields, points)
        self.pub.publish(cloud)
        self.get_logger().info(f'{len(points)} 개의 중심선 포인트 퍼블리시 완료')

        overlay_msg = self.bridge.cv2_to_imgmsg(overlay_img, encoding='bgr8')
        overlay_msg.header = color_msg.header
        self.overlay_pub.publish(overlay_msg)

    def preprocess(self, cv_image):
        pil = PilImage.fromarray(cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB))
        return self.transform(pil).unsqueeze(0).to(self.device)

def main(args=None):
    rclpy.init(args=args)
    node = CenterLineContourNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
