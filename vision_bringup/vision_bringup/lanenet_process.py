#!/usr/bin/env python3
import os
import sys

# ----------------------------------------
# 1) 프로젝트 루트 경로를 PYTHONPATH에 추가
#    └ vision_bringup 패키지 상위 디렉토리 (lanenet_model, model, resource 등 포함)
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if BASE_DIR not in sys.path:
    sys.path.append(BASE_DIR)
# ----------------------------------------

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point

import torch
import cv2
from cv_bridge import CvBridge
import numpy as np
from PIL import Image as PilImage
from torchvision import transforms

# 프로젝트 내에 정의된 LaneNet 클래스 및 기타 유틸을 불러옵니다.
from lanenet_model.lanenet.LaneNet import LaneNet
from lanenet_model.utils.cli_helper_test import parse_args  # 필요에 따라 CLI 파라미터 사용 가능

# CUDA 장치 설정
DEVICE = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")

# 이미지 전처리를 위한 transform 정의 (모델 입력 크기에 맞게 Resize, ToTensor, Normalize)
data_transform = transforms.Compose([
    transforms.Resize((256, 512)),  # 모델 입력 크기 (height, width)
    transforms.ToTensor(),
    transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])
])

def load_model(model_path, model_type):
    """
    주어진 모델 경로와 모델 타입에 따라 LaneNet 모델을 생성하고,
    state_dict를 로드한 후, 평가 모드로 전환합니다.
    """
    model = LaneNet(arch=model_type)
    state_dict = torch.load(model_path, map_location=DEVICE)
    model.load_state_dict(state_dict)
    model.eval()
    model.to(DEVICE)
    return model

class BrailleBlockDetector(Node):
    def __init__(self, model, device):
        super().__init__('braille_block_detector')

        # 이미지 토픽 구독 (/camera/camera/color/image_raw)
        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.listener_callback,
            10)
        
        # 중앙 좌표 퍼블리시를 위한 Publisher (Point 메시지)
        self.publisher_ = self.create_publisher(Point, '/braille_block/center_point', 10)

        # ROS 이미지 메시지를 OpenCV 이미지로 변환하기 위한 CvBridge
        self.bridge = CvBridge()

        self.device = device
        self.model = model

        self.get_logger().info('LaneNet 모델이 성공적으로 로드되었습니다.')

    def listener_callback(self, msg):
        # ROS 이미지 메시지를 OpenCV 이미지로 변환
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'이미지 변환 실패: {e}')
            return

        # 전처리: OpenCV 이미지를 PIL 이미지로 변환 후, 데이터 transform 적용
        input_tensor = self.preprocess(cv_image)

        # 모델 추론
        with torch.no_grad():
            outputs = self.model(input_tensor)

        # 후처리: binary_seg_pred를 이용해 컨투어 검출 후, 양쪽 경계의 중앙 좌표 산출
        center_x, center_y = self.postprocess(outputs['binary_seg_pred'], cv_image.shape)

        # 중앙 좌표를 Point 메시지로 만들어 퍼블리시
        point_msg = Point()
        point_msg.x = center_x
        point_msg.y = center_y
        point_msg.z = 0.0  # 2D 평면이므로 z=0

        self.publisher_.publish(point_msg)
        self.get_logger().info(f'중앙 좌표 퍼블리시: ({center_x:.1f}, {center_y:.1f})')

    def preprocess(self, cv_image):
        """
        OpenCV 이미지(BGR)를 PIL 이미지(RGB)로 변환하고,
        데이터 전처리를 수행하여 모델에 넣을 수 있는 Tensor를 반환합니다.
        """
        pil_image = PilImage.fromarray(cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB))
        image_transformed = data_transform(pil_image).to(self.device)
        # 배치 차원 추가 (1, C, H, W)
        return image_transformed.unsqueeze(0)

    def postprocess(self, binary_seg_pred, original_shape):
        """
        binary_seg_pred: 모델에서 반환한 binary segmentation 예측 (Tensor)
        original_shape: 원본 이미지 크기 (H, W, C)
        """
        # Tensor → NumPy, thresholding → 이진 마스크
        binary_pred_np = binary_seg_pred.squeeze().cpu().numpy()
        _, binary_mask = cv2.threshold(binary_pred_np, 0.5, 255, cv2.THRESH_BINARY)
        binary_mask = binary_mask.astype(np.uint8)

        # 원본 크기로 리사이즈
        h, w = original_shape[0], original_shape[1]
        binary_mask = cv2.resize(binary_mask, (w, h))

        # 컨투어 검출
        contours, _ = cv2.findContours(binary_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # 큰 두 컨투어의 중심 계산
        if len(contours) >= 2:
            # 면적 기준 내림차순 정렬 후 상위 2개 선택
            contours = sorted(contours, key=cv2.contourArea, reverse=True)[:2]
            centers = []
            for cnt in contours:
                M = cv2.moments(cnt)
                if M['m00'] != 0:
                    cx = M['m10'] / M['m00']
                    cy = M['m01'] / M['m00']
                    centers.append((cx, cy))
            # 좌우 순서대로 정렬 후 중점 계산
            centers = sorted(centers, key=lambda pt: pt[0])
            center_x = (centers[0][0] + centers[1][0]) / 2.0
            center_y = (centers[0][1] + centers[1][1]) / 2.0
            return center_x, center_y

        # 컨투어가 부족하면 -1 반환
        return -1.0, -1.0

def main(args=None):
    rclpy.init(args=args)

    # ▶ CLI 파라미터 사용 예시
    # cli_args = parse_args()
    # model_path = cli_args.model
    # model_type = cli_args.model_type

    # 하드코딩 대신, 상대 경로로 모델 로드
    model_path = os.path.join(BASE_DIR, 'model', 'best_model.pth')
    model_type = 'default'  # 필요에 따라 실제 아키텍처 이름으로 변경

    model = load_model(model_path, model_type)
    node = BrailleBlockDetector(model, DEVICE)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
