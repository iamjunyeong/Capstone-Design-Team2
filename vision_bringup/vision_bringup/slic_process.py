#!/usr/bin/env python3
import rclpy, cv2, numpy as np, time
from rclpy.node import Node
from rclpy.qos  import QoSProfile, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge  import CvBridge
from message_filters import Subscriber, ApproximateTimeSynchronizer
from cuda_slic  import slic as gpu_slic

# ───────── GPU‑SLIC helper ──────────────────────────────────────────────
def gpuslic_bgr(bgr, n_segments=1000, compactness=10, logger=None):
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
    rgb = np.ascontiguousarray(rgb)
    t0  = time.perf_counter()
    labels = gpu_slic(
        rgb,
        n_segments=n_segments,
        compactness=compactness,
        multichannel=True,
        max_iter=10,
        enforce_connectivity=True,
        convert2lab=True,           # ★ 반드시 True
    ).astype(np.int32)
    if logger:
        logger.debug(f'[SLIC] {time.perf_counter()-t0:.3f}s '
                     f'unique={len(np.unique(labels))}')
    return labels
# ────────────────────────────────────────────────────────────────────────


class SlicNode(Node):
    def __init__(self):
        super().__init__('slic_node')
        self.bridge = CvBridge()
        self.scale  = 0.40                               

        qos   = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=5)
        sub_c = Subscriber(self, Image     , '/camera/camera/color/image_raw',      qos_profile=qos)
        sub_d = Subscriber(self, Image     , '/camera/camera/aligned_depth_to_color/image_raw', qos_profile=qos)
        sub_i = Subscriber(self, CameraInfo, '/camera/camera/color/camera_info',     qos_profile=qos)

        ats = ApproximateTimeSynchronizer([sub_c, sub_d, sub_i], 5, 0.05)
        ats.registerCallback(self.callback)

        self.pub_labels = self.create_publisher(Image,      '/slic/labels',      10)
        self.pub_color  = self.create_publisher(Image,      '/slic/color',       10)
        self.pub_depth  = self.create_publisher(Image,      '/slic/depth',       10)
        self.pub_info   = self.create_publisher(CameraInfo, '/slic/camera_info', 10)
        self.pub_overlay= self.create_publisher(Image,      '/slic/overlay',     10)

        self.get_logger().info(f'SlicNode ready — scale={self.scale}')

    # ───────────────────────────────────────────────────────────────────
    def callback(self, img_msg, depth_msg, info_msg):
        t0 = time.perf_counter()

        # 1. 디코딩
        color_raw = self.bridge.imgmsg_to_cv2(img_msg, 'passthrough')
        if img_msg.encoding == 'rgb8':
            color_raw = cv2.cvtColor(color_raw, cv2.COLOR_RGB2BGR)
        depth_raw = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough')

        # 2. 축소
        H, W = color_raw.shape[:2]
        w_s, h_s = int(W*self.scale), int(H*self.scale)
        color_s  = cv2.resize(color_raw, (w_s, h_s), cv2.INTER_LINEAR)
        depth_s  = cv2.resize(depth_raw, (w_s, h_s), cv2.INTER_NEAREST)

        # 3. GPU‑SLIc
        n_seg = max(400, int((h_s*w_s)/(8**2)))
        labels = gpuslic_bgr(color_s, n_segments=n_seg, compactness=5, logger=self.get_logger()) 

        # 4. 카메라 내부 행렬 스케일
        K  = np.asarray(info_msg.k, np.float32).reshape(3, 3)
        Ks = K.copy()
        Ks[0,0] *= self.scale; Ks[1,1] *= self.scale
        Ks[0,2] *= self.scale; Ks[1,2] *= self.scale

        info_s = CameraInfo()
        info_s.header          = info_msg.header
        info_s.height, info_s.width = h_s, w_s
        info_s.distortion_model = info_msg.distortion_model
        info_s.d, info_s.r, info_s.p = list(info_msg.d), list(info_msg.r), list(info_msg.p)
        info_s.k = [float(v) for v in Ks.flatten()]
        self.pub_info.publish(info_s)

        # 5. 이미지 메시지 변환
        label_msg   = self.bridge.cv2_to_imgmsg(labels, '32SC1')
        color_msg   = self.bridge.cv2_to_imgmsg(color_s, 'bgr8')
        depth_msg_s = self.bridge.cv2_to_imgmsg(depth_s,
                        depth_s.dtype==np.uint16 and '16UC1' or '32FC1')

        # 6. 슈퍼픽셀 경계 오버레이
        edge_src = (labels % 256).astype(np.uint8)   # ← 수정
        edges = cv2.Canny(edge_src, 1, 1)
        overlay = color_s.copy()
        overlay[edges > 0] = (0, 0, 255)
        overlay_msg = self.bridge.cv2_to_imgmsg(overlay, 'bgr8')


        # 7. 공통 헤더
        for m in (label_msg, color_msg, depth_msg_s, overlay_msg):
            m.header.stamp    = img_msg.header.stamp
            m.header.frame_id = img_msg.header.frame_id

        # 8. 퍼블리시
        self.pub_labels.publish(label_msg)
        self.pub_color .publish(color_msg)
        self.pub_depth .publish(depth_msg_s)
        self.pub_overlay.publish(overlay_msg)

        # 9. 로그
        self.get_logger().info(
            f'CB latency={time.perf_counter()-t0:.3f}s '
            f'SLIC uniq={len(np.unique(labels))}'
        )
    # ───────────────────────────────────────────────────────────────────

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
