#!/usr/bin/env python3
"""
segmented_cloud_node.py
YOLO-Seg → pixel masks → SLIC labels → sparse coloured cloud per class
Publishes:
  /obstacle_roadway          (red points)
  /obstacle_caution_zone     (yellow points)
  /obstacle_braille_block    (cyan points)
  /segmented/image_annotated (annotated image)
"""
import rclpy, time, struct, cv2, numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from std_msgs.msg import Header
from cv_bridge import CvBridge
from message_filters import Subscriber, ApproximateTimeSynchronizer
import sensor_msgs_py.point_cloud2 as pc2
from ultralytics import YOLO

# 클래스별 색상 (R,G,B)
CLASS_COLORS = {
    2: (0,   255, 255),  # braille → cyan
    3: (255, 255,   0),  # caution → yellow
    4: (255,   0,   0),  # roadway → red
}

def rgb_to_float(r, g, b):
    return struct.unpack('f', struct.pack('I', (r<<16)|(g<<8)|b))[0]

def build_sparse_class_cloud(depth_mm, K, labels,
                               class_mask, class_color,
                               min_pix=30, max_std=500, z_min=0.05):
    """YOLO 클래스 마스크에 걸친 슈퍼픽셀만 벡터화로 PCD 생성"""
    if not class_mask.any():
        return np.empty((0, 6), np.float32)

    H, W   = labels.shape
    lab_id = labels[class_mask]                    # 마스크 안 라벨 ID
    uniq, cnt = np.unique(lab_id, return_counts=True)
    keep = cnt >= min_pix
    if not keep.any():
        return np.empty((0, 6), np.float32)
    uniq = uniq[keep]

    # ① 깊이 통계
    z_flat = depth_mm.astype(np.float32).ravel()
    lbl_f  = labels.ravel()
    cnt_all = np.bincount(lbl_f, minlength=uniq.max()+1)
    cnt_all[cnt_all == 0] = 1
    sum_z   = np.bincount(lbl_f, weights=z_flat, minlength=uniq.max()+1)
    sum_z2  = np.bincount(lbl_f, weights=z_flat**2, minlength=uniq.max()+1)

    mean_z  = (sum_z   / cnt_all)[uniq] * 1e-3     # m 단위
    std_z   = np.sqrt(sum_z2 / cnt_all - (sum_z/cnt_all)**2)[uniq] * 1e-3
    ok      = (std_z < max_std*1e-3) & (mean_z > z_min)
    if not ok.any():
        return np.empty((0, 6), np.float32)
    uniq, mean_z = uniq[ok], mean_z[ok]

    # ② u,v 평균
    ys, xs = np.indices((H, W))
    mean_u = np.bincount(lbl_f, xs.ravel(), minlength=uniq.max()+1)[uniq] / cnt_all[uniq]
    mean_v = np.bincount(lbl_f, ys.ravel(), minlength=uniq.max()+1)[uniq] / cnt_all[uniq]

    fx, fy, cx, cy = K[0,0], K[1,1], K[0,2], K[1,2]
    X = (mean_u - cx) * mean_z / fx
    Y = (mean_v - cy) * mean_z / fy
    Z = mean_z

    r, g, b = class_color
    pts = np.column_stack([X, Y, Z,
                           np.full_like(X, r),
                           np.full_like(Y, g),
                           np.full_like(Z, b)]).astype(np.float32)
    return pts


def cloud_rgb_to_msg(cloud_rgb, header_src):
    if cloud_rgb.size == 0:
        return None
    xyz  = cloud_rgb[:, :3]
    rgbf = np.vectorize(rgb_to_float)(*(cloud_rgb[:, 3:].astype(np.uint8)[:, ::-1]).T)
    data = np.hstack((xyz, rgbf[:,None])).astype(np.float32)

    # keyword 인자로만 PointField 생성
    fields = [
        PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
        PointField(name='rgb', offset=12,datatype=PointField.FLOAT32, count=1),
    ]

    hdr = Header()
    hdr.stamp    = header_src.stamp
    hdr.frame_id = header_src.frame_id
    return pc2.create_cloud(hdr, fields, data.tolist())

class SegmentedCloudNode(Node):
    ID_BRAILLE = 2
    ID_CAUTION = 3
    ID_ROADWAY = 4

    def __init__(self):
        super().__init__('segmented_cloud_node')
        self.bridge = CvBridge()
        self.K = None

        self.model = YOLO('/home/ubuntu/capston_ws/src/Capstone-Design-Team2/vision_bringup/resource/best.pt')

        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=5)
        sub_c = Subscriber(self, Image     , '/slic/color'       , qos_profile=qos)
        sub_d = Subscriber(self, Image     , '/slic/depth'       , qos_profile=qos)
        sub_i = Subscriber(self, CameraInfo, '/slic/camera_info' , qos_profile=qos)
        sub_l = Subscriber(self, Image     , '/slic/labels'      , qos_profile=qos)

        ats = ApproximateTimeSynchronizer([sub_c, sub_d, sub_i, sub_l], 5, 0.03)
        ats.registerCallback(self.cb_synced)

        self.pub_road    = self.create_publisher(PointCloud2, '/obstacle_roadway',       10)
        self.pub_caution = self.create_publisher(PointCloud2, '/obstacle_caution_zone',  10)
        self.pub_braille = self.create_publisher(PointCloud2, '/obstacle_braille_block',10)
        self.pub_annot   = self.create_publisher(Image    , '/segmented/image_annotated',10)

        self.get_logger().info('SegmentedCloudNode ready.')

    def cb_synced(self, img_msg, depth_msg, info_msg, labels_msg):
        t0 = time.perf_counter()
        if self.K is None:
            self.K = np.asarray(info_msg.k, np.float32).reshape(3,3)
            self.get_logger().info('Camera intrinsics received.')

        color  = self.bridge.imgmsg_to_cv2(img_msg,    'passthrough')
        if img_msg.encoding.startswith('rgb'):
            color = cv2.cvtColor(color, cv2.COLOR_RGB2BGR)
        depth  = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough')
        labels = self.bridge.imgmsg_to_cv2(labels_msg,'passthrough')

        # 1) YOLO segmentation & annotated image
        res = self.model(color, verbose=False)[0]
        if res.masks is None:
            self.get_logger().warn('No YOLO masks.')
            return
        annotated = res.plot()
        ann_msg = self.bridge.cv2_to_imgmsg(annotated, 'bgr8')
        ann_msg.header = img_msg.header
        self.pub_annot.publish(ann_msg)

        # 2) class masks
        h, w = color.shape[:2]
        mask_b = np.zeros((h, w), bool)
        mask_c = np.zeros((h, w), bool)
        mask_r = np.zeros((h, w), bool)
        for m, cid in zip(res.masks.data.cpu().numpy()>0.5,
                          res.boxes.cls.int().cpu().tolist()):
            m_up = cv2.resize(m.astype(np.uint8), (w, h),
                              interpolation=cv2.INTER_NEAREST).astype(bool)
            if cid == self.ID_BRAILLE: mask_b |= m_up
            elif cid == self.ID_CAUTION: mask_c |= m_up
            elif cid == self.ID_ROADWAY: mask_r |= m_up

        # 3) sparse clouds using slic labels
        cloud_b = build_sparse_class_cloud(depth, self.K, labels, mask_b, CLASS_COLORS[self.ID_BRAILLE])
        cloud_c = build_sparse_class_cloud(depth, self.K, labels, mask_c, CLASS_COLORS[self.ID_CAUTION])
        cloud_r = build_sparse_class_cloud(depth, self.K, labels, mask_r, CLASS_COLORS[self.ID_ROADWAY])

        # 4) publish or clear
        for cloud, pub in ((cloud_b, self.pub_braille),
                           (cloud_c, self.pub_caution),
                           (cloud_r, self.pub_road)):
            msg = cloud_rgb_to_msg(cloud, img_msg.header)
            if msg is None:
                # 빈 클라우드 생성도 동일하게 keyword args 사용
                empty_fields = [
                    PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
                    PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
                    PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
                    PointField(name='rgb',offset=12, datatype=PointField.FLOAT32, count=1),
                ]
                empty = pc2.create_cloud(img_msg.header, empty_fields, [])
                pub.publish(empty)
            else:
                pub.publish(msg)

        self.get_logger().info(
            f'proc_time={time.perf_counter()-t0:.3f}s '
            f'braille={cloud_b.shape[0]} '
            f'caution={cloud_c.shape[0]} '
            f'road={cloud_r.shape[0]}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = SegmentedCloudNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
