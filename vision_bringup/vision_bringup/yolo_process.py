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

def build_sparse_class_cloud(color_bgr, depth_mm, K, labels, class_mask, class_id,
                             scale=1.0, min_pix=30, max_depth_jitter_mm=3000):
    """Generate Nx6 array [X,Y,Z,r,g,b] for each superpixel that overlaps class_mask."""
    if not class_mask.any():
        return np.zeros((0,6), np.float32)

    fx, fy, cx, cy = K[0,0], K[1,1], K[0,2], K[1,2]
    pts = []
    for lab in np.unique(labels):
        m = (labels == lab)
        if m.sum() < min_pix or (class_mask & m).sum() == 0:
            continue
        zs = depth_mm[m].astype(np.float32)
        if zs.std() > max_depth_jitter_mm:
            continue
        z = np.median(zs) * 1e-3
        if z <= 0.05:
            continue
        ys, xs = np.nonzero(m)
        u, v = xs.mean(), ys.mean()
        X = (u - cx) * z / fx
        Y = (v - cy) * z / fy
        r, g, b = CLASS_COLORS[class_id]
        pts.append([X, Y, z, r, g, b])
    return np.asarray(pts, np.float32)

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

        self.model = YOLO('../resource/best.pt')

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

        color  = self.bridge.imgmsg_to_cv2(img_msg,   'bgr8')
        depth  = self.bridge.imgmsg_to_cv2(depth_msg, '16UC1')
        labels = self.bridge.imgmsg_to_cv2(labels_msg,'32SC1')

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
        cloud_b = build_sparse_class_cloud(color, depth, self.K, labels, mask_b, self.ID_BRAILLE)
        cloud_c = build_sparse_class_cloud(color, depth, self.K, labels, mask_c, self.ID_CAUTION)
        cloud_r = build_sparse_class_cloud(color, depth, self.K, labels, mask_r, self.ID_ROADWAY)

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
