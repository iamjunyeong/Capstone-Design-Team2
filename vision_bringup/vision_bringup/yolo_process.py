#!/usr/bin/env python3
"""
segmented_cloud_node.py
YOLO‑Seg → pixel masks (roadway / caution_zone / braille_guide_block)
        → SLIC super‑pixels → sparse coloured cloud
Publishes:
  /obstacle_roadway          (red points)
  /obstacle_caution_zone     (yellow points)
  /obstacle_braille_block    (cyan points)
  /segmented/image_annotated (RGB image with YOLO overlays)
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

# ─── assign fixed RGB colours per class ─────────────────────
# ORDER is (R, G, B)
CLASS_COLORS = {
    2: (0,   255, 255),   # braille_guide_block → cyan
    3: (255, 255,   0),   # caution_zone        → yellow
    4: (255,   0,   0),   # roadway             → red
}

def rgb_to_float(r, g, b):
    return struct.unpack('f', struct.pack('I', (r<<16)|(g<<8)|b))[0]

def slic_opencv_bgr(bgr, size=12, ruler=8.0, iters=5):
    if not hasattr(cv2, 'ximgproc'):
        raise RuntimeError('opencv‑contrib‑python is required')
    slicer = cv2.ximgproc.createSuperpixelSLIC(
        bgr, algorithm=cv2.ximgproc.SLICO,
        region_size=size, ruler=ruler)
    slicer.iterate(iters)
    return slicer.getLabels()

def build_sparse_class_cloud(color_bgr, depth_mm, K, class_mask, class_id,
                             scale=0.3, slic_size=12, slic_ruler=7.0,
                             min_pix=30, max_depth_jitter_mm=3000):
    if not class_mask.any():
        return np.zeros((0,6), np.float32)

    if scale != 1.0:
        color_bgr  = cv2.resize(color_bgr,  None, fx=scale, fy=scale, interpolation=cv2.INTER_LINEAR)
        depth_mm   = cv2.resize(depth_mm,   None, fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)
        class_mask = cv2.resize(class_mask.astype(np.uint8), None, fx=scale, fy=scale,
                                interpolation=cv2.INTER_NEAREST).astype(bool)
        fx, fy, cx, cy = (K[0,0]*scale, K[1,1]*scale, K[0,2]*scale, K[1,2]*scale)
    else:
        fx, fy, cx, cy = K[0,0], K[1,1], K[0,2], K[1,2]

    labels = slic_opencv_bgr(color_bgr, slic_size, slic_ruler)
    pts = []
    for lab in np.unique(labels):
        m = labels == lab
        if m.sum() < min_pix: continue
        if class_mask[m].mean() < 0.6: continue
        zs = depth_mm[m].astype(np.float32)
        if zs.std() > max_depth_jitter_mm: continue
        z = np.median(zs) * 1e-3
        if z <= 0.05: continue
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
    fields = [
        PointField(name='x',   offset=0,  datatype=PointField.FLOAT32, count=1),
        PointField(name='y',   offset=4,  datatype=PointField.FLOAT32, count=1),
        PointField(name='z',   offset=8,  datatype=PointField.FLOAT32, count=1),
        PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
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
        self.slic_labels = None

        self.model = YOLO(
            '/home/loe/workspace/yolo/surface/ultralytics/train3/weights/best.pt'
        )

        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=5)
        c = Subscriber(self, Image,      '/camera/camera/color/image_raw', qos_profile=qos)
        d = Subscriber(self, Image,      '/camera/camera/aligned_depth_to_color/image_raw', qos_profile=qos)
        i = Subscriber(self, CameraInfo, '/camera/camera/color/camera_info', qos_profile=qos)
        l = Subscriber(self, Image, '/slic_labels', qos_profile=qos)


        sync = ApproximateTimeSynchronizer([c,d,i], queue_size=5, slop=0.03)
        sync.registerCallback(self.cb_synced)

        self.pub_road      = self.create_publisher(PointCloud2, '/obstacle_roadway',       10)
        self.pub_caution   = self.create_publisher(PointCloud2, '/obstacle_caution_zone',  10)
        self.pub_braille   = self.create_publisher(PointCloud2, '/obstacle_braille_block', 10)
        self.pub_annotated = self.create_publisher(Image,       '/segmented/image_annotated', 10)

        self.get_logger().info('SegmentedCloudNode ready.')

    def make_empty_cloud(self, header_src):
        fields = [
            PointField(name='x',   offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y',   offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z',   offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        hdr = Header()
        hdr.stamp = header_src.stamp
        hdr.frame_id = header_src.frame_id
        return pc2.create_cloud(hdr, fields, [])  # 빈 포인트 리스트


    def cb_synced(self, img_msg, depth_msg, info_msg):
        t0 = time.perf_counter()
        if self.K is None:
            self.K = np.asarray(info_msg.k, np.float32).reshape(3,3)
            self.get_logger().info('Camera intrinsics received.')

        color = self.bridge.imgmsg_to_cv2(img_msg,   'bgr8')
        depth = self.bridge.imgmsg_to_cv2(depth_msg, '16UC1')

        res = self.model(color, verbose=False)[0]
        if res.masks is None:
            self.get_logger().warn('No YOLO masks.')
            return

        # Annotated image
        annotated = res.plot()
        ann_msg   = self.bridge.cv2_to_imgmsg(annotated, 'bgr8')
        ann_msg.header = img_msg.header
        self.pub_annotated.publish(ann_msg)

        cls_ids = res.boxes.cls.int().cpu().tolist()
        masks   = (res.masks.data.cpu().numpy() > 0.5)
        h, w = color.shape[:2]
        mask_b = np.zeros((h, w), bool)
        mask_c = np.zeros((h, w), bool)
        mask_r = np.zeros((h, w), bool)

        for m, cid in zip(masks, cls_ids):
            m_up = cv2.resize(m.astype(np.uint8), (w,h),
                              interpolation=cv2.INTER_NEAREST).astype(bool)
            if cid == self.ID_BRAILLE:
                mask_b |= m_up
            elif cid == self.ID_CAUTION:
                mask_c |= m_up
            elif cid == self.ID_ROADWAY:
                mask_r |= m_up

            cloud_b = build_sparse_class_cloud(color, depth, self.K, mask_b, self.ID_BRAILLE)
            cloud_c = build_sparse_class_cloud(color, depth, self.K, mask_c, self.ID_CAUTION)
            cloud_r = build_sparse_class_cloud(color, depth, self.K, mask_r, self.ID_ROADWAY)

            msg_b = cloud_rgb_to_msg(cloud_b, img_msg.header)
            if msg_b is None:
                msg_b = self.make_empty_cloud(img_msg.header)
            self.pub_braille.publish(msg_b)

            msg_c = cloud_rgb_to_msg(cloud_c, img_msg.header)
            if msg_c is None:
                msg_c = self.make_empty_cloud(img_msg.header)
            self.pub_caution.publish(msg_c)

            msg_r = cloud_rgb_to_msg(cloud_r, img_msg.header)
            if msg_r is None:
                msg_r = self.make_empty_cloud(img_msg.header)
            self.pub_road.publish(msg_r)


        self.get_logger().info(
            f'proc_time={time.perf_counter()-t0:.3f}s '
            f'pts: braille={cloud_b.shape[0]}, caution={cloud_c.shape[0]}, road={cloud_r.shape[0]}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = SegmentedCloudNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
