#!/usr/bin/env python3
"""
pcd_processs.py
RGB-D ⇒ sparse point cloud (fast-SLIC) ⇒ RANSAC ⇒ DBSCAN
Publishes coloured clusters on /obstacle_clusters_colored
"""
import rclpy, time, struct, cv2, open3d as o3d, numpy as np
import sensor_msgs_py.point_cloud2 as pc2
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy
from sensor_msgs.msg import Image, PointCloud2, PointField, CameraInfo
from std_msgs.msg import Header
from cv_bridge import CvBridge
from sklearn.cluster import DBSCAN
from message_filters import Subscriber, ApproximateTimeSynchronizer


def fastslic_bgr(bgr: np.ndarray, num_components=1600, compactness=10):
    """BGR 이미지를 fast-slic으로 슈퍼픽셀 라벨링"""
    image_rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    slic = Slic(num_components=num_components, compactness=compactness)
    return slic.iterate(image_rgb)


def rgb_to_float(r, g, b):
    return struct.unpack('f', struct.pack('I', (r << 16) | (g << 8) | b))[0]

def build_sparse_cloud(color, depth, K, labels,
                       min_pix=30, max_std=300, z_min=0.05):
    H, W = labels.shape
    Npix = H * W
    lab_flat = labels.ravel()

    # 1. 픽셀 수
    counts = np.bincount(lab_flat)
    valid  = counts >= min_pix
    if not valid.any():
        return np.empty((0, 6), np.float32)

    # 2. 깊이: median & std
    z = depth.astype(np.float32).ravel()
    # sort‑by‑labels trick
    order  = lab_flat.argsort()
    z_sort = z[order]
    lab_sort = lab_flat[order]
    split = np.cumsum(counts[valid])[:-1]
    groups = np.split(z_sort[valid[lab_sort]], split)

    med = np.array([np.median(g) for g in groups])
    std = np.array([g.std()     for g in groups])
    ok  = std < max_std
    med = med[ok] * 1e-3
    labs = np.nonzero(valid)[0][ok]

    # 3. 좌표계
    ys, xs = np.indices((H, W))
    u = np.bincount(lab_flat, xs.ravel()) / counts
    v = np.bincount(lab_flat, ys.ravel()) / counts
    u, v = u[labs], v[labs]

    fx, fy, cx, cy = K[0,0], K[1,1], K[0,2], K[1,2]
    X = (u - cx) * med / fx
    Y = (v - cy) * med / fy
    Z = med

    # 4. 컬러
    r = np.bincount(lab_flat, color[:,:,2].ravel())[labs] / counts[labs]
    g = np.bincount(lab_flat, color[:,:,1].ravel())[labs] / counts[labs]
    b = np.bincount(lab_flat, color[:,:,0].ravel())[labs] / counts[labs]

    pts = np.column_stack([X, Y, Z, r, g, b]).astype(np.float32)
    return pts[Z > z_min]



def process_cloud(cloud_rgb: np.ndarray,
                  voxel=0.01, ransac_dist=0.2,
                  eps=1.0, min_samples=10, top_k=15):
    # (원본과 동일)
    if cloud_rgb.size == 0:
        return None, [], []

    xyz_in, rgb_in = cloud_rgb[:, :3], cloud_rgb[:, 3:] / 255.0
    pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(xyz_in))
    pcd.colors = o3d.utility.Vector3dVector(rgb_in)
    pcd = pcd.voxel_down_sample(voxel)

    keep = np.ones(len(pcd.points), bool)
    ransac_info = []
    tmp = pcd
    for _ in range(1):
        if len(tmp.points) < 100:
            break
        plane_eq, inliers = tmp.segment_plane(ransac_dist, 3, 300)
        if len(inliers) < 50:
            break
        ransac_info.append({'eq': plane_eq, 'inliers': len(inliers)})
        keep[inliers] = False
        tmp = tmp.select_by_index(inliers, invert=True)

    xyz = np.asarray(pcd.points)[keep]
    if xyz.size == 0:
        return None, ransac_info, []

    labels = DBSCAN(eps=eps, min_samples=min_samples).fit_predict(xyz)
    mask   = labels >= 0
    xyz    = xyz[mask]
    labels = labels[mask]
    if xyz.size == 0:
        return None, ransac_info, []

    uniq, cnts = np.unique(labels, return_counts=True)
    order = uniq[np.argsort(cnts)[::-1]][:top_k]
    sizes = dict(zip(uniq, cnts))
    db_info = [{'label': int(l), 'size': int(sizes[l])} for l in order]

    if not db_info:
        return None, ransac_info, db_info

    d     = np.linalg.norm(xyz, axis=1)
    d_min = d.min();  d_max = d.max() + 1e-6
    rgb   = np.zeros_like(xyz)
    for lab in order:
        m     = labels == lab
        alpha = (d[m] - d_min) / (d_max - d_min)
        rgb[m, 0] = alpha
        rgb[m, 2] = 1 - alpha

    pcd_out = o3d.geometry.PointCloud()
    pcd_out.points = o3d.utility.Vector3dVector(xyz)
    pcd_out.colors = o3d.utility.Vector3dVector(rgb)

    return pcd_out, ransac_info, db_info


def pcd_to_cloud_msg(pcd: o3d.geometry.PointCloud,
                     header_src: Header) -> PointCloud2:
    xyz  = np.asarray(pcd.points)
    rgbf = np.vectorize(rgb_to_float)(
             *(np.asarray(pcd.colors)[:, ::-1] * 255).astype(np.uint8).T)
    data = np.hstack([xyz, rgbf[:, None]]).astype(np.float32)

    header = Header()
    header.stamp    = header_src.stamp
    header.frame_id = header_src.frame_id

    fields = [
        PointField(name='x',   offset=0,  datatype=PointField.FLOAT32, count=1),
        PointField(name='y',   offset=4,  datatype=PointField.FLOAT32, count=1),
        PointField(name='z',   offset=8,  datatype=PointField.FLOAT32, count=1),
        PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    return pc2.create_cloud(header, fields, data.tolist())


class SparseObstacleNode(Node):
    def __init__(self):
        super().__init__('sparse_obstacle_node')
        self.bridge = CvBridge()
        self.K = None

        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=5)
        sub_c = Subscriber(self, Image     , '/slic/color'       , qos_profile=qos)
        sub_d = Subscriber(self, Image     , '/slic/depth'       , qos_profile=qos)
        sub_i = Subscriber(self, CameraInfo, '/slic/camera_info' , qos_profile=qos)
        sub_l = Subscriber(self, Image     , '/slic/labels'      , qos_profile=qos)

        ats = ApproximateTimeSynchronizer([sub_c, sub_d, sub_i, sub_l], 5, 0.03)
        ats.registerCallback(self.synced_cb)

        self.pub = self.create_publisher(PointCloud2, '/obstacle_clusters_colored', 10)
        self.get_logger().info('SparseObstacleNode ready.')

    def synced_cb(self, img_msg, depth_msg, info_msg, labels_msg):
        t0 = time.perf_counter()
        if self.K is None:
            self.K = np.asarray(info_msg.k, np.float32).reshape(3,3)
            self.get_logger().info('Intrinsic K initialised.')

        color  = self.bridge.imgmsg_to_cv2(img_msg,    'passthrough')
        if img_msg.encoding.startswith('rgb'):
            color = cv2.cvtColor(color, cv2.COLOR_RGB2BGR)
        depth  = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough')
        labels = self.bridge.imgmsg_to_cv2(labels_msg,'passthrough')

        cloud6 = build_sparse_cloud(color, depth, self.K, labels)
        self.get_logger().info(f'SLIC→cloud: {cloud6.shape[0]} pts ({time.perf_counter()-t0:.3f}s)')

        t1 = time.perf_counter()
        pcd, rinfo, dinfo = process_cloud(cloud6)
        self._log_ransac(rinfo)
        self._log_dbscan(dinfo)
        self.get_logger().info(f'Processing time: {time.perf_counter()-t1:.3f}s')

        if pcd is None:
            self.get_logger().info('No PCD to publish. Clearing in RViZ.')

            # PointField 인자 없이 생성 후 속성 직접 할당
            empty_fields = []
            for name, offset in [('x',0), ('y',4), ('z',8), ('rgb',12)]:
                pf = PointField()
                pf.name     = name
                pf.offset   = offset
                pf.datatype = PointField.FLOAT32
                pf.count    = 1
                empty_fields.append(pf)

            empty = pc2.create_cloud(img_msg.header, empty_fields, [])
            self.pub.publish(empty)
            return

        self.pub.publish(pcd_to_cloud_msg(pcd, img_msg.header))
        self.get_logger().info(f'Published PCD ({time.perf_counter()-t0:.3f}s total)')

    def _log_ransac(self, rinfo):
        if not rinfo:
            self.get_logger().info('RANSAC: no plane removed')
            return
        for k, pl in enumerate(rinfo):
            a,b,c,d = pl["eq"]
            self.get_logger().info(
                f'RANSAC[{k}] inliers={pl["inliers"]:4d}  plane:{a:+.3f}x {b:+.3f}y {c:+.3f}z {d:+.3f}=0'
            )

    def _log_dbscan(self, dinfo):
        if not dinfo:
            self.get_logger().info('DBSCAN: no clusters')
            return
        summary = ', '.join([f'ℓ={d["label"]}({d["size"]})' for d in dinfo])
        self.get_logger().info(f'DBSCAN clusters: {summary}')


def main(args=None):
    rclpy.init(args=args)
    node = SparseObstacleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
