#!/usr/bin/env python3
"""
pcd_processs.py
RGB‑D ⇒ sparse point cloud (OpenCV‑SLIC) ⇒ RANSAC ⇒ DBSCAN
Publishes coloured clusters on /obstacle_clusters_colored
"""
from hdbscan import HDBSCAN
import rclpy, time, struct, cv2, open3d as o3d, numpy as np
import sensor_msgs_py.point_cloud2 as pc2
from rclpy.node import Node
from rclpy.qos  import QoSProfile, HistoryPolicy
from sensor_msgs.msg import Image, PointCloud2, PointField, CameraInfo
from std_msgs.msg    import Header
from cv_bridge       import CvBridge
from sklearn.cluster import DBSCAN
from message_filters import Subscriber, ApproximateTimeSynchronizer
from fast_slic.avx2 import SlicAvx2 as Slic
from cv_bridge import CvBridge




def fastslic_bgr(bgr: np.ndarray, num_components=1600, compactness=10):
    """BGR 이미지를 fast-slic으로 슈퍼픽셀 라벨링"""
    image_rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    slic = Slic(num_components=num_components, compactness=compactness)
    return slic.iterate(image_rgb)


# ───── OpenCV SLIC 래퍼 ────────────────────────────────
def slic_opencv_bgr(bgr: np.ndarray,
                    region_size: int = 12,
                    ruler      : float = 8.0,
                    iterate    : int   = 5) -> np.ndarray:
    if not hasattr(cv2, 'ximgproc'):
        raise RuntimeError('cv2.ximgproc 모듈이 없습니다. '
                           'opencv‑contrib‑python 설치 필요')
    slicer = cv2.ximgproc.createSuperpixelSLIC(
        bgr, algorithm=cv2.ximgproc.SLICO,
        region_size=region_size, ruler=ruler)
    slicer.iterate(iterate)
    return slicer.getLabels()


# ───── 헬퍼 ────────────────────────────────────────────
def rgb_to_float(r, g, b):
    return struct.unpack('f', struct.pack('I', (r << 16) | (g << 8) | b))[0]

def build_sparse_cloud(color_bgr: np.ndarray,
                       depth_mm : np.ndarray,
                       K        : np.ndarray,
                       scale    : float = 0.4,
                       reg_size : int   = 16,
                       ruler    : float = 10.0):
    """Down‑scale → fast-SLIC → superpixel centroids → Nx6 array"""
    if scale != 1.0:
        color_bgr = cv2.resize(color_bgr, None, fx=scale, fy=scale,
                               interpolation=cv2.INTER_LINEAR)
        depth_mm  = cv2.resize(depth_mm, None, fx=scale, fy=scale,
                               interpolation=cv2.INTER_NEAREST)
        fx, fy, cx, cy = K[0,0]*scale, K[1,1]*scale, K[0,2]*scale, K[1,2]*scale
    else:
        fx, fy, cx, cy = K[0,0], K[1,1], K[0,2], K[1,2]

    # Superpixel segmentation using fast-slic
    h, w = color_bgr.shape[:2]
    num_components = int((h * w) / (reg_size ** 2))
    print(f'num_components: {num_components}')
    labels = fastslic_bgr(color_bgr, num_components=num_components, compactness=ruler)

    pts = []
    for lab in np.unique(labels):
        m = labels == lab
        

        zs = depth_mm[m].astype(np.float32)

        if m.sum() < 30:  # 작은 superpixel 무시
            continue
        if zs.std() > 3000:  # 깊이 값이 흔들리면 무시
            continue
        z = np.median(zs) * 1e-3  # mm → m

        if z <= 0.05:  # 너무 가까운 거리 무시
            continue
        ys, xs = np.nonzero(m)
        u, v   = xs.mean(), ys.mean()
        X, Y   = (u - cx) * z / fx, (v - cy) * z / fy
        b, g, r = [int(color_bgr[:, :, c][m].mean()) for c in range(3)]
        pts.append([X, Y, z, r, g, b])

    return np.asarray(pts, np.float32)


def process_cloud(cloud_rgb: np.ndarray,
                  voxel=0.01, ransac_dist=0.10,
                  eps=1.0, min_samples=3, top_k=20):
    """
    Parameters
    ----------
    cloud_rgb : (N,6)  x,y,z,r,g,b  float32
    Returns
    -------
    pcd_out      : open3d.geometry.PointCloud  (None if nothing to publish)
    ransac_info  : [{'eq':(a,b,c,d), 'inliers':N}, ...]
    db_info      : [{'label':ℓ, 'size':N}, ...]   (top_k by size)
    """
    if cloud_rgb.size == 0:
        return None, [], []

    # ── ①  voxel down‑sample & 바닥(평면) 제거 ─────────────────
    xyz_in, rgb_in = cloud_rgb[:, :3], cloud_rgb[:, 3:] / 255.0
    pcd  = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(xyz_in))
    pcd.colors = o3d.utility.Vector3dVector(rgb_in)
    pcd  = pcd.voxel_down_sample(voxel)

    keep = np.ones(len(pcd.points), bool)
    ransac_info = []
    tmp = pcd
    for _ in range(1):                           # <― 평면 1개만 제거
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

    # ── ②  DBSCAN  ───────────────────────────────────────────
    labels = DBSCAN(eps=eps, min_samples=min_samples).fit_predict(xyz)

    # 노이즈 제거 ──>
    mask   = labels >= 0
    xyz    = xyz[mask]
    labels = labels[mask]
    if xyz.size == 0:
        return None, ransac_info, []

    # 각 클러스터 크기 계산
    uniq, cnts = np.unique(labels, return_counts=True)
    # 내림차순 정렬 후 top‑k
    order   = uniq[np.argsort(cnts)[::-1]][:top_k]
    sizes   = dict(zip(uniq, cnts))
    db_info = [{'label': int(l), 'size': int(sizes[l])} for l in order]

    if not db_info:                # 실제 클러스터가 1개도 없으면 종료
        return None, ransac_info, db_info

    # ── ③  거리 기반 R‑B 그라데이션 색 입히기 ────────────────
    d     = np.linalg.norm(xyz, axis=1)
    d_min = d.min();  d_max = d.max() + 1e-6
    rgb   = np.zeros_like(xyz)                      # (N,3) all zeros
    for lab in order:                               # top_k 만 색칠
        m        = labels == lab
        alpha    = (d[m] - d_min) / (d_max - d_min) # 0(가까움)→1(멀리)
        rgb[m, 0] = alpha                           # R : 멀수록 ↑
        rgb[m, 2] = 1 - alpha                       # B : 가까울수록 ↑

    # ── ④  open3d PointCloud 생성 & 반환 ─────────────────────
    pcd_out = o3d.geometry.PointCloud()
    pcd_out.points = o3d.utility.Vector3dVector(xyz)
    pcd_out.colors = o3d.utility.Vector3dVector(rgb)

    return pcd_out, ransac_info, db_info


def pcd_to_cloud_msg(pcd: o3d.geometry.PointCloud,
                     header_src: Header) -> PointCloud2:
    xyz  = np.asarray(pcd.points)
    rgbf = np.vectorize(rgb_to_float)(
             *(np.asarray(pcd.colors)[:, ::-1]*255).astype(np.uint8).T)
    data = np.hstack([xyz, rgbf[:, None]]).astype(np.float32)

    header          = Header()
    header.stamp    = header_src.stamp
    header.frame_id = header_src.frame_id

    # ─── 여기! keyword 인자로 작성 ─────────────────────
    fields = [
        PointField(name='x',   offset=0,  datatype=PointField.FLOAT32, count=1),
        PointField(name='y',   offset=4,  datatype=PointField.FLOAT32, count=1),
        PointField(name='z',   offset=8,  datatype=PointField.FLOAT32, count=1),
        PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    # ──────────────────────────────────────────────────
    return pc2.create_cloud(header, fields, data.tolist())


# ───── ROS2 노드 ──────────────────────────────────────
class SparseObstacleNode(Node):
    def __init__(self):
        super().__init__('sparse_obstacle_node')
        self.bridge, self.K = CvBridge(), None
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=5)

        c = Subscriber(self, Image , '/camera/camera/color/image_raw', qos_profile=qos)
        d = Subscriber(self, Image , '/camera/camera/aligned_depth_to_color/image_raw', qos_profile=qos)
        i = Subscriber(self, CameraInfo, '/camera/camera/color/camera_info', qos_profile=qos)

        ats = ApproximateTimeSynchronizer([c,d,i], 5, 0.03)
        ats.registerCallback(self.synced_cb)
        self.bridge = CvBridge()
        self.pub_labels = self.create_publisher(Image, '/slic_labels', 10)
        self.pub = self.create_publisher(PointCloud2,'/obstacle_clusters_colored',10)
        self.get_logger().info('SparseObstacleNode ready.')

    # ───────── callback ─────────
    def synced_cb(self, img_msg, depth_msg, info_msg):
        t0 = time.perf_counter()
        if self.K is None:
            self.K = np.asarray(info_msg.k, np.float32).reshape(3,3)
            self.get_logger().info('Intrinsic K initialised.')

        color = self.bridge.imgmsg_to_cv2(img_msg,'bgr8')
        depth = self.bridge.imgmsg_to_cv2(depth_msg,'16UC1')

        cloud6 = build_sparse_cloud(color, depth, self.K)
        h, w = color.shape[:2]
        num_components = int((h * w) / (16 ** 2))  # reg_size=16 기준
        compactness = 10.0  # 보통 쓰던 값

        labels = fastslic_bgr(color, num_components=num_components, compactness=compactness)

        label_msg = self.bridge.cv2_to_imgmsg(labels.astype(np.int32), encoding='32SC1')
        label_msg.header = img_msg.header
        self.pub_labels.publish(label_msg)

        self.get_logger().info(f'SLIC→cloud: {cloud6.shape[0]} pts ({time.perf_counter()-t0:.3f}s)')

        t1 = time.perf_counter()
        pcd, rinfo, dinfo = process_cloud(cloud6)
        self._log_ransac(rinfo)
        self._log_dbscan(dinfo)
        self.get_logger().info(f'Processing time: {time.perf_counter()-t1:.3f}s')

        if pcd is None:
            self.get_logger().info('No PCD to publish. Sending empty cloud to clear RViZ.')
            empty_cloud = pc2.create_cloud(
                img_msg.header,
                fields=[
                    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                    PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1)
                ],
                points=[]  # empty list
            )
            self.pub.publish(empty_cloud)
            return

        self.pub.publish(pcd_to_cloud_msg(pcd, img_msg.header))
        self.get_logger().info(f'Published PCD ({time.perf_counter()-t0:.3f}s total)')

    # ───── helper logs ─────────
    def _log_ransac(self, rinfo):
        if not rinfo:
            self.get_logger().info('RANSAC: no plane removed')
            return
        for k,pl in enumerate(rinfo):
            a,b,c,d = pl["eq"]
            self.get_logger().info(
                f'RANSAC[{k}] inliers={pl["inliers"]:4d}  '
                f'plane:{a:+.3f}x {b:+.3f}y {c:+.3f}z {d:+.3f}=0')

    def _log_dbscan(self, dinfo):
        if not dinfo:
            self.get_logger().info('DBSCAN: no clusters')
            return
        summary = ', '.join([f'ℓ={d["label"]}({d["size"]})' for d in dinfo])
        self.get_logger().info(f'DBSCAN clusters: {summary}')


# ───── main ──────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = SparseObstacleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
