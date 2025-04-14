#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import struct

import open3d as o3d  # pip install open3d
# 기존 scikit-learn 대신 cuML DBSCAN 사용 (CUDA 환경 필요)
from cuml.cluster import DBSCAN as cuDBSCAN
import cupy as cp

import os

def rgb_to_float(r, g, b):
    """Convert 8-bit r, g, b values (0–255) to a packed float32 value."""
    rgb_int = (int(r) << 16) | (int(g) << 8) | int(b)
    return struct.unpack('f', struct.pack('I', rgb_int))[0]

class PlaneClusterNode(Node):
    def __init__(self):
        super().__init__('plane_cluster_node')
        
        # 비동기 처리를 위해 처리중 여부 플래그 초기화
        self.processing_flag = False
        
        # 구독할 PointCloud2 토픽 (예: /camera/camera/depth/color/points)
        self.subscription = self.create_subscription(
            PointCloud2,
            '/camera/camera/depth/color/points',
            self.pointcloud_callback,
            10  # 큐 크기가 10: 최대 10개의 메시지를 버퍼링
        )
        
        # 처리된 결과 (색상 정보 포함)를 퍼블리시할 토픽
        self.publisher_ = self.create_publisher(PointCloud2, '/obstacle_clusters_colored', 10)
        self.get_logger().info("PlaneClusterNode initialized and subscribed to point cloud topic.")
    
    def pointcloud_callback(self, msg: PointCloud2):
        # 만약 이전 메시지 처리가 아직 진행 중이면 드롭
        if self.processing_flag:
            self.get_logger().info("Previous processing not finished; dropping this message.")
            return
        
        self.processing_flag = True  # 처리 시작
        
        # 1) ROS2 PointCloud2 메시지를 NumPy 배열로 변환 (XYZ만 추출)
        points = self.convert_cloud_to_numpy(msg)
        if points.shape[0] < 3:
            self.processing_flag = False
            return
        self.get_logger().info(f"Input points: {points.shape[0]}")
        
        # 1a) 다운샘플링: 전체 포인트 수를 줄임 (voxel 크기: 2cm)
        voxel_size = 0.02
        pcd_orig = o3d.geometry.PointCloud()
        pcd_orig.points = o3d.utility.Vector3dVector(points[:, :3])
        pcd = pcd_orig.voxel_down_sample(voxel_size=voxel_size)
        points_ds = np.asarray(pcd.points)
        self.get_logger().info(f"Downsampled points: {points_ds.shape[0]}")
        
        # 2) 여러 평면 후보 추출 (예시로 2회 반복)
        candidate_planes = []  # 각 원소: (inlier_indices, avg_z)
        pcd_remaining = pcd  # 점들을 계속 업데이트
        num_candidates = 2
        for i in range(num_candidates):
            if np.asarray(pcd_remaining.points).shape[0] < 100:
                break
            plane_model, inlier_indices = pcd_remaining.segment_plane(
                distance_threshold=0.1,  # 평면 판단 임계값
                ransac_n=3,
                num_iterations=200
            )
            inlier_indices = np.asarray(inlier_indices)
            if len(inlier_indices) < 50:  # 후보로 보기 어려우면 중단
                break
            candidate = pcd_remaining.select_by_index(inlier_indices)
            candidate_np = np.asarray(candidate.points)
            avg_z = np.mean(candidate_np[:,2])
            candidate_planes.append((inlier_indices, avg_z))
            # 제거하여 남은 점으로 업데이트
            pcd_remaining = pcd_remaining.select_by_index(inlier_indices, invert=True)
            self.get_logger().info(f"Candidate {i}: {len(inlier_indices)} points, avg_z={avg_z:.3f}")
        
        # 2a) 바닥면으로 간주할 후보 선택: inlier 포인트의 평균 z값이 가장 낮은 평면
        if candidate_planes:
            floor_candidate = min(candidate_planes, key=lambda x: x[1])
            floor_inliers = floor_candidate[0]
            self.get_logger().info(f"Selected floor candidate with avg_z={floor_candidate[1]:.3f}")
        else:
            floor_inliers = np.array([], dtype=int)
            self.get_logger().warn("No candidate floor plane detected.")
        
        # 2b) 원본 다운샘플링 데이터(points_ds)에서 바닥 평면 제거:
        mask = np.ones(len(points_ds), dtype=bool)
        if floor_inliers.size > 0:
            mask[floor_inliers] = False
        remaining_points = points_ds[mask]
        self.get_logger().info(f"After floor removal: {remaining_points.shape[0]} points")
        
        # 3) cuML DBSCAN을 이용하여 남은 점들 군집화
        # 먼저, GPU 배열(CuPy)로 변환
        
        remaining_points_gpu = cp.asarray(remaining_points[:, :3])
        clustering = cuDBSCAN(eps=0.05, min_samples=20)
        clustering.fit(remaining_points_gpu)
        labels_gpu = clustering.labels_
        labels = cp.asnumpy(labels_gpu)
        unique_labels, counts = np.unique(labels, return_counts=True)
        self.get_logger().info(f"Initial clusters: {dict(zip(unique_labels, counts))}")
        
        # 4) 클러스터 필터링 및 정렬: 노이즈(-1) 제외, 군집 크기를 기준으로 내림차순 정렬 후 상위 10개 선택
        clusters_info = []  # 각 튜플: (label, avg_distance, cluster_size)
        for lab in unique_labels:
            if lab == -1:
                continue
            cluster_pts = remaining_points[labels == lab]
            avg_distance = np.mean(np.linalg.norm(cluster_pts, axis=1))
            cluster_size = cluster_pts.shape[0]
            clusters_info.append((lab, avg_distance, cluster_size))
        clusters_info.sort(key=lambda x: x[2], reverse=True)
        top_clusters = clusters_info[:10]
        top_labels = [lab for lab, _, _ in top_clusters]
        self.get_logger().info(f"Top clusters (by size): {top_clusters}")
        
        # 5) 선택한 클러스터에 대해 평균 거리 범위를 기준으로 색상 그라데이션 결정
        if top_clusters:
            distances = [avg for _, avg, _ in top_clusters]
            min_distance = min(distances)
            max_distance = max(distances)
        else:
            min_distance, max_distance = 0, 1
            
        cluster_color_map = {}  # {label: (r, g, b)}
        for lab, avg_distance, _ in top_clusters:
            if max_distance > min_distance:
                alpha = (avg_distance - min_distance) / (max_distance - min_distance)
            else:
                alpha = 0.0
            # 가까울수록 파랑 (0,0,255), 멀수록 빨강 (255,0,0)
            r = int(alpha * 255)
            g = 0
            b = int((1 - alpha) * 255)
            cluster_color_map[lab] = (r, g, b)
        self.get_logger().info(f"Cluster color map: {cluster_color_map}")
        
        # 6) 선택된 클러스터에 속한 점들에 대해 색상 정보 추가 → (x, y, z, rgb)
        colored_points_list = []
        for lab in top_labels:
            pts = remaining_points[labels == lab]
            color = cluster_color_map[lab]
            for pt in pts:
                rgb_float = rgb_to_float(*color)
                colored_points_list.append((pt[0], pt[1], pt[2], rgb_float))
        if not colored_points_list:
            self.get_logger().warn("No colored points generated after filtering top clusters.")
            self.processing_flag = False
            return
        colored_points = np.array(colored_points_list, dtype=np.float32)
        
        # 7) 색상 정보를 포함한 PointCloud2 메시지 생성 후 퍼블리시
        out_msg = self.convert_numpy_to_cloud_colored(colored_points, msg.header)
        self.publisher_.publish(out_msg)
        
        self.processing_flag = False  # 처리 끝 플래그 해제

    def convert_cloud_to_numpy(self, ros_cloud: PointCloud2) -> np.ndarray:
        point_list = []
        for point in pc2.read_points(ros_cloud, field_names=("x", "y", "z"), skip_nans=True):
            point_list.append([point[0], point[1], point[2]])
        if not point_list:
            return np.array([])
        return np.array(point_list, dtype=np.float32)
    
    def convert_numpy_to_cloud_colored(self, colored_points: np.ndarray, header: Header) -> PointCloud2:
        """
        Convert a NumPy array of colored points (N, 4) where columns 0-2 are x, y, z and column 3 is a packed rgb float,
        into a ROS2 PointCloud2 message with an rgb field.
        """
        new_header = Header()
        new_header.stamp = header.stamp
        new_header.frame_id = header.frame_id
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        cloud_msg = pc2.create_cloud(new_header, fields, colored_points.tolist())
        return cloud_msg

def main(args=None):
    rclpy.init(args=args)
    node = PlaneClusterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
