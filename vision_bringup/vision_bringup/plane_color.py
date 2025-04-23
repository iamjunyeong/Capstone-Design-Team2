#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import struct
import open3d as o3d  # pip install open3d
# Open3D Tensor API용 모듈
import open3d.core as o3c
import open3d.t.geometry as o3dt
import os

# DBSCAN은 이번 코드에서는 사용하지 않음

def rgb_to_float(r, g, b):
    """Convert 8-bit r, g, b values (0–255) to a packed float32 value."""
    rgb_int = (int(r) << 16) | (int(g) << 8) | int(b)
    return struct.unpack('f', struct.pack('I', rgb_int))[0]

class PlaneClusterNode(Node):
    def __init__(self):
        super().__init__('plane_cluster_node')
        
        # 처리 중 여부 플래그 (비동기 방지를 위한 간단한 재진입 제어)
        self.processing_flag = False
        
        # 파라미터 설정
        self.voxel_size = 0.1          # 다운샘플링 voxel 크기 (미터)
        self.distance_threshold = 0.05  # RANSAC 평면 판단 임계값 (미터)
        self.n_green_candidates = 6
        
        # QoS 설정: 최신 메시지 1개만 유지
        self.qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.subscription = self.create_subscription(
            PointCloud2,
            '/camera/camera/depth/color/points',
            self.pointcloud_callback,
            self.qos
        )
        self.publisher_ = self.create_publisher(PointCloud2, '/obstacle_clusters_colored', 10)
        self.get_logger().info("PlaneClusterNode initialized and subscribed to point cloud topic.")
    
    def pointcloud_callback(self, msg: PointCloud2):
        if self.processing_flag:
            self.get_logger().info("Previous processing not finished; dropping this message.")
            return
        
        self.processing_flag = True
        
        # 1) ROS2 PointCloud2 메시지를 NumPy 배열로 변환 (XYZ만 추출)
        points = self.convert_cloud_to_numpy(msg)
        if points.shape[0] < 3:
            self.processing_flag = False
            return
        self.get_logger().info(f"Input points: {points.shape[0]}")
        
        # 1a) 다운샘플링: legacy Open3D PointCloud를 생성하여 voxel_down_sample 수행
        pcd_orig = o3d.geometry.PointCloud()
        pcd_orig.points = o3d.utility.Vector3dVector(points[:, :3])
        pcd = pcd_orig.voxel_down_sample(voxel_size=self.voxel_size)
        points_ds = np.asarray(pcd.points)
        self.get_logger().info(f"Downsampled points: {points_ds.shape[0]}")
        
        # 2) RANSAC을 사용하여 평면 후보들을 추출 (최대 num_candidates회 반복)
        candidate_planes = []  # 각 후보: (inlier_indices, avg_z, inlier_count)
        pcd_remaining = pcd  # 반복마다 업데이트
        num_candidates = 8
        for i in range(num_candidates):
            if np.asarray(pcd_remaining.points).shape[0] < 100:
                break
            plane_model, inlier_indices = pcd_remaining.segment_plane(
                distance_threshold=self.distance_threshold,
                ransac_n=3,
                num_iterations=1000
            )
            inlier_indices = np.asarray(inlier_indices)
            if len(inlier_indices) < 50:
                break
            candidate = pcd_remaining.select_by_index(inlier_indices)
            candidate_np = np.asarray(candidate.points)
            avg_z = np.mean(candidate_np[:,2])
            candidate_planes.append((inlier_indices, avg_z, len(inlier_indices)))
            # 후보 평면 부분은 제거하여 다음 후보 검출에 사용
            pcd_remaining = pcd_remaining.select_by_index(inlier_indices, invert=True)
            self.get_logger().info(f"Candidate {i}: {len(inlier_indices)} points, avg_z={avg_z:.3f}")
        
        # 2a) 두 후보 선택:
        # 첫 번째 후보 (red): 평균 z가 가장 큰 후보(즉, 높은 영역)와 inlier 수가 많아야 함.
        # 두 번째 후보 (green): 첫 번째 후보의 avg_z보다 낮은 후보들 중 inlier 수가 가장 많은 후보.
        if candidate_planes:
            # 먼저, 후보들을 평균 z 값(내림차순)과 inlier count(내림차순)로 정렬
            # 여기서는 우선순위를 평균 z 값에 두되, inlier 수도 고려합니다.
            candidate_planes_sorted = sorted(candidate_planes, key=lambda x: (x[1]), reverse=True)
            red_candidate = candidate_planes_sorted[0]
            red_inliers = red_candidate[0]
            red_avg_z = red_candidate[1]
            self.get_logger().info(f"Selected red candidate: {len(red_inliers)} inliers, avg_z={red_avg_z:.3f}")
            
            # 두 번째 후보: red_candidate의 avg_z보다 낮은 후보 중에서 inlier 수가 가장 많은 후보 선택
            # 두 번째 후보: red_candidate의 avg_z보다 낮은 후보 중에서 inlier 수 기준 내림차순으로 정렬
            green_candidates = [cand for cand in candidate_planes_sorted if cand[1] < red_avg_z]
            if green_candidates:
                # inlier 수(즉, cand[2]) 기준 내림차순 정렬
                green_candidates_sorted = sorted(green_candidates, key=lambda x: x[2], reverse=True)
                # 상위 n개 후보를 선택 (n은 self.n_green_candidates로 조정 가능)
                top_green_candidates = green_candidates_sorted[:self.n_green_candidates]
                # 선택된 후보들의 inlier 인덱스의 합집합을 green candidate로 사용
                green_inliers = np.unique(np.concatenate([cand[0] for cand in top_green_candidates]))
                # green 후보의 평균 z 값을, 예를 들면 상위 후보 중 가장 inlier 수가 많은 후보의 avg_z로 결정
                green_candidate = green_candidates_sorted[0]
                green_avg_z = green_candidate[1]
                self.get_logger().info(f"Selected green candidate: {green_inliers.size} inliers, avg_z={green_avg_z:.3f}")
            else:
                green_inliers = np.array([], dtype=int)
                self.get_logger().warn("No green candidate found (no candidate with lower avg_z than red).")

        
        # 2b) 전체 다운샘플링 데이터(points_ds)에서 선택된 두 후보들의 inlier 점들은 제거하지 않고,
        #     해당 인덱스에 대해 색상을 부여합니다.
        total_points = points_ds.shape[0]
        colors = np.zeros((total_points, 3), dtype=np.float32)
        mask_red = np.zeros(total_points, dtype=bool)
        mask_green = np.zeros(total_points, dtype=bool)
        if red_inliers.size > 0:
            mask_red[red_inliers] = True
        if green_inliers.size > 0:
            mask_green[green_inliers] = True
        # 만약 두 후보가 겹치면, red 후보 우선.
        mask_green = np.logical_and(mask_green, np.logical_not(mask_red))
        colors[mask_red] = [255, 0, 0]    # Red candidate
        colors[mask_green] = [0, 255, 0]    # Green candidate
        
        # 나머지 포인트는 파랑으로 설정
        mask_remaining = np.logical_not(np.logical_or(mask_red, mask_green))
        colors[mask_remaining] = [0, 0, 255]
        
        self.get_logger().info(f"Assigned colors: {np.unique(colors, axis=0)}")
        
        # 3) 각 점과 색상을 (x, y, z, rgb) 형식으로 결합
        colored_points_list = []
        for i in range(total_points):
            pt = points_ds[i]
            rgb_float = rgb_to_float(*colors[i])
            colored_points_list.append((pt[0], pt[1], pt[2], rgb_float))
        colored_points = np.array(colored_points_list, dtype=np.float32)
        
        # 4) 새로운 PointCloud2 메시지 생성 후 퍼블리시
        out_msg = self.convert_numpy_to_cloud_colored(colored_points, msg.header)
        self.publisher_.publish(out_msg)
        
        self.processing_flag = False  # 처리 완료

    def convert_cloud_to_numpy(self, ros_cloud: PointCloud2) -> np.ndarray:
        point_list = []
        for point in pc2.read_points(ros_cloud, field_names=("x", "y", "z"), skip_nans=True):
            point_list.append([point[0], point[1], point[2]])
        if not point_list:
            return np.array([])
        return np.array(point_list, dtype=np.float32)
    
    def convert_numpy_to_cloud_colored(self, colored_points: np.ndarray, header: Header) -> PointCloud2:
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
