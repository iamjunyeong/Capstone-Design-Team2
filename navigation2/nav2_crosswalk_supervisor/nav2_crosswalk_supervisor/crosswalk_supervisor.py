#!/usr/bin/env python3
"""
crosswalk_supervisor.py
ROS 2 Humble (Python rclpy)

변경 사항 — 2025‑06‑25
────────────────────
* velocity_smoother.speed_limit 파라미터 **폐지** → /crosswalk_stop Bool 토픽으로 제어
* 기존 로직·주석 최대한 유지, 추가 부분은  ### NEW  로 표시
"""

import math, yaml, pathlib
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Bool                     # ### NEW
from std_srvs.srv import Trigger
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from nav2_msgs.srv import ClearEntireCostmap
from ament_index_python.packages import get_package_share_directory

# ---------------------------------------------------------------- 설정 상수
UTM_TOPIC  = '/odometry/global'     # PoseWithCovarianceStamped, frame_id "map"
IN_THRESH  = 1.5                    # [m]  ≤ 1.5 m → “횡단보도 안”

# ### FIX — 패키지 share 디렉터리에서 맵 경로 가져오기 (src 원본이 symlink‑install 로 노출)
MAP_PATH = (
    pathlib.Path(get_package_share_directory('nav2_bringup')) /
    'maps' / 'global_map.yaml'
)

# /crosswalk_stop 토픽 (Bool)
STOP_TOPIC = '/crosswalk_stop'

class CrosswalkSupervisor(Node):
    def __init__(self):
        super().__init__('crosswalk_supervisor')
        cbg = ReentrantCallbackGroup()

        # ─ detect_crosswalk 서비스 (Vision 노드 안전 판단)
        self.detect_cli = self.create_client(Trigger,
                                             '/detect_crosswalk',
                                             callback_group=cbg)

        # ─ costmap 파라미터·클리어
        self.param_cli  = self.create_client(SetParameters,
            '/local_costmap/local_costmap/set_parameters', callback_group=cbg)
        self.clear_cli  = self.create_client(ClearEntireCostmap,
            '/local_costmap/clear_entirely_local_costmap', callback_group=cbg)

        # ### NEW — /crosswalk_stop 퍼블리셔 (True 정지 / False 주행)
        self.stop_pub = self.create_publisher(Bool, STOP_TOPIC, 10, callback_group=cbg)
        self._publish_stop(False)        # 초기값 : 주행 허용

        # ─ crosswalk 좌표 로드
        self.wpts = self._load_crosswalk_points(MAP_PATH)

        # ─ UTM 포즈 (1 Hz) 구독
        self.curr_pos = None
        self.create_subscription(PoseWithCovarianceStamped, UTM_TOPIC,
                                 self._pose_cb, 10, callback_group=cbg)

        self.in_prev = False
        self.get_logger().info(f'CrosswalkSupervisor ready  |  MAP={MAP_PATH}')

    # ---------------------------------------------------- YAML 파싱
    def _load_crosswalk_points(self, path: pathlib.Path) -> List[Tuple[float, float]]:
        data     = yaml.safe_load(path.read_text())
        node_tbl = {n['id']: (n['x'], n['y']) for n in data.get('nodes', [])}
        pts: List[Tuple[float, float]] = []
        for e in data.get('edges', []):
            if not e.get('crosswalk', False):
                continue
            wps = e.get('waypoints', [])
            if wps:
                pts += [(wp['x'], wp['y']) for wp in wps]
            else:
                pts += [node_tbl[e['from']], node_tbl[e['to']]]
        self.get_logger().info(f'Loaded {len(pts)} crosswalk points')
        return pts

    # ---------------------------------------------------- 포즈 콜백 (1 Hz)
    def _pose_cb(self, msg: PoseWithCovarianceStamped):
        self.curr_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        dist  = self._closest_dist()
        in_now = (dist <= IN_THRESH)

        if in_now and not self.in_prev:      # 진입
            self._on_entry()
        elif not in_now and self.in_prev:    # 탈출
            self._on_exit()

        self.in_prev = in_now

    def _closest_dist(self) -> float:
        if self.curr_pos is None or not self.wpts:
            return math.inf
        cx, cy = self.curr_pos
        return min(math.hypot(cx - x, cy - y) for x, y in self.wpts)

    # ---------------------------------------------------- 상태 전환
    def _on_entry(self):
        self.get_logger().info('▶ ENTRY: stop & road layer OFF')
        self._publish_stop(True)                 # ### NEW 정지
        self._toggle_road_layer(False)

        fut = self.detect_cli.call_async(Trigger.Request())
        fut.add_done_callback(self._on_detect_done)

    def _on_exit(self):
        self.get_logger().info('↩ EXIT: road layer ON & resume')
        self._publish_stop(False)                # ### NEW 주행 재개
        self._toggle_road_layer(True)

    def _on_detect_done(self, fut):
        try:
            safe = fut.result().success
        except Exception as e:
            self.get_logger().error(f'detect_crosswalk error: {e}')
            return

        if safe:
            self.get_logger().info('SAFE: resume (road layer still OFF)')
            self._publish_stop(False)            # ### NEW 주행 재개
        else:
            self.get_logger().info('UNSAFE: keep waiting')

    # ---------------------------------------------------- 헬퍼
    def _publish_stop(self, flag: bool):        # ### NEW
        """True : 정지  |  False : 주행"""
        self.stop_pub.publish(Bool(data=flag))

    def _toggle_road_layer(self, enable: bool):
        param = Parameter(
            name='road_obstacle_layer.enabled',
            value=ParameterValue(
                type=ParameterType.PARAMETER_BOOL,
                bool_value=enable
            )
        )
        self.param_cli.call_async(SetParameters.Request(parameters=[param]))
        self.clear_cli.call_async(ClearEntireCostmap.Request())

# ------------------------------------------------------------------ main
def main():
    rclpy.init()
    rclpy.spin(CrosswalkSupervisor())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
