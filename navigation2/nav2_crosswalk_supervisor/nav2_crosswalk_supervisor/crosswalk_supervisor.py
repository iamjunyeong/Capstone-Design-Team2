#!/usr/bin/env python3
"""
crosswalk_supervisor.py
ROS 2 Humble (Python rclpy)

변경 사항 — 2025-06-21
────────────────────
* UTM 좌표 입력 토픽을 **/odometry/global**(geometry_msgs/PoseWithCovarianceStamped) 로 변경
  → GPS 1 Hz 이므로 노드는 1 Hz 주기로만 동작
* slow/fast 타이머 제거 → **포즈 콜백 한 곳**에서 진입·탈출 판정
* ‘SAFE’ 응답 시엔 **speed 제한만 해제**, road layer OFF 유지  
  (횡단보도를 완전히 벗어날 때 `_on_exit()`에서만 ON)

기능
1) UTM 현재 좌표와 global_map.yaml 의 crosswalk:true edge 좌표를 비교
2) 진입 시  • SetSpeedLimit(0)   • road_obstacle_layer.enabled=False   • detect_crosswalk 서비스 호출
3) SAFE 판단 후 속도 해제 (road layer는 계속 OFF)
4) 경계선 벗어나면 road layer ON + costmap clear
"""

import math, yaml, pathlib
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import PoseWithCovarianceStamped          # ← GPS 1 Hz 포즈
from std_srvs.srv import Trigger
from nav2_msgs.srv import SetSpeedLimit, ClearEntireCostmap
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

# ---------------------------------------------------------------- 설정 상수
UTM_TOPIC  = '/odometry/global'   # PoseWithCovarianceStamped, frame_id "map"
IN_THRESH  = 1.5                  # [m]  ≤ 1.5 m → “횡단보도 안”
MAP_PATH   = pathlib.Path(__file__).parent / 'nav2_bringup' / 'maps' / 'global_map.yaml'

class CrosswalkSupervisor(Node):
    def __init__(self):
        super().__init__('crosswalk_supervisor')
        cbg = ReentrantCallbackGroup()

        # ─ Nav2 서비스 클라이언트 ─
        self.speed_cli  = self.create_client(SetSpeedLimit,
                                             '/controller_server/set_speed_limit',
                                             callback_group=cbg)
        self.detect_cli = self.create_client(Trigger,
                                             '/detect_crosswalk',
                                             callback_group=cbg)
        self.param_cli  = self.create_client(SetParameters,
            '/local_costmap/local_costmap/set_parameters', callback_group=cbg)
        self.clear_cli  = self.create_client(ClearEntireCostmap,
            '/local_costmap/clear_entirely_local_costmap', callback_group=cbg)

        # ─ crosswalk 좌표 로드 ─
        self.wpts = self._load_crosswalk_points(MAP_PATH)

        # ─ UTM 포즈 구독 (1 Hz) ─
        self.curr_pos = None
        self.create_subscription(PoseWithCovarianceStamped, UTM_TOPIC,
                                 self._pose_cb, 10, callback_group=cbg)

        # 내부 상태
        self.in_prev = False
        self.get_logger().info('CrosswalkSupervisor ready (1 Hz)')

    # ---------------------------------------------------- YAML 파싱
    def _load_crosswalk_points(self, path: pathlib.Path) -> List[Tuple[float,float]]:
        data      = yaml.safe_load(path.read_text())
        node_tbl  = {n['id']:(n['x'], n['y']) for n in data.get('nodes', [])}
        pts: List[Tuple[float,float]] = []
        for e in data.get('edges', []):
            if not e.get('crosswalk', False):
                continue
            wps = e.get('waypoints', [])
            if wps:
                pts += [(wp['x'], wp['y']) for wp in wps]
            else:                                      # waypoint 없으면 from/to 노드
                pts += [node_tbl[e['from']], node_tbl[e['to']]]
        self.get_logger().info(f'Loaded {len(pts)} crosswalk points')
        return pts

    # ---------------------------------------------------- 포즈 콜백 (1 Hz)
    def _pose_cb(self, msg: PoseWithCovarianceStamped):
        self.curr_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        dist = self._closest_dist()
        in_now = (dist <= IN_THRESH)

        if in_now and not self.in_prev:           # 진입 edge
            self._on_entry()
        elif not in_now and self.in_prev:         # 탈출 edge
            self._on_exit()

        self.in_prev = in_now

    # ---------------------------------------------------- 유틸
    def _closest_dist(self) -> float:
        if self.curr_pos is None or not self.wpts:
            return math.inf
        cx, cy = self.curr_pos
        return min(math.hypot(cx-x, cy-y) for x,y in self.wpts)

    # ---------------------------------------------------- 상태 전환
    def _on_entry(self):
        self.get_logger().info('▶ ENTRY: stop & road layer OFF')
        self._set_speed(0.0)
        self._toggle_road_layer(False)          # 끄기

        # detect_crosswalk(무한 루프 내부 3 s 주기) 호출
        fut = self.detect_cli.call_async(Trigger.Request())
        fut.add_done_callback(self._on_detect_done)

    def _on_exit(self):
        self.get_logger().info('↩ EXIT: restore road layer ON')
        self._set_speed(float('nan'))           # 속도 제한 해제
        self._toggle_road_layer(True)           # 켜기

    def _on_detect_done(self, fut):
        """SAFE 응답 → 속도 해제. (road layer ON 은 _on_exit()에서)"""
        try:
            safe = fut.result().success
        except Exception as e:
            self.get_logger().error(f'detect_crosswalk call failed: {e}')
            return

        if safe:
            self.get_logger().info('SAFE: resume (road layer still OFF)')
            self._set_speed(float('nan'))
        else:
            # 일반적으로 False 응답은 오지 않음(무한 루프). 예외 대비.
            self.get_logger().warn('UNSAFE: keep stopping')

    # ---------------------------------------------------- 서비스 래퍼
    def _set_speed(self, val: float):
        req = SetSpeedLimit.Request(speed_limit=val, percentage=False)
        self.speed_cli.call_async(req)

    def _toggle_road_layer(self, enable: bool):
        param = Parameter(
            name='road_obstacle_layer.enabled',
            value=ParameterValue(type=ParameterType.PARAMETER_BOOL,
                                 bool_value=enable)
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
