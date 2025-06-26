#!/usr/bin/env python3
"""
crosswalk_supervisor.py
ROS 2 Humble (Python rclpy)

* velocity_smoother 없는 버전
* road_obstacle_layer OFF/ON
* GPS(UTM) 1 Hz로 횡단보도 진입·탈출 판단
* detect_crosswalk 서비스 SAFE 응답 후 출발
* --------------------------------------------------------------------
* 2025-06-25 추가: 10 초 타임아웃 (옵션)               ### TIMEOUT
    - ENTRY 후 10 초 안에 SAFE 응답이 없으면
      • 정지 해제 후 주행 재개
      • 이후 서비스 응답이 와도 무시
    - ENABLE_TIMEOUT=False 로 끄기
"""

import math, yaml, pathlib, functools
from typing import List, Tuple, Optional

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from nav2_msgs.srv import ClearEntireCostmap
from ament_index_python.packages import get_package_share_directory

# ─────────────────────────────────────────────────────────── 설정 상수
UTM_TOPIC  = '/odometry/global'
IN_THRESH  = 1.0  # [m] ─ 1 m 이내를 “횡단보도 안”으로 간주
MAP_PATH   = pathlib.Path(get_package_share_directory('nav2_bringup')) / 'maps' / 'global_map.yaml'

STOP_TOPIC = '/crosswalk_stop'   # Bool true → 정지, false → 주행

# === TIMEOUT 기능 ON/OFF (한 줄로 제어) ================================  ### TIMEOUT
ENABLE_TIMEOUT = True          # False 로 바꾸면 타임아웃 완전 비활성
TIMEOUT_SEC    = 10.0          # 응답 기다릴 최대 시간(초)
# =======================================================================


class CrosswalkSupervisor(Node):
    def __init__(self):
        super().__init__('crosswalk_supervisor')
        cbg = ReentrantCallbackGroup()

        # ─ detect_crosswalk 서비스
        self.detect_cli = self.create_client(Trigger,
                                             '/detect_crosswalk',
                                             callback_group=cbg)

        # ─ costmap 파라미터·클리어
        self.param_cli = self.create_client(SetParameters,
            '/local_costmap/local_costmap/set_parameters', callback_group=cbg)
        self.clear_cli = self.create_client(ClearEntireCostmap,
            '/local_costmap/clear_entirely_local_costmap', callback_group=cbg)

        # ─ 정지/재출발 Bool 토픽
        self.stop_pub = self.create_publisher(Bool, STOP_TOPIC, 10, callback_group=cbg)
        self._publish_stop(False)   # 초기 값: 주행

        # ─ crosswalk 좌표 로드
        self.wpts = self._load_crosswalk_points(MAP_PATH)

        # ─ UTM 포즈(1 Hz) 구독
        self.curr_pos = None
        self.create_subscription(PoseWithCovarianceStamped, UTM_TOPIC,
                                 self._pose_cb, 10, callback_group=cbg)

        # 내부 state
        self.in_prev = False
        self._detect_future = None
        self._timeout_timer: Optional[rclpy.timer.Timer] = None   # ### TIMEOUT

        self.get_logger().info(f'CrosswalkSupervisor ready  |  MAP={MAP_PATH}')

    # ------------------------------------------------ YAML 파싱
    def _load_crosswalk_points(self, path: pathlib.Path) -> List[Tuple[float, float]]:
        data = yaml.safe_load(path.read_text())
        node_tbl = {n['id']: (n['x'], n['y']) for n in data.get('nodes', [])}
        pts: List[Tuple[float, float]] = []
        for e in data.get('edges', []):
            if not e.get('crosswalk', False):
                continue
            wps = e.get('waypoints', [])
            pts += [(wp['x'], wp['y']) for wp in wps] if wps else \
                   [node_tbl[e['from']], node_tbl[e['to']]]
        return pts

    # ------------------------------------------------ 포즈 콜백 (1 Hz)
    def _pose_cb(self, msg: PoseWithCovarianceStamped):
        self.curr_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        dist = self._closest_dist()
        in_now = (dist <= IN_THRESH)

        if in_now and not self.in_prev:
            self._on_entry()
        elif not in_now and self.in_prev:
            self._on_exit()

        self.in_prev = in_now

    def _closest_dist(self) -> float:
        if self.curr_pos is None or not self.wpts:
            return math.inf
        cx, cy = self.curr_pos
        return min(math.hypot(cx - x, cy - y) for x, y in self.wpts)

    # ------------------------------------------------ ENTRY
    def _on_entry(self):
        self.get_logger().info('▶ ENTRY: stop & road layer OFF')
        self._publish_stop(True)
        self._toggle_road_layer(False)

        # detect_crosswalk 서비스 호출
        self._detect_future = self.detect_cli.call_async(Trigger.Request())
        self._detect_future.add_done_callback(self._on_detect_done)

        # -------- 타임아웃 타이머 설정 -------------------------------  ### TIMEOUT
        if ENABLE_TIMEOUT:
            if self._timeout_timer:              # 중복 방지
                self._timeout_timer.cancel()
            self._timeout_timer = self.create_timer(
                TIMEOUT_SEC,
                functools.partial(self._on_timeout, tstart=self.get_clock().now()),
                callback_group=self.get_default_callback_group()
            )

    # ------------------------------------------------ EXIT
    def _on_exit(self):
        self.get_logger().info('↩ EXIT: road layer ON & resume')
        self._publish_stop(False)
        self._toggle_road_layer(True)
        self._cancel_timeout()        # ### TIMEOUT

    # ------------------------------------------------ detect 응답
    def _on_detect_done(self, fut):
        self._cancel_timeout()        # ### TIMEOUT
        try:
            safe = fut.result().success
        except Exception as e:
            self.get_logger().error(f'detect_crosswalk error: {e}')
            return

        if safe:
            self.get_logger().info('SAFE: resume (road layer still OFF)')
            self._publish_stop(False)
        else:
            self.get_logger().info('UNSAFE: keep waiting')

    # ------------------------------------------------ TIMEOUT 핸들러   ### TIMEOUT
    def _on_timeout(self, tstart):
        self.get_logger().warn(
            f'No SAFE response within {TIMEOUT_SEC}s → depart anyway')
        self._publish_stop(False)      # 주행 재개
        # future 결과는 무시 (SAFE / UNSAFE 뒤늦게 와도 상태 유지)
        self._timeout_timer = None

    def _cancel_timeout(self):        # ### TIMEOUT
        if self._timeout_timer:
            self._timeout_timer.cancel()
            self._timeout_timer = None

    # ------------------------------------------------ 유틸
    def _publish_stop(self, flag: bool):
        self.stop_pub.publish(Bool(data=flag))

    def _toggle_road_layer(self, enable: bool):
        p = Parameter(
            name='road_obstacle_layer.enabled',
            value=ParameterValue(type=ParameterType.PARAMETER_BOOL,
                                 bool_value=enable)
        )
        self.param_cli.call_async(SetParameters.Request(parameters=[p]))
        self.clear_cli.call_async(ClearEntireCostmap.Request())

# ---------------------------------------------------------------- main
def main():
    rclpy.init()
    rclpy.spin(CrosswalkSupervisor())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
