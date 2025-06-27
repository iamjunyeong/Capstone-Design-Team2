#!/usr/bin/env python3
"""
crosswalk_supervisor.py  ─ ROS 2 Humble

[역할]
1. 1 Hz UTM 좌표(/odometry/global)로 차량이 횡단보도(edge crosswalk:true)에
   진입/탈출했는지 판단
2. 진입 시
   • /crosswalk_stop  Bool True  → 정지
   • local_costmap road_obstacle_layer.enabled=False  → 연석 무시
   • detect_crosswalk (Trigger) 서비스 SAFE 응답 대기
3. SAFE 응답이 오면 정지 해제 후 주행
4. 10 초 내 SAFE 응답이 없으면 “타임아웃”으로 주행 재개 (옵션)
5. 탈출 시 road layer ON, /crosswalk_stop False

[주요 토픽/서비스]
* /odometry/global (PoseWithCovarianceStamped)   : 1 Hz UTM 좌표
* /crosswalk_stop   (std_msgs/Bool)              : 정지/주행 명령
* /detect_crosswalk (std_srvs/Trigger)           : 비전 안전 판단
* /local_costmap/local_costmap/set_parameters    : road layer 토글
* /local_costmap/clear_entirely_local_costmap    : 즉시 반영
"""

import math, yaml, pathlib
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

# ──────────────────────────── 상수 설정 ────────────────────────────
UTM_TOPIC   = '/odometry/global'        # 1 Hz UTM pose
IN_THRESH   = 1.5                       # [m]  : 횡단보도 판정 거리
MAP_PATH    = pathlib.Path(get_package_share_directory('nav2_bringup')
              ) / 'maps' / 'global_map.yaml'

STOP_TOPIC  = '/crosswalk_stop'         # Bool True 정지 / False 주행

# --- 타임아웃 옵션 -------------------------------------------------
ENABLE_TIMEOUT = True                   # False → 타임아웃 기능 끔
TIMEOUT_SEC    = 15                     # n 초 안에 SAFE 없으면 자동 출발
# ------------------------------------------------------------------

class CrosswalkSupervisor(Node):
    def __init__(self):
        super().__init__('crosswalk_supervisor')

        # 동시 호출을 허용하기 위해 Reentrant Callback Group 사용
        cbg = ReentrantCallbackGroup()

        # 안전 판단 서비스 클라이언트
        self.detect_cli = self.create_client(Trigger,
                                             '/detect_crosswalk',
                                             callback_group=cbg)

        # 로드 레이어 파라미터 토글 + costmap 클리어
        self.param_cli = self.create_client(
            SetParameters,
            '/local_costmap/local_costmap/set_parameters',
            callback_group=cbg)
        self.clear_cli = self.create_client(
            ClearEntireCostmap,
            '/local_costmap/clear_entirely_local_costmap',
            callback_group=cbg)

        # 정지 / 주행 Bool 퍼블리셔
        self.stop_pub = self.create_publisher(
            Bool, STOP_TOPIC, 10, callback_group=cbg)
        self._publish_stop(False)        # 초기 주행 상태

        # 맵에서 횡단보도 포인트 로드
        self.wpts = self._load_crosswalk_points(MAP_PATH)

        # UTM pose 구독
        self.curr_pos = None
        self.create_subscription(
            PoseWithCovarianceStamped, UTM_TOPIC,
            self._pose_cb, 10, callback_group=cbg)

        # 내부 상태 변수
        self.in_prev: bool = False                   # 이전 프레임 횡단보도 여부
        self._detect_future = None                   # detect 서비스 future
        self._timeout_timer: Optional[rclpy.timer.Timer] = None
        self._elapsed: int = 0                       # 경과 초  ### COUNTDOWN

        self._cbg = cbg                              # 타이머에서 사용
        self.get_logger().info(f'CrosswalkSupervisor ready — MAP: {MAP_PATH}')

    # ────────────────────── 맵 YAML 파싱 ──────────────────────
    def _load_crosswalk_points(self, path: pathlib.Path) -> List[Tuple[float,float]]:
        """crosswalk:true edge의 waypoint 또는 from/to 노드 좌표 반환"""
        data = yaml.safe_load(path.read_text())
        node_tbl = {n['id']: (n['x'], n['y']) for n in data['nodes']}
        pts = []
        for e in data['edges']:
            if not e.get('crosswalk', False):
                continue
            wps = e.get('waypoints', [])
            pts += [(wp['x'], wp['y']) for wp in wps] if wps else \
                   [node_tbl[e['from']], node_tbl[e['to']]]
        self.get_logger().info(f'crosswalk points loaded: {len(pts)}개')
        return pts

    # ────────────────────── 포즈 콜백 (1 Hz) ──────────────────────
    def _pose_cb(self, msg: PoseWithCovarianceStamped):
        # 현재 위치 저장
        self.curr_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        # 가장 가까운 횡단보도 점까지 거리
        dist = self._closest_dist()
        in_now = dist <= IN_THRESH

        # rising edge → 진입
        if in_now and not self.in_prev:
            self._on_entry()
        # falling edge → 탈출
        elif not in_now and self.in_prev:
            self._on_exit()

        self.in_prev = in_now

    def _closest_dist(self) -> float:
        """현재 위치와 crosswalk 점 사이의 최단 거리 계산"""
        if self.curr_pos is None or not self.wpts:
            return math.inf
        cx, cy = self.curr_pos
        return min(math.hypot(cx - x, cy - y) for x, y in self.wpts)

    # ────────────────────── 진입 처리 ──────────────────────
    def _on_entry(self):
        self.get_logger().info('▶ ENTRY: 정지 및 road layer OFF')
        self._publish_stop(True)              # 정지
        self._toggle_road_layer(False)        # 연석 무시

        # 비전팀 안전 판단 서비스 호출
        self._detect_future = self.detect_cli.call_async(Trigger.Request())
        self._detect_future.add_done_callback(self._on_detect_done)

        # --- 1 초 주기 타임아웃 카운트다운 시작 ---------------- ### COUNTDOWN
        if ENABLE_TIMEOUT:
            if self._timeout_timer:
                self._timeout_timer.cancel()
            self._elapsed = 0
            self._timeout_timer = self.create_timer(
                1.0, self._timeout_tick, callback_group=self._cbg)

    # ────────────────────── 탈출 처리 ──────────────────────
    def _on_exit(self):
        self.get_logger().info('↩ EXIT: road layer ON 및 주행 재개')
        self._publish_stop(False)             # 주행
        self._toggle_road_layer(True)         # 연석 다시 활성
        self._cancel_timeout()                # 카운트다운 중단

    # ────────────────────── SAFE 응답 콜백 ──────────────────────
    def _on_detect_done(self, fut):
        self._cancel_timeout()
        try:
            safe = fut.result().success
        except Exception as e:
            self.get_logger().error(f'detect_crosswalk error: {e}')
            return

        if safe:
            self.get_logger().info('SAFE → 주행 재개 (layer OFF 유지)')
            self._publish_stop(False)         # 정지 해제
        else:
            self.get_logger().info('UNSAFE → 계속 대기')

    # ────────────────────── 카운트다운 tick ──────────────────────
    def _timeout_tick(self):
        """1 초마다 호출되어 남은 시간을 로그. 10 초가 되면 출발"""
        self._elapsed += 1
        self.get_logger().info(
            f'waiting SAFE… {self._elapsed}/{TIMEOUT_SEC}s')
        if self._elapsed >= TIMEOUT_SEC:
            self.get_logger().warn(
                'TIMEOUT 도달 → 안전 판단 무시하고 출발')
            self._publish_stop(False)         # 주행
            self._cancel_timeout()

    def _cancel_timeout(self):
        """타이머가 살아있으면 취소하고 핸들 초기화"""
        if self._timeout_timer:
            self._timeout_timer.cancel()
            self._timeout_timer = None

    # ────────────────────── 헬퍼 함수 ──────────────────────
    def _publish_stop(self, flag: bool):
        """/crosswalk_stop 토픽 publish"""
        self.stop_pub.publish(Bool(data=flag))

    def _toggle_road_layer(self, enable: bool):
        """Costmap Layer 파라미터 토글 + 클리어"""
        param = Parameter(
            name='road_obstacle_layer.enabled',
            value=ParameterValue(
                type=ParameterType.PARAMETER_BOOL,
                bool_value=enable
            )
        )
        self.param_cli.call_async(SetParameters.Request(parameters=[param]))
        self.clear_cli.call_async(ClearEntireCostmap.Request())

# ────────────────────── main ──────────────────────
def main():
    rclpy.init()
    rclpy.spin(CrosswalkSupervisor())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
