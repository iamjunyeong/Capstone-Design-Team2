#!/usr/bin/env python3
"""
crosswalk_supervisor.py
ROS 2 Humble (Python rclpy)

기능
1) UTM 현재 좌표와 global_map.yaml 에서 crosswalk:true 인 edge 좌표를 비교
   • 5 m 밖 → 1 Hz slow 루프
   • 5 m 안 → 10 Hz fast 루프
2) 진입 시
   • Nav2 SetSpeedLimit(0) → 정지
   • local_costmap 의 road_obstacle_layer.enabled=False  → 연석 무시
   • detect_crosswalk(Trigger) 서비스 호출로 (무한 루프 안 3 s 주기) 안전 확인
3) SAFE 판단(속도만 해제) 후에도 횡단보도 벗어날 때까지
   road_obstacle_layer 는 OFF 상태 유지
4) 경계선 벗어나면
   • road_obstacle_layer.enabled=True  • costmap 클리어
"""

import math
import yaml
import pathlib
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import PointStamped
from std_srvs.srv import Trigger
from nav2_msgs.srv import SetSpeedLimit, ClearEntireCostmap
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

# ----- 설정 상수 ------------------------------------------------------------
UTM_TOPIC   = '/utm_pose'           # 실제 UTM 좌표 토픽 이름으로 바꿔 주세요
FAR_THRESH  = 5.0                   # [m]  >5 m  : slow loop
IN_THRESH   = 1.5                   # [m] ≤1.5 m : “횡단보도 안” 판정
FAST_HZ     = 10.0
SLOW_HZ     = 1.0
MAP_PATH    = pathlib.Path(__file__).parent / 'nav2_bringup' / 'maps' / 'global_map.yaml'

class CrosswalkSupervisor(Node):
    def __init__(self):
        super().__init__('crosswalk_supervisor')
        cbg = ReentrantCallbackGroup()

        # ─ Nav2 서비스 클라이언트들 ─
        self.speed_cli = self.create_client(SetSpeedLimit,
                                            '/controller_server/set_speed_limit',
                                            callback_group=cbg)
        self.detect_cli = self.create_client(Trigger,
                                             '/detect_crosswalk',
                                             callback_group=cbg)
        self.param_cli = self.create_client(SetParameters,
            '/local_costmap/local_costmap/set_parameters', callback_group=cbg)
        self.clear_cli = self.create_client(ClearEntireCostmap,
            '/local_costmap/clear_entirely_local_costmap', callback_group=cbg)

        # ─ crosswalk 좌표 캐싱 ─
        self.wpts = self._load_crosswalk_points(MAP_PATH)

        # ─ 현재 UTM 위치 구독 ─
        self.curr_pos = None
        self.create_subscription(PointStamped, UTM_TOPIC,
                                 self._pose_cb, 10, callback_group=cbg)

        # ─ 타이머 (slow·fast) ─
        self.fast_timer = self.create_timer(1/FAST_HZ,
                                            self._fast_loop, callback_group=cbg)
        self.fast_timer.cancel()         # 처음엔 slow loop만
        self.slow_timer = self.create_timer(1/SLOW_HZ,
                                            self._slow_loop, callback_group=cbg)

        # 내부 상태
        self.in_prev = False
        self.get_logger().info('CrosswalkSupervisor initialised')

    # ------------------------------------------------------------------ YAML 파싱
    def _load_crosswalk_points(self, yaml_path: pathlib.Path) -> List[Tuple[float, float]]:
        """crosswalk:true edge의 모든 waypoint 좌표 + (waypoint 없을 땐) from/to 노드 좌표"""
        data = yaml.safe_load(yaml_path.read_text())
        node_tbl = {n['id']: (n['x'], n['y']) for n in data.get('nodes', [])}

        pts = []
        for edge in data.get('edges', []):
            if not edge.get('crosswalk', False):
                continue
            waypoints = edge.get('waypoints', [])
            if waypoints:
                pts += [(wp['x'], wp['y']) for wp in waypoints]
            else:  # waypoint 비어 있으면 양 끝 노드 좌표 추가
                pts += [node_tbl[edge['from']], node_tbl[edge['to']]]
        self.get_logger().info(f'Loaded {len(pts)} crosswalk points')
        return pts

    # ------------------------------------------------------------------ 기본 유틸
    def _pose_cb(self, msg: PointStamped):
        self.curr_pos = (msg.point.x, msg.point.y)

    def _closest_dist(self) -> float:
        if self.curr_pos is None or not self.wpts:
            return math.inf
        cx, cy = self.curr_pos
        return min(math.hypot(cx - x, cy - y) for x, y in self.wpts)

    # ------------------------------------------------------------------ 루프 로직
    def _slow_loop(self):
        """5 m 안으로 들어오면 fast 루프로 전환"""
        if self._closest_dist() <= FAR_THRESH:
            self.slow_timer.cancel()
            self.fast_timer.reset()

    def _fast_loop(self):
        """진입·탈출 판정 + 상태 전환"""
        dist = self._closest_dist()
        if dist > FAR_THRESH:
            # 다시 멀어짐 → slow 루프로 복귀
            self.fast_timer.cancel()
            self.slow_timer.reset()
            return

        in_now = (dist <= IN_THRESH)
        if in_now and not self.in_prev:
            self._on_entry()
        elif not in_now and self.in_prev:
            self._on_exit()

        self.in_prev = in_now

    # ------------------------------------------------------------------ 상태 전환
    def _on_entry(self):
        self.get_logger().info('▶ ENTRY: stop & road layer OFF')
        self._set_speed(0.0)
        self._toggle_road_layer(False)  # 끄기

        # 안전판단 서비스(무한 루프 내부 3 s 주기) 비동기 호출
        fut = self.detect_cli.call_async(Trigger.Request())
        fut.add_done_callback(self._on_detect_done)

    def _on_exit(self):
        # SAFE 응답이 왔든 안 왔든, 경계선 벗어나면 레이어 ON 및 속도 해제
        self.get_logger().info('↩ EXIT: restore road layer ON')
        self._set_speed(float('nan'))   # speed limit 해제
        self._toggle_road_layer(True)   # 켜기

    def _on_detect_done(self, fut):
        """SAFE 응답 시 속도만 해제, road layer는 계속 OFF (횡단보도 끝날 때까지)"""
        try:
            safe = fut.result().success
        except Exception as e:
            self.get_logger().error(f'detect_crosswalk call failed: {e}')
            return

        if safe:
            self.get_logger().info('SAFE: resume but keep road layer OFF')
            self._set_speed(float('nan'))
            # road layer ON 은 _on_exit() 에서 처리
        else:
            # 이 노드는 False 응답이 오지 않는 구조이지만 예외에 대비
            self.get_logger().warn('UNSAFE: keeping stop')

    # ------------------------------------------------------------------ 서비스 호출 헬퍼
    def _set_speed(self, speed_val: float):
        """speed_val = 0.0 ➜ 제한 / NaN ➜ 제한 해제 (Nav2 규칙)"""
        req = SetSpeedLimit.Request(speed_limit=speed_val, percentage=False)
        self.speed_cli.call_async(req)

    def _toggle_road_layer(self, enable: bool):
        """road_obstacle_layer.enabled 파라미터 토글 + costmap 클리어"""
        param = Parameter(
            name='road_obstacle_layer.enabled',
            value=ParameterValue(
                type=ParameterType.PARAMETER_BOOL,
                bool_value=enable
            )
        )
        self.param_cli.call_async(SetParameters.Request(parameters=[param]))
        # 파라미터 반영 직후 costmap 비워서 즉시 적용
        self.clear_cli.call_async(ClearEntireCostmap.Request())

# ---------------------------------------------------------------------- main
def main():
    rclpy.init()
    rclpy.spin(CrosswalkSupervisor())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
