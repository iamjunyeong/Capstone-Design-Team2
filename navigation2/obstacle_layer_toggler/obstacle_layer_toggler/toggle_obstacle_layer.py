#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from std_msgs.msg import Int8
from nav2_msgs.srv import ClearEntireCostmap

class ObstacleLayerToggler(Node):
    def __init__(self):
        super().__init__('obstacle_layer_toggler')
        self.get_logger().info('Obstacle Layer Toggler 노드가 시작되었습니다.')

        # 서비스 콜백 그룹 분리
        self.param_cb_group = MutuallyExclusiveCallbackGroup()
        self.clear_cb_group = MutuallyExclusiveCallbackGroup()

        # dynamic_obstacle_layer 파라미터 토글용 클라이언트
        self._param_client = self.create_client(
            SetParameters,
            '/local_costmap/local_costmap/set_parameters',
            callback_group=self.param_cb_group
        )
        # 전체 Costmap 초기화용 클라이언트
        self._clear_cli = self.create_client(
            ClearEntireCostmap,
            '/local_costmap/clear_entirely_local_costmap',
            callback_group=self.clear_cb_group
        )

        # 서비스 준비를 기다리기 위한 타이머
        self._ready_timer = self.create_timer(0.5, self._check_services_ready)
        self._services_ready = False

        # obstacle_info 구독
        self._sub = self.create_subscription(
            Int8, '/obstacle_info', self._on_info, 10
        )
        # 현재 dynamic layer 상태 기억
        self._dyn_enabled = True

        # 콜백 과도 실행 방지용
        self._last_time = self.get_clock().now()
        self._min_interval = 0.1  # 100ms

    def _check_services_ready(self):
        if not self._services_ready:
            if self._param_client.service_is_ready() and self._clear_cli.service_is_ready():
                self._services_ready = True
                self._ready_timer.cancel()
                self.get_logger().info('모든 서비스가 준비되었습니다.')

    def _on_info(self, msg: Int8):
        now = self.get_clock().now()
        if (now - self._last_time).nanoseconds * 1e-9 < self._min_interval:
            return
        self._last_time = now

        v = msg.data
        # 0,1 → 켜기, 2,3 → 끄기
        want = (v in [0,1])
        if want != self._dyn_enabled:
            self._toggle_dynamic(want)
            self._dyn_enabled = want
            st = '활성화' if want else '비활성화'
            self.get_logger().info(f'obstacle_info={v}: dynamic_obstacle_layer {st}')
        elif v not in [0,1,2,3]:
            self.get_logger().warn(f'알 수 없는 obstacle_info 값: {v}')

    def _toggle_dynamic(self, enable: bool):
        # YAML 에 정의된 dynamic_obstacle_layer.enabled 토글
        p = Parameter(
            name='dynamic_obstacle_layer.enabled',
            value=ParameterValue(
                type=ParameterType.PARAMETER_BOOL,
                bool_value=enable
            )
        )
        req = SetParameters.Request(parameters=[p])
        fut = self._param_client.call_async(req)
        fut.add_done_callback(self._on_params_set)

    def _on_params_set(self, future):
        try:
            res = future.result()
            if all(r.successful for r in res.results):
                self.get_logger().debug('dynamic_obstacle_layer 파라미터 갱신 OK → Costmap 클리어')
                # 설정이 반영된 뒤 전체 Costmap 한 번 비우기
                clear_req = ClearEntireCostmap.Request()
                self._clear_cli.call_async(clear_req)
            else:
                self.get_logger().error(f'파라미터 설정 실패: {res.results}')
        except Exception as e:
            self.get_logger().error(f'파라미터 서비스 에러: {e}')

def main():
    rclpy.init()
    node = ObstacleLayerToggler()
    exec = MultiThreadedExecutor()
    exec.add_node(node)
    try:
        exec.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__=='__main__':
    main()

