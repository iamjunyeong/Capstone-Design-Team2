#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from std_msgs.msg import Int8
from std_srvs.srv import Trigger

class CrosswalkService(Node):
    """
    /obstacle_crosswalk_info (Int8) 를 구독하여
    서비스 요청 시점부터 3 초 동안 값을 관찰,
    0이 한 번이라도 나오면 다시 3초 관찰,
    최종적으로 1만 관찰되면 "1" 을 응답.
    """

    def __init__(self):
        super().__init__('crosswalk_service_server')

        # 최근 수신 값을 저장 (초기값 1)
        self.latest_code = 1
        self.create_subscription(
            Int8,
            '/obstacle_crosswalk_info',
            self._info_cb,
            10
        )
        self.srv = self.create_service(
            Trigger,
            'detect_crosswalk',
            self.handle_detect_crosswalk
        )
        self.get_logger().info('Crosswalk service ready')

    def _info_cb(self, msg: Int8):
        self.latest_code = msg.data         # 0 또는 1

    def handle_detect_crosswalk(self, request, response):
        timeout = Duration(seconds=3)   # int형 seconds 사용

        while True:
            self.get_logger().info('3초간 관찰 시작')
            start = self.get_clock().now()
            saw_zero = False

            # 3초 관찰
            while self.get_clock().now() - start < timeout:
                rclpy.spin_once(self, timeout_sec=0.1)
                if self.latest_code == 0:
                    saw_zero = True
                    self.get_logger().info('0 감지 → 다시 관찰')
                    break

            # 3초 내내 1만 나왔으면 루프 종료
            if not saw_zero:
                self.get_logger().info('3초간 모두 1 → 서비스 응답')
                response.success = True
                response.message = '1'
                return response
            # saw_zero=True 면 while True가 다시 반복되어 관찰 재시작


def main():
    rclpy.init()
    node = CrosswalkService()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
