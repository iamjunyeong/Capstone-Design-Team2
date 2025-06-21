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
        while True:
            self.get_logger().info('Start 3-second observation')
            # 3초간 값 관찰
            start = self.get_clock().now()
            timeout = Duration(seconds=2.0)
            saw_zero = False
            while self.get_clock().now() - start < timeout:
                rclpy.spin_once(self, timeout_sec=0.1)
                if self.latest_code == 0:
                    saw_zero = True
                    break

            if saw_zero:
                self.get_logger().info('0 detected → retry')
                continue            # 다시 3초 관찰
            # 여기까지 왔으면 3초 동안 모두 1
            response.success = True
            response.message = '1'
            self.get_logger().info('All clear → return 1')
            return response

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
