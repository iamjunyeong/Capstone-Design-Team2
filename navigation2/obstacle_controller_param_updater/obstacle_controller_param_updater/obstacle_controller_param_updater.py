#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int8
from rcl_interfaces.srv import SetParameters


class ObstacleParamUpdater(Node):
    def __init__(self):
        super().__init__('obstacle_param_updater')

        # 1) 서비스 클라이언트 생성 (controller_server의 set_parameters)
        self.param_client = self.create_client(
            SetParameters,
            '/controller_server/set_parameters'
        )

        # 2) 토픽 구독자
        self.state = 0
        self.create_subscription(
            Int8,
            '/obstacle_info',
            self.obstacle_callback,
            10
        )

        self.distance = float('inf')
        self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

    def obstacle_callback(self, msg: Int8):
        """ /obstacle_info 값 (int8) 을 받아서 상태로 저장 """
        self.state = int(msg.data)
        self.get_logger().debug(f"obstacle_state ← {self.state}")

    def scan_callback(self, msg: LaserScan):
        """ LaserScan 중 –180°~–170°, 170°~180° 구간의 최소 유효 거리값을 계산 """
        # 라디안으로 변환
        start1 = -180.0 * math.pi / 180.0  # -π
        end1   = -170.0 * math.pi / 180.0  # -170°
        start2 =  170.0 * math.pi / 180.0  #  170°
        end2   =  180.0 * math.pi / 180.0  #  π

        # 인덱스 계산 함수
        def idx(angle):
            return int(round((angle - msg.angle_min) / msg.angle_increment))

        # 각 구간의 시작/끝 인덱스
        i1_start = max(0,   idx(start1))
        i1_end   = min(len(msg.ranges)-1, idx(end1))
        i2_start = max(0,   idx(start2))
        i2_end   = min(len(msg.ranges)-1, idx(end2))

        # 두 구간의 거리값 슬라이스
        window1 = msg.ranges[i1_start:i1_end+1]
        window2 = msg.ranges[i2_start:i2_end+1]

        # Inf, NaN 필터링
        valid1 = [r for r in window1 if r == r and r != float('inf')]
        valid2 = [r for r in window2 if r == r and r != float('inf')]

        # 두 구간 합쳐서 최소 거리 계산
        all_valid = valid1 + valid2
        self.distance = min(all_valid) if all_valid else float('inf')
        self.get_logger().debug(
            f"min_dist(–180°~–170°,170°~180°) = {self.distance:.2f} m"
        )

        # 파라미터 서비스 호출
        self.update_parameters()

    def update_parameters(self):
        # 서비스가 준비될 때까지 대기
        if not self.param_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("set_parameters 서비스 호출 실패")
            return

        # 요청 생성
        req = SetParameters.Request()
        req.parameters = [
            # obstacle_state ← /obstacle_info 토픽에서 받은 정수
            Parameter(
                name='FollowPath.ObstacleSpeedCritic.obstacle_state',
                value=self.state
            ).to_parameter_msg(),
            # obstacle_distance ← –6~+6° 최소 거리값
            Parameter(
                name='FollowPath.ObstacleSpeedCritic.obstacle_distance',
                value=self.distance
            ).to_parameter_msg()
        ]

        # 비동기 호출
        future = self.param_client.call_async(req)
        future.add_done_callback(self.parameter_set_callback)
        # self.get_logger().info(
        #     f"파라미터 설정 요청 → state={self.state}, distance={self.distance:.2f}"
        # )

    def parameter_set_callback(self, future):
        try:
            resp = future.result()
            # if all(r.successful for r in resp.results):
            #     self.get_logger().info("파라미터 설정 성공")
            # else:
            #     self.get_logger().error("파라미터 설정 중 일부 실패")
            if not all(r.successful for r in resp.results):
                self.get_logger().error("파라미터 설정 중 일부 실패")
        except Exception as e:
            self.get_logger().error(f"파라미터 설정 예외: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleParamUpdater()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
