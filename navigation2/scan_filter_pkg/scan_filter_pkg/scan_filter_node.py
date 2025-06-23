3#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class ScanFilterNode(Node):
    def __init__(self):
        super().__init__('scan_filter_node')
        # 구독 토픽
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan_input',
            self.scan_callback,
            10)
        # 발행(퍼블리시) 토픽
        self.publisher = self.create_publisher(
            LaserScan,
            '/scan_output',
            10)

        # 필터 각도 (도 → 라디안)
        self.angle_min_deg = -75.0
        self.angle_max_deg =  75.0
        self.angle_min_rad = math.radians(self.angle_min_deg)
        self.angle_max_rad = math.radians(self.angle_max_deg)

        self.get_logger().info(f"{self.angle_min_deg}° ~ {self.angle_max_deg}° 구간을 비활성화합니다.")

    def scan_callback(self, msg: LaserScan):
        # msg 복사를 위한 새로운 메시지 생성
        filtered = LaserScan()
        filtered.header          = msg.header
        filtered.angle_min       = msg.angle_min
        filtered.angle_max       = msg.angle_max
        filtered.angle_increment = msg.angle_increment
        filtered.time_increment  = msg.time_increment
        filtered.scan_time       = msg.scan_time
        filtered.range_min       = msg.range_min
        filtered.range_max       = msg.range_max

        # ranges, intensities 리스트 복사
        filtered.ranges     = list(msg.ranges)
        filtered.intensities= list(msg.intensities) if msg.intensities else []

        # 각도별로 검사하여 –60°~+60° 구간은 무한대로 설정
        for i in range(len(filtered.ranges)):
            angle = filtered.angle_min + i * filtered.angle_increment
            if self.angle_min_rad <= angle <= self.angle_max_rad:
                filtered.ranges[i] = float('inf')

        # 결과 발행
        self.publisher.publish(filtered)


def main(args=None):
    rclpy.init(args=args)
    node = ScanFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
