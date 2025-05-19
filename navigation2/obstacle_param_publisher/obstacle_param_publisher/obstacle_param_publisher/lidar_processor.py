import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
import math

class LidarProcessor(Node):
    def __init__(self):
        super().__init__('lidar_processor')
        self.sub = self.create_subscription(
            LaserScan,
            '/scan',  # LiDAR 토픽 (시뮬레이션: /scan, 실제: /lidar/scan)
            self.scan_callback,
            10
        )
        self.pub = self.create_publisher(Float32, '/lidar_distance', 10)
        self.get_logger().info("LiDAR 전방 90도 감시 시작")

    def scan_callback(self, msg):
        # 전방 ±45도(90도) 범위 계산
        angle_min = msg.angle_min
        angle_max = msg.angle_max
        angle_increment = msg.angle_increment
        
        # 시작/종료 각도 (라디안)
        start_angle = -math.pi/4  # -45도
        end_angle = math.pi/4     # +45도
        
        # 인덱스 계산
        start_idx = int((start_angle - angle_min) / angle_increment)
        end_idx = int((end_angle - angle_min) / angle_increment)
        
        # 범위 내 거리 추출 (유효값만)
        valid_ranges = [
            r for r in msg.ranges[start_idx:end_idx+1] 
            if msg.range_min < r < msg.range_max
        ]
        
        # 최소 거리 계산 (장애물 없으면 100m)
        min_distance = min(valid_ranges) if valid_ranges else 100.0
        
        # 거리 발행
        self.pub.publish(Float32(data=min_distance))

def main(args=None):
    rclpy.init(args=args)
    node = LidarProcessor()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
