#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler

class OdomTfPublisher(Node):
    def __init__(self):
        super().__init__('odom_tf_publisher')
        # TransformBroadcaster 생성
        self.br = TransformBroadcaster(self)
        # 시작 시간 기록
        self.start_time = self.get_clock().now()

        # 매 0.1초마다 timer_callback 호출 → 10 Hz
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        now = self.get_clock().now()
        # 시작 시점으로부터 경과 시간 (초 단위)
        elapsed = (now - self.start_time).nanoseconds * 1e-9
        # 속도 0.2 m/s로 전진했을 때의 거리
        distance = 0.2 * elapsed

        # TransformStamped 메시지 구성
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'map'       # 부모 프레임
        t.child_frame_id = 'odom'       # 자식 프레임
        t.transform.translation.x = distance
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        # 회전은 고정 (yaw=0)
        q = quaternion_from_euler(0, 0, 0)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # TF 발행
        self.br.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = OdomTfPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
