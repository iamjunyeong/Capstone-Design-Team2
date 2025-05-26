import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import Float32

class HMIAnglePublisher(Node):
    def __init__(self):
        super().__init__('hmi_angle_publisher')
        # 퍼블리셔: std_msgs/Float32 @ /hmi_angle
        self.publisher_ = self.create_publisher(Float32, 'hmi_angle', 10)
        # 서브스크라이버: AckermannDrive @ /ackermann_cmd
        self.subscription = self.create_subscription(
            AckermannDrive,
            'ackermann_cmd',
            self.listener_callback,
            10)
        self.subscription  # unused variable 방지

    def listener_callback(self, msg: AckermannDrive):
        angle = Float32()
        angle.data = msg.steering_angle
        self.publisher_.publish(angle)
        self.get_logger().info(f'Published steering_angle: {angle.data:.3f}')

def main(args=None):
    rclpy.init(args=args)
    node = HMIAnglePublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
