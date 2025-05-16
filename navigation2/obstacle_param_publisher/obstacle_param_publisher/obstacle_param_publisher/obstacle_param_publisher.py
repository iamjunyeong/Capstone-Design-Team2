import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter  # ✅ 추가

class ObstacleParamPublisher(Node):
    def __init__(self):
        super().__init__('obstacle_param_publisher')
        self.declare_parameter('obstacle_state', 0)
        self.declare_parameter('obstacle_distance', 100.0)
        self.timer = self.create_timer(0.1, self.update_params)

    def update_params(self):
        # Parameter 객체 생성 후 리스트로 전달
        params = [
            Parameter('obstacle_state', Parameter.Type.INTEGER, 2),
            Parameter('obstacle_distance', Parameter.Type.DOUBLE, 3.0)
        ]
        self.set_parameters([
            Parameter('/controller_server/FollowPath.ObstacleSpeedCritic.obstacle_state', Parameter.Type.INTEGER, 2),
            Parameter('/controller_server/FollowPath.ObstacleSpeedCritic.obstacle_distance', Parameter.Type.DOUBLE, 3.0)
        ])

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleParamPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
