import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters
from std_msgs.msg import Int8, Float32

class ObstacleIntegrationNode(Node):
    def __init__(self):
        super().__init__('obstacle_integration_node')
        
        # 상태 초기화
        self.state = 0      # 비전 상태 (0-3)
        self.distance = 3.0 # LiDAR 거리 (최소 3m)
        
        # 서비스 클라이언트
        self.param_client = self.create_client(SetParameters, '/controller_server/set_parameters')
        while not self.param_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('컨트롤러 서버 연결 대기...')
        
        # 구독 설정
        self.create_subscription(Int8, '/obstacle_info', self.vision_cb, 10)
        self.create_subscription(Float32, '/lidar_distance', self.lidar_cb, 10)

    def vision_cb(self, msg):
        self.state = msg.data
        self.update_params()

    def lidar_cb(self, msg):
        self.distance = max(0.5, msg.data)  # 최소 0.5m 보장
        self.update_params()

    def update_params(self):
        req = SetParameters.Request()
        req.parameters = [
            Parameter(
                name='FollowPath.ObstacleSpeedCritic.obstacle_state', 
                value=self.state
            ).to_parameter_msg(),  # ✅ 핵심 수정 부분
            Parameter(
                name='FollowPath.ObstacleSpeedCritic.obstacle_distance', 
                value=self.distance
            ).to_parameter_msg()   # ✅ 핵심 수정 부분
        ]
        self.param_client.call_async(req)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleIntegrationNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
