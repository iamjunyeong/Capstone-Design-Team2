#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav2_msgs.srv import ClearEntireCostmap

class CostmapClearer(Node):
    def __init__(self):
        super().__init__('costmap_clearer')
        self.client = self.create_client(
            ClearEntireCostmap,
            '/local_costmap/clear_entirely_local_costmap')
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('CostmapClearer node started')

    def timer_callback(self):
        self.get_logger().info('💥 timer_callback fired')
        if not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('⏳ Clear service not available, retrying...')
            return

        self.get_logger().info('✅ Service is available, sending request')
        req = ClearEntireCostmap.Request()
        future = self.client.call_async(req)
        future.add_done_callback(self.on_clear_response)

    def on_clear_response(self, future):
        try:
            # future.result() 가 예외를 던지지 않으면 성공
            future.result()
            self.get_logger().info('🎉 Called clear_entirely_local_costmap')
        except Exception as e:
            self.get_logger().error(f'❌ Service call failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = CostmapClearer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
