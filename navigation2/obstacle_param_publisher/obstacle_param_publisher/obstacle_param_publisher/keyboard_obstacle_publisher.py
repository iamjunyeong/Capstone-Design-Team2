#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters
import sys, termios, tty, threading
import time
import functools

class KeyboardObstaclePublisher(Node):
    def __init__(self):
        super().__init__('keyboard_obstacle_publisher')
        self.state = 0
        self.distance = 6.0
        self.param_client = self.create_client(SetParameters, '/controller_server/set_parameters')
        
        self.get_logger().info('Press 0/1/2/3 for 상태, +/- for 거리, q to quit')
        self.get_logger().info('Waiting for /controller_server parameter service...')
        
        while not self.param_client.wait_for_service(timeout_sec=2.0):
            if not rclpy.ok():
                self.get_logger().error('Interrupted while waiting for the service. Exiting.')
                rclpy.try_shutdown()
                return
            self.get_logger().info('/controller_server service not available, waiting again...')
        
        self.get_logger().info('/controller_server parameter service is available.')
        self.update_parameters()

    def update_parameters(self):
        request = SetParameters.Request()
        request.parameters = [
            Parameter(name='FollowPath.ObstacleSpeedCritic.obstacle_state', value=self.state).to_parameter_msg(),
            Parameter(name='FollowPath.ObstacleSpeedCritic.obstacle_distance', value=self.distance).to_parameter_msg()
        ]
        future = self.param_client.call_async(request)
        future.add_done_callback(self.parameter_set_callback)
        self.get_logger().info(f"Requested to set state={self.state}, distance={self.distance:.1f}")

    def parameter_set_callback(self, future):
        try:
            response = future.result()
            if all([res.successful for res in response.results]):
                self.get_logger().info("Parameters set successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to set parameters: {e}")

    def run(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            print("Keyboard input enabled...")
            while rclpy.ok():
                c = sys.stdin.read(1)
                if c == 'q':
                    break
                elif c in ['0', '1', '2', '3']:
                    self.state = int(c)
                elif c == '+':
                    self.distance += 0.5
                elif c == '-':
                    self.distance = max(0.0, self.distance - 0.5)
                self.update_parameters()
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            print("Keyboard input disabled.")

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardObstaclePublisher()
    try:
        node.run()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()  # ✅ main() 함수 추가
