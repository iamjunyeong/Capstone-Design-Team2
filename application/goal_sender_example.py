import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import time

class Nav2Client(Node):
    def __init__(self):
        super().__init__('nav2_action_client')
        self._client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self._last_feedback_time = 0  # 마지막 출력 시각 저장용

    def send_goal(self):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = 0.0
        goal_msg.pose.pose.position.y = 0.0
        goal_msg.pose.pose.orientation.w = 1.0

        self._client.wait_for_server()

        self.get_logger().info('Sending goal to /navigate_to_pose...')
        self._send_goal_future = self._client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal was rejected ❌')
            return

        self.get_logger().info('Goal accepted ✅')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        current_time = time.time()

        # 2초가 지났을 때만 출력
        if current_time - self._last_feedback_time >= 2.0:
            self._last_feedback_time = current_time

            self.get_logger().info(f'📍[Feedback]')
            self.get_logger().info(f'  - Distance remaining: {feedback.distance_remaining:.2f}m')
            self.get_logger().info(f'  - Current pose: (x={feedback.current_pose.pose.position.x:.2f}, y={feedback.current_pose.pose.position.y:.2f})')
            self.get_logger().info(f'  - Navigation time: {feedback.navigation_time}')
            self.get_logger().info(f'  - Recoveries: {feedback.number_of_recoveries}')
            self.get_logger().info('----------------------------')

    def result_callback(self, future):
        result = future.result().result
        status = future.result().status
        self.get_logger().info(f'🚩Goal result received. Status: {status}')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    client = Nav2Client()
    client.send_goal()
    rclpy.spin(client)

if __name__ == '__main__':
    main()
