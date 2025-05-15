import rclpy
from rclpy.node import Node

# std_msgs/UInt8: 음성 인식 노드에서 전달하는 건물 ID (정수)
from std_msgs.msg import UInt8
# geometry_msgs/PoseStamped: 목표 위치와 방향 정보를 담는 메시지 타입
from geometry_msgs.msg import PoseStamped
# Nav2 액션 메시지: NavigateToPose 액션 인터페이스를 사용함
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from hmi_interface.srv import IntentToPlanning
import time
# 미리 정의된 건물 좌표 데이터베이스 (건물 ID에 따른 좌표 및 방향 정보)

BUILDING_DB = {
    0: {"x": 0.0, "y": 0.0, "orientation": (0.0, 0.0, 0.0, 1.0), "building":"신공학관"}, #신공학관 
    3: {"x": 1.789409, "y": 45.007426, "orientation": (0.0, 0.0, 0.0, 1.0), "building": "공학관"}, #공대 c동 
    4: {"x": -60.514286, "y": 126.617229, "orientation": (0.0, 0.0, 0.0, 1.0) , "building":"공학관"}, #공대 A동
    8: {"x": -142.485558, "y": 192.364434, "orientation": (0.0, 0.0, 0.0, 1.0), "building":"학생회관"}, #학생회관
    9: {"x": -193.356556, "y": 198.505700, "orientation": (0.0, 0.0, 0.0, 1.0), "building":"청심대"}, #청심대
    11: {"x": -309.681705, "y": 114.077176, "orientation": (0.0, 0.0, 0.0, 1.0), "building":"법학관"}, #법학관
    15: {"x": -443.098181, "y": -142.948562, "orientation": (0.0, 0.0, 0.0, 1.0), "building":"수의학관"}, #수의대
    18: {"x": -439.816741, "y": -142.948562, "orientation": (0.0, 0.0, 0.0, 1.0), "building":"동물생명과학관"}, #동생대
    20: {"x": -487.130077, "y": -45.599748, "orientation": (0.0, 0.0, 0.0, 1.0), "building":"입학정보관"} #입학정보관
}
ESTIMATED_TIME_GAIN = 1.2

class GoalSender(Node):
    def __init__(self):
        super().__init__('hmi_planning_node')

        # /voice/building_id 토픽에서 UInt8 메시지를 구독하여 건물 ID를 수신
        self.subscription = self.create_subscription(UInt8,'/voice/building_id',self.building_id_callback, 10)
        # 구독자가 사용되지 않는 경우 경고가 발생하는 것을 방지하기 위한 참조

        # NavigateToPose 액션 클라이언트 생성
        # "navigate_to_pose"라는 이름의 액션 서버에 연결한다.
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.planning_feedback = NavigateToPose.Feedback()

        self.intent_server = self.create_service(IntentToPlanning, '/intent_to_planning', self.intent_server_callback)
        
        self.get_logger().info("hmi_planning started.")

    def building_id_callback(self, msg: UInt8):
        """
        /voice/building_id 토픽으로부터 건물 ID를 수신하는 콜백 함수.
        음성 인식 노드로부터 전달된 uint8 정수 값을 건물 ID로 해석하고,
        해당 건물의 좌표를 데이터베이스에서 조회 후, 목표 PoseStamped 메시지를 생성하여 액션 Goal을 전송.
        """
        building_id = msg.data
        self.get_logger().info(f"수신된 건물 ID: {building_id}")

        # 데이터베이스에 건물 ID가 존재하는지 확인
        if building_id not in BUILDING_DB:
            self.get_logger().error(f"존재하지 않는 건물 ID: {building_id}")
            return

        # 건물 ID에 해당하는 좌표 정보 조회
        coords = BUILDING_DB[building_id]
        # 조회된 좌표 값을 사용해 PoseStamped 메시지 생성
        goal_pose = self.create_goal_pose(coords)
        # 생성한 PoseStamped 메시지를 액션 Goal로 전송
        self.send_goal(goal_pose)

    def create_goal_pose(self, coords: dict) -> PoseStamped:
        """
        주어진 좌표 데이터를 바탕으로 geometry_msgs/PoseStamped 메시지를 생성하는 함수.

        입력:
          coords - 딕셔너리 형태로 'x', 'y', 및 'orientation' (튜플: x,y,z,w) 값을 포함.

        출력:
          생성된 PoseStamped 메시지
        """
        goal_pose = PoseStamped()
        # 현재 시간을 header.stamp에 기록
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        # 지도 좌표계 프레임으로 설정
        goal_pose.header.frame_id = "map"
        # 위치 값 설정
        goal_pose.pose.position.x = coords["x"]
        goal_pose.pose.position.y = coords["y"]
        goal_pose.pose.position.z = 0.0  # 2D 환경에서는 보통 0
        # 쿼터니언 orientation 값 설정 (튜플 형태로 저장된 값을 unpack)
        ox, oy, oz, ow = coords["orientation"]
        goal_pose.pose.orientation.x = ox
        goal_pose.pose.orientation.y = oy
        goal_pose.pose.orientation.z = oz
        goal_pose.pose.orientation.w = ow
        
        self.get_logger().info(f"생성된 목표 Pose: {goal_pose.pose}")
        return goal_pose

    def send_goal(self, pose: PoseStamped):
        """
        생성된 PoseStamped 메시지를 NavigateToPose 액션 Goal에 담아 전송하는 함수.

        입력:
          pose - 생성된 geometry_msgs/PoseStamped 메시지.
        """
        # NavigateToPose 액션 Goal 메시지 생성
        goal_msg = NavigateToPose.Goal()
        # Goal 메시지 내의 pose 필드에 전달받은 PoseStamped 메시지 할당
        goal_msg.pose = pose

        self.get_logger().info("액션 서버 접속을 위해 대기 중...")
        # 액션 서버가 준비될 때까지 대기
        self._action_client.wait_for_server()

        self.get_logger().info("NavigateToPose 액션 Goal 전송 중...")
        # 비동기 방식으로 액션 Goal 전송 및 결과 처리 콜백 설정
        send_goal_future = self._action_client.send_goal_async(goal_msg,feedback_callback=self.feedback_callback)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """
        액션 서버의 Goal 응답을 처리하는 콜백 함수.
        Goal이 거부되었는지 여부를 판단하고, 승인된 경우 결과 반환을 위한 콜백을 설정.
        #액션의 response만 받는것, feedback 토픽으로 받을 수 있음. 
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal이 거부되었습니다.')
            return

        self.get_logger().info('Goal이 승인되었습니다.')
        # 액션 서버로부터 결과를 비동기적으로 받아오기 위한 후속 콜백 설정
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)
    
    def feedback_callback(self, feedback_msg):
            self.feedback = feedback_msg.feedback
            current_time = time.time()

            # 2초가 지났을 때만 출력
            if current_time - self._last_feedback_time >= 2.0:
                self._last_feedback_time = current_time

                self.get_logger().info(f'📍[Feedback]')
                self.get_logger().info(f'  - Distance remaining: {self.feedback.distance_remaining:.2f}m')
                self.get_logger().info(f'  - Current pose: (x={self.feedback.current_pose.pose.position.x:.2f}, y={self.feedback.current_pose.pose.position.y:.2f})')
                self.get_logger().info(f'  - Navigation time: {self.feedback.navigation_time}')
                self.get_logger().info(f'  - Recoveries: {self.feedback.number_of_recoveries}')
                self.get_logger().info(f'  - estimated time remaining: {self.feedback.estimated_time_remaining.sec:.2f}m')
                self.get_logger().info('----------------------------')

    def get_result_callback(self, future):
        """
        액션 결과를 수신하는 콜백 함수.
        결과 메시지에서 필요한 정보를 로거로 출력.
        """
        result = future.result().result
        self.get_logger().info(f'Navigation 완료: {result}')
        # 추가 후속 처리나 오류 핸들링을 구현할 수 있음.
    
    def intent_server_callback(self, request, response):
        """
        IntentToPlanning 서비스 요청을 처리하는 콜백 함수.
        요청 intent에 따라 가장 가까운 건물(랜드마크)과 남은 시간 계산 결과를 응답에 포함시킨다.
        """
        if request.intent == "get_eta":
            # 남은 시간: estimated_time_remaining (Duration.sec)
            total_seconds = self.feedback.estimated_time_remaining.sec
            minutes = int(ESTIMATED_TIME_GAIN * total_seconds / 60)
            response.estimated_time_remaining =  minutes
            response.closest_landmark = "unknown"  # ETA 요청 시 랜드마크 정보는 필요 없음

        elif response.intent == "get_location":
            # 현재 위치
            current_x = self.current_pose.pose.position.x
            current_y = self.current_pose.pose.position.y

            # 가장 가까운 랜드마크 계산
            min_distance = float('inf')
            closest_landmark = "알 수 없음"

            for info in BUILDING_DB.values():
                x, y = info["x"], info["y"]
                dist = ((current_x - x)**2 + (current_y - y)**2) ** 0.5
                if dist < min_distance:
                    min_distance = dist
                    closest_landmark = info["building"]

            response.closest_landmark = closest_landmark
            response.estimated_time_remaining = 0
        
        # 기본적으로 요청 intent 그대로 반환
        response.intent = request.intent

        return response

def main(args=None):
    """
    노드를 초기화하고, GoalSender 노드를 실행하는 메인 함수.
    rclpy.init()을 통해 ROS2 노드 시스템을 초기화하고, spin()을 호출하여 콜백 함수를 지속적으로 처리.
    """
    rclpy.init(args=args)
    node = GoalSender()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
