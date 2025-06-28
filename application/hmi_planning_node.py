import rclpy
from rclpy.node import Node

# std_msgs/UInt8: 음성 인식 노드에서 전달하는 건물 ID (정수)
from std_msgs.msg import UInt8, Bool
# geometry_msgs/PoseStamped: 목표 위치와 방향 정보를 담는 메시지 타입
from geometry_msgs.msg import PoseStamped
# Nav2 액션 메시지: NavigateToPose 액션 인터페이스를 사용함
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from hmi_interface.srv import IntentToPlanning
from hmi_interface.msg import IntentToPlanningmsg
import time
# 미리 정의된 건물 좌표 데이터베이스 (건물 ID에 따른 좌표 및 방향 정보)
"""
        1. 신공학관
        3. 공대 C동
        5. 공대 A동 (시작 지점)
        8. 학생회관
        9. 청심대
        11. 법학관
        15. 수의대
        18. 동생대
        20. 입학정보관

"""
BUILDING_DB = {
    1: {"x": 63.4208470000303, "y": -126.286642000079, "orientation": (0.0, 0.0, 0.795833, 0.605516)},
    3: {"x": 65.2102560000494, "y": -81.2792159998789, "orientation": (0.0, 0.0, 0.0, 1.0)},
    # 5: {"x": 0.0, "y": 0.0, "orientation": (0.0, 0.0, 0.0, 1.0)},
    5: {"x": 39.8891853134264, "y": -72.1832431419753, "orientation": (0.0, 0.0, -0.752556,0.658528)},
    # 8: {"x": -79.0647109999554, "y": 66.0777920000255, "orientation": (0.0, 0.0, 0.999373,0.035408)},
    8: {"x": 0.0, "y": 0.0, "orientation": (0.0, 0.0, 0.930787,0.365561)},
    9: {"x": -129.935708999983, "y": 72.219058000017, "orientation": (0.0, 0.0, 0.998196, 0.060034)},
    11: {"x": -246.260857999965, "y": -12.2094660000876, "orientation": (0.0, 0.0, -0.836539, 0.547907)},
    15: {"x": -379.677333999949, "y": -269.235204000026, "orientation": (0.0, 0.0, -0.763272, 0.646078)},
    18: {"x": -376.395893999957, "y": -169.805312000215, "orientation": (0.0, 0.0, 0.572659, 0.819793)},
    20: {"x": -423.709229999979, "y": -171.886390000116, "orientation": (0.0, 0.0, 0.968174, 0.250278)}


}
ESTIMATED_TIME_GAIN = 1.2

class GoalSender(Node):
    def __init__(self):
        super().__init__('hmi_planning_node')

        # /voice/building_id 토픽에서 UInt8 메시지를 구독하여 건물 ID를 수신
        self.subscription = self.create_subscription(IntentToPlanningmsg,'/voice/building_id',self.building_id_callback, 10)
        # 구독자가 사용되지 않는 경우 경고가 발생하는 것을 방지하기 위한 참조

        # NavigateToPose 액션 클라이언트 생성
        # "navigate_to_pose"라는 이름의 액션 서버에 연결한다.
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.planning_feedback = NavigateToPose.Feedback()
        self._current_goal_handle = None
        self.intent_server = self.create_service(IntentToPlanning, '/intent_to_planning', self.intent_server_callback)
        self.handlebutton_sub = self.create_subscription(UInt8, '/handlebutton_state', self.handlebutton_callback, 10)  # 핸들 버튼 상태 구독
        self.intent_client_to_intent_node = self.create_client(IntentToPlanning, '/intent_to_planning')
        self.heartbeat_pub = self.create_publisher(UInt8, '/heartbeat/hmi_planning_node', 10)  # heartbeat 퍼블리셔
        self.arrived_pub = self.create_publisher(Bool, '/arrived_at_destination', 10)
        self._last_feedback_time = None
        self._first_feedback_time = None 

        self.heartbeat = 0
        self.handlebutton_state = 0 
        self.target_pose = None  # 목표 위치를 저장할 변수
        self.intent = None  # 수신된 intent를 저장할 변수
        self.wait_for_handle_grab = False  # 핸들 버튼을 기다리는 상태를 나타내는 변수
        ##########################
        self.feedback = None
        ##########################
        #테스트용
        #self.go = False 
        #self.go_pub = self.create_publisher(Bool, '/go', 10)  # go 퍼블리셔
        
        self.get_logger().info("hmi_planning started.")
        self.create_timer(1.0, self.pub_heartbeat)  # 1초마다 타이머 콜백 호출
        # 생성자 내부에 타이머 추가
        self.create_timer(0.5, self.wait_for_handle_and_send_goal)
    
    def wait_for_handle_and_send_goal(self):
        if getattr(self, 'waiting_for_handle_grab', False):
            if self.handlebutton_state == 1:
                self.send_goal(self.target_pose)
                self._first_feedback_time = time.time()
                self._last_feedback_time = time.time()
                self.get_logger().info("목적지 설정 완료")
                self.waiting_for_handle_grab = False
                self.heartbeat = 1  # 목표 설정 후 heartbeat를 1로 설정
                
    def building_id_callback(self, msg: IntentToPlanningmsg):
        """
        /voice/building_id 토픽으로부터 intent와 building_id를 수신하는 콜백 함수.
        intent에 따라 목적지를 설정하거나 기존 goal을 취소하고 새 goal을 설정함.
        """
        self.intent = msg.intent
        building_id = msg.building_id

        self.get_logger().info(f"수신된 intent: {self.intent}, building_id: {building_id}")

        if building_id not in BUILDING_DB:
            self.get_logger().error(f"존재하지 않는 건물 ID: {building_id}")
            return
        self.go = True
        self.target_pose = self.create_goal_pose(BUILDING_DB[building_id])
       
        if self.intent in ("set_destination_confirmed", "change_dst_confirmed"):
            if self.intent == "change_dst_confirmed":
                self.cancel_current_goal()
            self.waiting_for_handle_grab = True  # ✅ handlebutton 대기 시작
            self.get_logger().info("손잡이 잡기를 대기합니다.")

    def handlebutton_callback(self, msg):
        self.handlebutton_state = msg.data
    # def pub_stopandgo(self):
    #     msg = Bool()
    #     msg.data = self.go
    #     self.go_pub.publish(msg)

    def pub_heartbeat(self):
        msg = UInt8()
        msg.data = self.heartbeat
        self.heartbeat_pub.publish(msg)
        
        # self.pub_stopandgo()
    

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
        self.heartbeat = 0
        self._action_client.wait_for_server()
        

        
        # 비동기 방식으로 액션 Goal 전송 및 결과 처리 콜백 설정
        send_goal_future = self._action_client.send_goal_async(goal_msg,feedback_callback=self.feedback_callback)
        send_goal_future.add_done_callback(self.goal_response_callback)
        self.get_logger().info("NavigateToPose 액션 Goal 전송 중...")
        
    def cancel_current_goal(self):
        if self._current_goal_handle is not None:
            self.get_logger().info("replanning, 현재 goal 취소")
            future = self._current_goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, future)

    
    
    def goal_response_callback(self, future):
        """
        액션 서버의 Goal 응답을 처리하는 콜백 함수.
        Goal이 거부되었는지 여부를 판단하고, 승인된 경우 결과 반환을 위한 콜백을 설정.
        #액션의 response만 받는것, feedback 토픽으로 받을 수 있음. 
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal이 거부되었습니다.')
            self.heartbeat = 0
            return

        self.get_logger().info('Goal이 승인되었습니다.')
        # 액션 서버로부터 결과를 비동기적으로 받아오기 위한 후속 콜백 설정
        self._current_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        self.heartbeat = 1
        result_future.add_done_callback(self.get_result_callback)
        
   
    def feedback_callback(self, feedback_msg):
            self.feedback = feedback_msg.feedback
            current_time = time.time()
            
            # 2초가 지났을 때만 출력
            if current_time - self._first_feedback_time >= 10.0:
                if self.feedback is not None:
                    if (self.feedback.distance_remaining<1.0):
                        msg = Bool()
                        msg.data = True
                        self.arrived_pub.publish(msg)
                        self.get_logger().info("목적지 도착 알림 토픽 발행 완료.")

            if current_time - self._last_feedback_time >= 2.0:
                self._last_feedback_time = current_time

                self.get_logger().info(f'[Feedback]')
                self.get_logger().info(f'  - Distance remaining: {self.feedback.distance_remaining:.2f}m')
                self.get_logger().info(f'  - Current pose: (x={self.feedback.current_pose.pose.position.x:.2f}, y={self.feedback.current_pose.pose.position.y:.2f})')
                self.get_logger().info(f'  - Navigation time: {self.feedback.navigation_time}')
                self.get_logger().info(f'  - Recoveries: {self.feedback.number_of_recoveries}')
                self.get_logger().info(f'  - estimated time remaining: {self.feedback.estimated_time_remaining.sec:.2f}m')
                self.get_logger().info('----------------------------')
            
    def get_result_callback(self, future):
        result = future.result().result
        status = result.result  # 상태 코드
#############################
        # if status == 4:
        #     self.get_logger().info("✅ 목적지 도착 (SUCCEEDED)")

        #     msg = Bool()
        #     msg.data = True
        #     self.arrived_pub.publish(msg)
        #     self.get_logger().info("목적지 도착 알림 토픽 발행 완료.")

#############################

        if status == 6:
            self.get_logger().error("❌ 탐색 실패 (ABORTED)")
            # 여기서 알림 전송!!! 
            # and replanning 
            if self.target_pose is not None:
                
                self.send_goal(self.target_pose)  # 재탐색을 위해 현재 목표 위치로 다시 Goal 전송
            # 실패 알림 전송 가능
            

        else:
            self.get_logger().warn(f"⚠️ 미확인 상태 코드: {status}")

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
