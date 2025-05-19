import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32, UInt8

class ButtonNode(Node):
    def __init__(self):
        super().__init__('button_node')

        # 상태 추적 변수 초기화
        self.last_emergency_state = None
        self.last_stt_state = None
        self.last_tact_state = None

        # emergency_state: Bool 타입
        self.emergency_sub = self.create_subscription(Bool,'emergency_state',self.emergency_callback,10)
        # stt_button_state: Bool 타입
        self.stt_sub = self.create_subscription(Bool,'stt_button_state',self.stt_callback,10)
        # tact_switch_state: Int32
        self.tact_sub = self.create_subscription(Int32,'tact_switch_state',self.tact_callback,10)

        # 퍼블리셔 선언 (TTSNode 구독용)
        self.talkbutton_pub = self.create_publisher(Bool, '/talkbutton_pressed', 10)
        self.emergency_pub = self.create_publisher(Bool, '/emergency', 5)
        self.handlebutton_pub = self.create_publisher(UInt8, '/handlebutton_code', 5)
        self.hmi_stop_pub = self.create_publisher(Bool, '/hmi_stop', 10)

        self.handlebutton_ = self.create_publisher(UInt8, '/handlebutton_state', 10)

    def emergency_callback(self, msg):
            if msg.data:
                self.get_logger().info('[EMERGENCY] 비상정지 발생 (TRUE)')
            else:
                self.get_logger().warn('[EMERGENCY] 정상 상태 (FLASE)')
            self.last_emergency_state = msg.data  # 상태 갱신
            self.emergency_pub.publish(Bool(data=msg.data))

    def stt_callback(self, msg):
            if msg.data:
                self.get_logger().info('[STT] 버튼 눌림 감지됨 (True)')
            else:
                self.get_logger().info('[STT] 대기 상태(False)')
            self.last_stt_state = msg.data  # 상태 갱신
            self.talkbutton_pub.publish(Bool(data=msg.data))

    def tact_callback(self, msg):
        # `Int32`로 받은 데이터를 `UInt8`로 변환
        tact_state = msg.data
        tact_code = 0

        # `Int32` -> `UInt8` 변환
        if tact_state == 0:  # 상태가 0이면
            tact_code = 0
        elif tact_state == 1:  # 상태가 1이면
            tact_code = 1
        elif tact_state == 2:  # 상태가 2이면
            tact_code = 2
        else:
            tact_code = 0  # 예외 처리, 적절한 값으로 설정 (기본값: 0)

        if tact_code != self.last_tact_state:  # 상태가 바뀌었을 때만 출력
            if tact_code == 0:
                self.get_logger().warn('[TACT] 손잡이 해제 감지됨 >3초 (0)')
            elif tact_code == 1:
                self.get_logger().info('[TACT] 손잡이 정상 인식 (1)')
            else:
                self.get_logger().warn('[TACT] 상태 불명 (2)')

            self.last_tact_state = tact_code  # 상태 갱신
            self.handlebutton_pub.publish(UInt8(data=tact_code))  # 퍼블리시

def main(args=None):
    rclpy.init(args=args)
    node = ButtonNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
