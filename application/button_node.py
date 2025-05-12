import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32, UInt8

class ButtonNode(Node):
    def __init__(self):
        super().__init__('button_node')

        # 상태 추적 변수 초기화
        self.last_emergency_state = None
        self.last_stt_state = None
        self.last_tact_state = -1

        # Arduino data 수신 
        self.emergency_sub = self.create_subscription(Bool,'/emergency_state',self.emergency_callback, 10)
        self.stt_sub = self.create_subscription(Bool,'/talk_button_state', self.stt_callback, 10)
        self.tact_sub = self.create_subscription(Int32,'/tact_switch_state', self.tact_callback, 10)

        # 퍼블리셔 선언 (TTSNode 구독용)
        self.talkbutton_pub = self.create_publisher(Bool, '/talkbutton_pressed', 10) # to STT and TTS 
        self.emergency_pub = self.create_publisher(Bool, '/emergency', 10) #이거 수행이형이 받음
        self.handlebutton_pub = self.create_publisher(UInt8, '/handlebutton_code', 10)
        self.hmi_stop_pub = self.create_publisher(Bool, '/hmi_stop', 10)

    def emergency_callback(self, msg):
        self.last_emergency_state = msg.data  # 상태 갱신
        if self.last_emergency_state:
            self.get_logger().warn('[EMERGENCY] 비상 정지 버튼 눌림 감지됨')
        elif not self.last_emergency_state:
            self.get_logger().info('[EMERGENCY] 정상 상태')
        
        self.emergency_pub.publish(Bool(data=msg.data))  #현재 비상정지 상태 퍼블리시. to TTS 및 CONTROL 모듈

    def stt_callback(self, msg):
        self.last_stt_state = msg.data
        self.get_logger().info(f'[STT] STT 버튼 상태: {self.last_stt_state}')
        self.talkbutton_pub.publish(Bool(data=msg.data))    

    def tact_callback(self, msg):
        # `Int32`로 받은 데이터를 `UInt8`로 변환
        tact_state = msg.data
        tact_code = 0

        # `Int32` -> `UInt8` 변환
        if tact_state in [0, 1, 2]:
            tact_code = tact_state
        else:
            tact_code = 0  # 예외 처리, 적절한 값으로 설정 (기본값: 0)

        if tact_code != self.last_tact_state:  # 상태가 바뀌었을 때만 출력
            if tact_code == 1:
                self.get_logger().warn('[TACT] 손잡이 해제 감지됨 >3초 (0)')
            elif tact_code == 2:
                self.get_logger().info('[TACT] 손잡이 정상 인식 (1)')
            elif tact_code == 0:
                self.get_logger().warn('[TACT] 다 떨어짐')

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

