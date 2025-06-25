import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32, UInt8
from hmi_interface.msg import Heartbeat
import datetime
class ButtonNode(Node):
    def __init__(self):
        super().__init__('button_node')

        # 상태 추적 변수 초기화
        self.last_emergency_state = None
        self.last_stt_state = None
        self.last_tact_state = None
        self.hmi_stop_state = False  # HMI stop 상태 초기화
        self.tact_state = 0
        # 각 노드 heartbeat 상태 초기화 
        self.stt_hb_state = 0 
        self.hpl_hb_state = 0


        # emergency_state: Bool 타입
        self.emergency_sub = self.create_subscription(Bool,'/emergency_state',self.emergency_callback,10)
        # stt_button_state: Bool 타입
        self.stt_sub = self.create_subscription(Bool,'/stt_button_state',self.stt_callback,10)
        # tact_switch_state: Int32
        self.tact_sub = self.create_subscription(Int32,'/tact_switch_state',self.tact_callback,10)

        #heartbeat 
        self.hb_stt_sub = self.create_subscription(Heartbeat, '/heartbeat/stt_node', self.hb_stt_callback, 10) #stt 노드 callback (이상:0, 정상:1)
        self.hb_hpl_sub = self.create_subscription(Heartbeat, '/heartbeat/hmi_planning_node', self.hb_hpl_callback, 10) #stt 노드 callback (이상:0, 정상:1)
        # 퍼블리셔 선언 (TTSNode 구독용)
        self.talkbutton_pub = self.create_publisher(Bool, '/talkbutton_pressed', 10)
        self.emergency_pub = self.create_publisher(Bool, '/emergency', 5)
        self.handlebutton_pub = self.create_publisher(UInt8, '/handlebutton_state', 5)
        self.hmi_stop_pub = self.create_publisher(Bool, '/hmi_stop', 10) #control 로 보내는 hmi 상태 

        self.create_timer(0.1, self.pub_log)  # 1초마다 heartbeat 퍼블리시

    def emergency_callback(self, msg):
    
        self.last_emergency_state = msg.data  # 상태 갱신
        self.emergency_pub.publish(Bool(data=msg.data))
        
    def stt_callback(self, msg):
            
            self.last_stt_state = msg.data  # 상태 갱신
            self.talkbutton_pub.publish(Bool(data=msg.data))

    def tact_callback(self, msg):
        # `Int32`로 받은 데이터를 `UInt8`로 변환
        
        self.tact_state = msg.data
        tact_code = 0

        # `Int32` -> `UInt8` 변환
        if self.tact_state == 0:  # 상태가 0이면
            tact_code = 0
        elif self.tact_state == 1:  # 상태가 1이면
            tact_code = 1
        elif self.tact_state == 2:  # 상태가 2이면
            tact_code = 2
        else:
            tact_code = 2  # 예외 처리, 적절한 값으로 설정 (기본값: 0)

        if tact_code != self.last_tact_state:  # 상태가 바뀌었을 때만 출력
            if tact_code == 0:
                self.get_logger().warn('[TACT] 손잡이 해제 감지됨 >3초 (0)')
            elif tact_code == 1:
                self.get_logger().info('[TACT] 손잡이 정상 인식 (1)')
            else:
                self.get_logger().warn('[TACT] 상태 불명 (2)')
        #self.tact_state = True
        self.last_tact_state = tact_code  # 상태 갱신
        self.handlebutton_pub.publish(UInt8(data=self.tact_state))  # 퍼블리시

    def pub_log(self):
        
        if self.stt_hb_state == 0:
            stt = '❌'
        elif self.stt_hb_state == 1:
            stt = '✅'
        if self.hpl_hb_state == 0:
            hpl = '❌'
        elif self.hpl_hb_state == 1:
            hpl = '✅'
        if self.last_emergency_state is None or self.last_emergency_state == False:
            emergency = '__'
            
        elif self.last_emergency_state:
            emergency = '🛑'
            
        if self.last_stt_state is None or self.last_stt_state == False:
            talk = '__'
        elif self.last_stt_state:
            talk = '🎤'
        
        if self.tact_state == 0 or self.last_emergency_state == True or self.hpl_hb_state == 0 :
            self.hmi_stop_state = True
        else: 
            self.hmi_stop_state = False
        
        self.get_logger().info(f'[STT]{stt} | [HPL]{hpl} | [EMERGENCY]{emergency} | [talkbt]{talk} |[tact]{self.tact_state} | ')    
        self.hmi_stop_pub.publish(Bool(data=self.hmi_stop_state))  # HMI stop 상태 퍼블리시

    def hb_stt_callback(self, msg): 
        stamp_sec = msg.timestamp.sec  
        stamp_nanosec = msg.timestamp.nanosec
        code = msg.code
        ros_time = datetime.datetime.fromtimestamp(stamp_sec + stamp_nanosec * 1e-9)
        if ros_time is not None:
            delta = datetime.datetime.now() - ros_time
            if delta.total_seconds() > 5:  # 5초 이상 차이가 나면 이상으로 간주
                self.stt_hb_state = 0
            else:
                
                self.stt_hb_state = 1
        
    def hb_hpl_callback(self, msg):
        stamp_sec = msg.timestamp.sec  
        stamp_nanosec = msg.timestamp.nanosec
        code = msg.code
        ros_time = datetime.datetime.fromtimestamp(stamp_sec + stamp_nanosec * 1e-9)
        if ros_time is not None:
            delta = datetime.datetime.now() - ros_time
            if delta.total_seconds() > 5:  # 5초 이상 차이가 나면 이상으로 간주
                self.hpl_hb_state = 0
            else:
                
                self.hpl_hb_state = 1
        
def main(args=None):
    rclpy.init(args=args)
    node = ButtonNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
