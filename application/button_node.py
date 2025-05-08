import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, UInt8
import pygame
import time
import threading
class ButtonNode(Node):
    def __init__(self):
        super().__init__('button_node')
        
        self.talkbutton_sub = self.create_subscription(Bool, '/talkbutton_state', self.talkbutton_callback, 10)
        self.handlebutton_sub = self.create_subscription(UInt8, '/handlebutton_state', self.handlebutton_callback, 10)
        self.emergency_sub = self.create_subscription(Bool, '/emergencybutton_state', self.emergencybutton_callback, 10)

        self.talkbutton_pub = self.create_publisher(Bool, '/talkbutton_pressed', 10)
        self.emergency_pub = self.create_publisher(Bool, '/emergency', 10)
        self.handlebutton_pub = self.create_publisher(UInt8, '/handlebutton_code', 10)
        self.hmi_stop_pub = self.create_publisher(Bool, '/hmi_stop', 10)

        self.handlebutton_ = self.create_publisher(UInt8, '/handlebutton_state', 10)
        
        # 버튼 콜백 변수
        self.talkbutton_pressed = False
        self.handlebutton_status = 0 
        self.emergency_button_pressed = False

        #핸들버튼 상태 모니터링용
        self.lock = threading.Lock()
        self.current_status = None
        self.monitor_thread = None
        self.monitoring = False

        self.get_logger().info("Keyboard Button Publisher Node has started.")
    
    # talkbutton 상태를 받아서 처리하는 콜백 (필요시 구현)
    def talkbutton_callback(self, msg):
        self.talkbutton_pressed = msg.data
        #self.get_logger().info(f"Talk button 상태: {self.talkbutton_pressed}")
        self.talkbutton_pub.publish(Bool(data=self.talkbutton_pressed))
        
    # handlebutton 상태 콜백   
    def handlebutton_callback(self, msg):
        new_status = msg.data

        with self.lock:
            self.current_status = new_status

            #양 손 떨어짐
            if new_status == 0:
                if not self.monitoring:
                    self.monitoring = True
                    self.monitor_thread = threading.Thread(target=self.monitor_zero_status)
                    self.monitor_thread.daemon = True
                    self.monitor_thread.start() 
            #양 손 잡음. pub
            elif new_status == 2:
                self.monitoring = False
                #self.get_logger().info("양 손 잡음 → publish(2)")
                self.handlebutton_pub.publish(UInt8(data=2))
                self.hmi_stop_pub.publish(Bool(data=False))
                
            #한쪽 손 잡음. 3초 모니터링 후 pub     
            elif new_status == 1:
                if not self.monitoring:
                    self.monitoring = True
                    self.monitor_thread = threading.Thread(target=self.monitor_one_status)
                    self.monitor_thread.daemon = True
                    self.monitor_thread.start()
    # 양 손 떨어지면 0.5초 모니터링 함수 
    def monitor_zero_status(self):
        start_time = time.time()
        while time.time() - start_time < 0.5:
            with self.lock:
                if self.current_status != 0:
                    #self.get_logger().info("0.5초 내 상태 변화 감지됨 → publish(0) 생략")
                    self.monitoring = False
                    return
            time.sleep(0.05)  # 20Hz 감시

        with self.lock:
            if self.current_status == 0:
                self.handlebutton_pub.publish(UInt8(data=0))
                #self.get_logger().info("비상정지")
                self.hmi_stop_pub.publish(Bool(data=True)) 
            self.monitoring = False
    # 한쪽 손만 잡으면 3초 모니터링 함수
    def monitor_one_status(self):
        """핸들버튼 상태 모니터링 함수"""
        start_time = time.time()
        while time.time() - start_time < 2.0:
            with self.lock:
                if self.current_status != 1: #상태 변경됨
                    self.monitoring = False
                    return
            time.sleep(0.05)  # 약 20Hz로 감시

        with self.lock:
            if self.current_status == 1:
                self.handlebutton_pub.publish(UInt8(data=1))
                self.get_logger().info("손잡이를 잡아주세요")
            self.monitoring = False
        self.hmi_stop_pub.publish(Bool(data=False)) 

    def emergencybutton_callback(self, msg):
        self.emergency_button_pressed = msg.data
        # emergencybutton 상태를 받아서 처리하는 콜백 (필요시 구현)

def main(args=None):
    rclpy.init(args=args)
    node = ButtonNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
