
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, UInt8
import pygame
import time

class KeyboardButtonPublisher(Node):
    def __init__(self):
        super().__init__('button_node')
        
        self.talkbutton_sub = self.create_subscription(Bool, '/talkbutton_state', self.talkbutton_callback, 10)
        self.handlebutton_sub = self.create_subscription(UInt8, '/handlebutton_state', self.handlebutton_callback, 10)
        self.emergency_sub = self.create_subscription(Bool, '/emergencybutton_state', self.emergencybutton_callback, 10)

        self.talkbutton_pub = self.create_publisher(Bool, '/talkbutton_pressed', 10)
        self.emergency_pub = self.create_publisher(Bool, '/emergency_button_pressed', 10)
        self.handlebutton_pub = self.create_publisher(UInt8, '/handlebutton_code', 10)

        self.talkbutton_pressed = False
        self.handlebutton_status = 0 
        self.emergency_button_pressed = False

        self.get_logger().info("Keyboard Button Publisher Node has started.")
        self.run_keyboard_listener()

    def talkbutton_callback(self, msg):
        self.talkbutton_pressed = msg.data
        # talkbutton 상태를 받아서 처리하는 콜백 (필요시 구현)
    def handlebutton_callback(self, msg):
        # handlebutton 상태를 받아서 처리하는 콜백 (필요시 구현)
        self.handlebutton_status = msg.data
        if self.handlebutton_status == 0:
            self.stdscr.addstr(4, 0, "핸들 버튼 눌리지 않음")
            self.handlebutton_pub.publish(0)
        elif self.handlebutton_status == 1:
            self.stdscr.addstr(4, 0, "핸들 버튼 1개 눌림")
            self.handlebutton_pub.publish(1)
        elif self.handlebutton_status == 2:
            self.stdscr.addstr(4, 0, "핸들 버튼 2개 눌림")
            self.handlebutton_pub.publish(2)
        self.stdscr.refresh()
    def emergencybutton_callback(self, msg):
        self.emergency_button_pressed = msg.data
        # emergencybutton 상태를 받아서 처리하는 콜백 (필요시 구현)
        
    def run_keyboard_listener(self):
        pygame.init()
        screen = pygame.display.set_moade((400, 300))
        pygame.display.set_caption("Keyboard Button Control")
        clock = pygame.time.Clock()

        pressed = False
        emergency_active = False
        talk_pub_time = 0.0
        pub_rate = 0.1  # 10Hz

        running = True
        while rclpy.ok() and running:
            rclpy.spin_once(self, timeout_sec=0.0)
            screen.fill((255, 255, 255))

            keys = pygame.key.get_pressed()
            now = time.time()

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

            # 비상 정지: '2' 키
            if keys[pygame.K_2]:
                if not emergency_active:
                    self.publish_emergency(True)
                    emergency_active = True
            else:
                if emergency_active:
                    self.publish_emergency(False)
                    emergency_active = False

            # 음성 상호작용: '1' 키 (주기적 발행)
            if keys[pygame.K_1]:
                pressed = True
            else:
                pressed = False

            if now - talk_pub_time > pub_rate:
                self.publish_talk(pressed)
                talk_pub_time = now

            # 핸들 버튼 코드: 'a', 's' 키 감지
            code = 0
            if keys[pygame.K_a] and keys[pygame.K_s]:
                code = 2
            elif keys[pygame.K_a] or keys[pygame.K_s]:
                code = 1
            self.publish_handle(code)

            pygame.display.flip()
            clock.tick(60)

        pygame.quit()

    def publish_talk(self, state: bool):
        self.talkbutton_pub.publish(Bool(data=state))
        status = "음성 상호작용 시작" if state else "음성 상호작용 종료"
        self.get_logger().info(f"[TALK] {status}")
    def publish_handle(self, state: int):
        self.handlebutton_pub.publish(UInt8(data=state))
        self.get_logger().info(f"[HANDLE] {state}")
    
    def publish_emergency(self, state: bool):
        self.emergency_pub.publish(Bool(data=state))
        status = "비상 정지 시작" if state else "비상 정지 해제"
        self.get_logger().info(f"[EMERGENCY] {status}")

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardButtonPublisher()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
