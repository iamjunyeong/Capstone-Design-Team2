import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import curses
import time

class KeyboardButtonPublisher(Node):
    def __init__(self, stdscr):
        super().__init__('button_node')
        self.stdscr = stdscr

        # 퍼블리셔 등록
        self.talkbutton_pub = self.create_publisher(Bool, '/talkbutton_pressed', 10)
        self.emergency_pub = self.create_publisher(Bool, '/emergency_button_pressed', 10)

        self.pressed = False  # 현재 버튼 눌림 상태
        self.current_button = None  # 'talk', 'emergency', None
        self.emergency_active = False  # emergency 상태 여부

        self.get_logger().info("Keyboard Button Publisher Node has started.")
        self.run_keyboard_listener()

    def run_keyboard_listener(self):
        self.stdscr.nodelay(True)
        curses.curs_set(0)
        self.stdscr.clear()
        self.stdscr.addstr(0, 0, "1: 음성 상호작용\n2: 비상정지\nq: 종료")
        self.stdscr.refresh()

        pub_rate = 0.1  # 10Hz
        last_pub_time = 0
        key_down_detected = False
        key_down_time = 0

        while rclpy.ok():
            key = self.stdscr.getch()
            now = time.time()

            # 종료
            if key == ord('q'):
                self.get_logger().info("종료합니다.")
                break

            # 버튼 눌림 감지
            if key == ord('1'):
                key_down_detected = True
                key_down_time = now
                self.current_button = 'talk'

            elif key == ord('2'):
                key_down_detected = True
                key_down_time = now
                self.current_button = 'emergency'

            # 눌림 상태 반영
            if key_down_detected and not self.pressed:
                if now - key_down_time > 0.03:
                    self.pressed = True

                    if self.current_button == 'emergency' and not self.emergency_active:
                        self.publish_emergency(True)
                        self.emergency_active = True

            # 버튼이 떨어졌다고 판단 (0.5초 이상 입력 없음)
            if self.pressed and (now - key_down_time > 0.5):
                self.pressed = False
                key_down_detected = False

                if self.current_button == 'emergency' and self.emergency_active:
                    self.publish_emergency(False)
                    self.emergency_active = False

                self.current_button = None

            # 1번 버튼은 주기적으로 발행
            if self.current_button == 'talk' and now - last_pub_time > pub_rate:
                self.publish_talk(self.pressed)
                status = "Pressed  " if self.pressed else "Released "
                self.stdscr.addstr(2, 0, f"상태: {status}")
                self.stdscr.refresh()
                last_pub_time = now

            time.sleep(0.005)

    def publish_talk(self, state):
        msg = Bool()
        msg.data = state
        self.talkbutton_pub.publish(msg)

    def publish_emergency(self, state):
        msg = Bool()
        msg.data = state
        self.emergency_pub.publish(msg)
        status = "비상 정지 시작" if state else "비상 정지 해제"
        self.get_logger().info(f"[EMERGENCY] {status}")

def main(args=None):
    rclpy.init(args=args)
    curses.wrapper(lambda stdscr: KeyboardButtonPublisher(stdscr))
    rclpy.shutdown()

if __name__ == '__main__':
    main()
