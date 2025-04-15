import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from gtts import gTTS
import pygame
import time
import threading

class SimpleTTSNode(Node):
    def __init__(self):
        super().__init__('simple_tts_node')
        self.last_button_state = False
        self.subscription = self.create_subscription(Bool,'/talkbutton_pressed',self.talk_button_callback,10)
        self.sub = self.create_subscription(Bool, '/emergency_button_pressed', self.emergency_button_callback, 10)
        self.is_playing = False
        self.lock = threading.Lock()

        # pygame mixer 초기화
        try:
            pygame.mixer.init()
            self.get_logger().info("🔊 pygame mixer 초기화 완료")
        except Exception as e:
            self.get_logger().error(f"pygame mixer 초기화 실패: {e}")

        self.get_logger().info("✅ TTS 테스트 노드 시작됨 (버튼 누르면 '듣고있어요' 출력)")

    def talk_button_callback(self, msg):
        if msg.data and not self.last_button_state:
            self.last_button_state = True  # 눌림 감지
            self.get_logger().info("🟢 버튼 최초 눌림 → 반응")

            with self.lock:
                if self.is_playing:
                    pygame.mixer.music.stop()
                    self.is_playing = False
                threading.Thread(target=self.play_tts, args=("듣고있어요",)).start()
        elif not msg.data:
            self.last_button_state = False  # 버튼 떨어짐
    def emergency_button_callback(self, msg):
        if msg.data:
            self.get_logger().info("🔴 비상 정지 버튼 눌림 → 비상 정지"
                                   )        
    def play_tts(self, text):
        try:
            self.is_playing = True
            tts = gTTS(text=text, lang='ko')
            filename = 'output.mp3'
            tts.save(filename)

            pygame.mixer.music.load(filename)
            pygame.mixer.music.play()

            while pygame.mixer.music.get_busy():
                time.sleep(0.1)

        except Exception as e:
            self.get_logger().error(f"TTS 재생 오류: {e}")
        finally:
            self.is_playing = False

def main(args=None):
    rclpy.init(args=args)
    node = SimpleTTSNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
