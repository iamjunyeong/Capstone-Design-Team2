import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from gtts import gTTS
from pydub import AudioSegment
from pydub.playback import play
import io

class TTSNode(Node):
    def __init__(self):
        super().__init__('tts_node')
        self.subscription = self.create_subscription(
            String,
            '/tts_text',
            self.listener_callback,
            10
        )
        self.get_logger().info('✅ TTS Node has started. Listening to /tts_text')

    def listener_callback(self, msg):
        text = msg.data
        self.get_logger().info(f'🔈 TTS 요청: "{text}"')
        self.speak(text)

    def speak(self, text):
        try:
            tts = gTTS(text=text, lang='ko')
            fp = io.BytesIO()
            tts.write_to_fp(fp)
            fp.seek(0)
            audio = AudioSegment.from_file(fp, format='mp3')
            play(audio)
        except Exception as e:
            self.get_logger().error(f'TTS 오류: {e}')

    def speak_remaining_time(self, minutes):
        self.speak(f"남은 시간은 {minutes}분입니다.")

    def speak_current_location(self, place):
        self.speak(f"현재 위치는 {place} 앞입니다.")

    def speak_destination_set(self, destination):
        self.speak(f"목적지를 {destination}로 설정하였습니다.")

def main(args=None):
    rclpy.init(args=args)
    node = TTSNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()