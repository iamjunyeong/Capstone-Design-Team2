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
            '/intent',
            self.listener_callback,
            10
        )
        self.get_logger().info('✅ TTS Node has started. Listening to /intent')

    def listener_callback(self, msg):
        intent = msg.data
        self.get_logger().info(f'🔈 의도 수령: "{intent}"')
        self.speak(intent)

    def speak(self, intent):
        if intent.startswith("set_destination:"):
            destination = intent.split(":")[1]
            response = f"목적지를 {destination}로 설정하였습니다."
        elif intent == "get_eta":
            response = "남은 시간은 <플래닝 시간 변수> 분입니다."
        elif intent == "get_location":
            response = "현재 위치는 <플래닝 위치 변수> 앞입니다."
        else:
            response = "알 수 없는 요청입니다."

        # 터미널에 출력
        self.get_logger().info(f"응답: {response}")

        try:
            tts = gTTS(text=response, lang='ko')
            fp = io.BytesIO()
            tts.write_to_fp(fp)
            fp.seek(0)
            audio = AudioSegment.from_file(fp, format='mp3')
            play(audio)
        except Exception as e:
            self.get_logger().error(f'TTS 오류: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = TTSNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

