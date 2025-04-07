import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import difflib

class IntentNode(Node):
    def __init__(self):
        super().__init__('intent_node')
        self.subscription = self.create_subscription(
            String,
            '/stt_text',
            self.listener_callback,
            10
        )
        self.publisher_ = self.create_publisher(String, '/tts_text', 10)
        self.get_logger().info('🧠 Intent Node started. Waiting for STT input...')

        # 키워드 목록
        self.boosted_keywords = [
            "공학관", "신공학관", "새천년관", "학생회관", "법학관",
            "가줘", "가자", "가고 싶어", "데려다줘", "이동", "목적지",
            "몇 분", "얼마나", "도착 시간", "시간", "얼마 걸려", "남았어",
            "어디야", "지금 어디", "현재 위치", "어디를 지나", "위치"
        ]

    def listener_callback(self, msg):
        user_text = msg.data.strip()
        self.get_logger().info(f'👂 STT로부터 받은 텍스트: "{user_text}"')

        intent_info = self.classify_intent(user_text)
        response = self.generate_response(intent_info)

        if response:
            self.get_logger().info(f'📤 TTS 응답: "{response}"')
            tts_msg = String()
            tts_msg.data = response
            self.publisher_.publish(tts_msg)
        else:
            self.get_logger().warn("❓ 의도를 파악하지 못했습니다.")

    def classify_intent(self, user_text):
        intent_keywords = {
            "set_destination": ["가줘", "가자", "가고 싶어", "데려다줘", "이동", "목적지"],
            "get_eta": ["몇 분", "얼마나", "도착 시간", "시간", "얼마 걸려", "남았어"],
            "get_location": ["어디야", "지금 어디", "현재 위치", "어디를 지나", "위치"]
        }

        destination = None
        intent = None

        for key, keywords in intent_keywords.items():
            if any(k in user_text for k in keywords):
                intent = key
                break

        if intent == "set_destination":
            destination = self.find_closest_destination(user_text)

        return {
            "intent": intent,
            "destination": destination
        }

    def find_closest_destination(self, text):
        for word in text.split():
            match = difflib.get_close_matches(word, self.boosted_keywords, n=1, cutoff=0.6)
            if match:
                return match[0]
        return None

    def generate_response(self, intent_info):
        intent = intent_info["intent"]
        destination = intent_info["destination"]

        if intent == "set_destination" and destination:
            return f"목적지를 {destination}로 설정하였습니다."
        elif intent == "get_eta":
            return "남은 시간은 5분입니다."  # 임시 값
        elif intent == "get_location":
            return "현재 위치는 공학관 앞입니다."  # 임시 값
        else:
            return None


def main(args=None):
    rclpy.init(args=args)
    node = IntentNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

