import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import difflib

class IntentNode(Node):
    def __init__(self):
        super().__init__('intent_node')
        self.subscription = self.create_subscription(String,'/stt_text',self.listener_callback,10)
        
        self.intent_to_tts_publisher = self.create_publisher(String, '/intent_to_tts', 10)  #set_dst or get_eta or get_loc or unknown

        #(pub) intent -> tts 로 가는  필요(set dst, get eta, get loc ) ~~~ 가 맞나요 ? 
        #(pub) planning으로 갈 정수 인코딩된 dst 
        
        self.get_logger().info('Intent Node started. Waiting for STT input...')

        # 키워드 목록
        self.boosted_dst = ["공학관", "신공학관", "새천년관", "학생회관", "법학관" ]
        self.boosted_togo = ["가줘", "가자", "가고 싶어", "데려다줘", "이동", "목적지",]
        #이것들 find_closest_destination 처럼 구현 필요
        self.boosted_howlong = ["몇 분", "얼마나", "도착 시간", "시간", "얼마 걸려", "남았어",]
        #이것들 find_closest_destination 처럼 구현 필요
        self.boosted_where = ["어디야", "지금 어디", "현재 위치", "어디를 지나", "위치"]
        #이것들 find_closest_destination 처럼 구현 필요


    def listener_callback(self, msg):
        user_text = msg.data.strip()
        self.get_logger().info(f'STT로부터 받은 텍스트: "{user_text}"')

        

        intent_info = self.classify_intent(user_text)
        self.generate_response(intent_info)

    def classify_intent(self, user_text):
        #여기서 전체 문장에 대해서 교정 한번 보고 뭔가 시작 !! 
        
        
        
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
        if intent == "get_eta":
            #여기서 단어를 찾아 
            
            
            print("get_eta logic 필요, action 호출 필요")
        return {
            "intent": intent,
            "destination": destination
        }

    def find_closest_destination(self, text):
        for word in text.split():
            match = difflib.get_close_matches(word, self.boosted_dst, n=1, cutoff=0.6)
            if match:
                return match[0]
        return None
    def find_closest_togo(self, text):
        for word in text.split():
            match = difflib.get_close_matches(word, self.boosted_togo, n=1, cutoff=0.6)
            if match:
                return match
        return None





    def generate_response(self, intent_info):
        intent = intent_info["intent"]
        destination = intent_info["destination"]

        # 의도에 맞는 텍스트가 아닌 의도만 /intent 토픽으로 전송
        if intent == "set_destination" and destination:
            self.send_intent_message("set_destination") 
            #여기서 dst 만 추출해서 정수로 바꿔서 전송하는 함수!! 
        elif intent == "get_eta":
            self.send_intent_message("get_eta")
            #여기서는 planning 쪽에 지금 어디인지?? 이거 물어보는 action 만들어야됨(나중)
        elif intent == "get_location":
            self.send_intent_message("get_location")
        else:
            self.send_intent_message("unknown")
            self.get_logger().warn("의도를 파악하지 못했습니다.")

    def send_intent_message(self, intent_message):
        intent_msg = String()
        intent_msg.data = intent_message
        self.intent_to_tts_publisher.publish(intent_msg)
        self.get_logger().info(f'의도 전송: "{intent_message}"')


def main(args=None):
    rclpy.init(args=args)
    node = IntentNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

