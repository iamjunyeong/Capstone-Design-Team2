import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import difflib

class IntentNode(Node):
    def __init__(self):
        super().__init__('intent_node')
        self.subscription = self.create_subscription(String,'/stt_text',self.listener_callback,10)
        self.publisher_ = self.create_publisher(String, '/intent', 10)  # /intent 토픽으로 의도만 전송
        self.get_logger().info('Intent Node started. Waiting for STT input...')

        self.user_text = None

        # 키워드 목록
        self.boosted_dst = ["공학관", "신공학관", "새천년관", "학생회관", "법학관" ]
        self.boosted_togo = ["가줘", "가자", "가고 싶어", "데려다줘", "이동", "갈래", "가고","가고싶어", "출발", "가야돼"]
        self.boosted_howlong = ["까지 ", "몇", "분이", "분이나", "얼마나", "도착", "시간", "걸릴까", "걸려", "남았어","얼마", "언제"]
        self.boosted_where = ["어디야", "현재 위치", "어디를", "지나고", "위치","지금", "어디", "현재", "지나"]
        self.boosted_yesno = ["네", "예", "그래", "응", "아니오", "아니", "아닌데","맞아", "맞습니다", "그렇습니다", "아닙니다", "틀려", "틀렸어","아니야" ]
        self.boosted_change_dst = ["바꿔줘", "바꾸다", "바꿀래", "변경", "변경할","바꿔", "변경해", "다시"]

        # self.input_loop() #stt 대신 키보드 입력으로 하기위한 함수

    # def input_loop(self): #stt 대신 키보드 입력으로 하기위한 함수
    #     while rclpy.ok():

    #         self.get_logger().info('입력')
    #         unprocessed_text = input()
    #         if not unprocessed_text:
    #             continue

    #         # 메시지 객체로 래핑해서 콜백처럼 처리
    #         msg = String()
    #         msg.data = unprocessed_text
    #         # self.listener_callback(msg)
    #         self.closest_text(msg)

    def closest_text(self, msg): # 입력받은 stt를 의도파악 전 관련있는 단어들만 추출
        unprocessed_text = msg.data.strip()
        self.get_logger().info(f'STT로부터 받은 텍스트: "{unprocessed_text}"')
        processed_text = []
        processed_text.extend(self.find_closest_destination(unprocessed_text))
        processed_text.extend(self.find_closest_howlong(unprocessed_text))
        processed_text.extend(self.find_closest_togo(unprocessed_text))
        processed_text.extend(self.find_closest_where(unprocessed_text))
        processed_text.extend(self.find_closest_yesno(unprocessed_text))
        processed_text.extend(self.find_closest_change_dst(unprocessed_text))
        user_text = ""
        
        #중복 제거 
        processed_text = list(set(processed_text))

        self.get_logger().info(f"추출된 키워드: {processed_text}")
        for i in processed_text:
            user_text += i + " "
        # self.get_logger().info(f"추출된 키워드: {user_text}")
        # self.listener_callback(user_text)

        intent_info = self.classify_intent(user_text)
        self.generate_response(intent_info)


    def listener_callback(self, msg):
        user_text = msg.data.strip()
        self.get_logger().info(f'STT로부터 받은 텍스트: "{user_text}"')

        

        intent_info = self.classify_intent(user_text)
        self.generate_response(intent_info)

    def classify_intent(self, user_text):
        #여기서 전체 문장에 대해서 교정 한번 보고 뭔가 시작 !! 
        
        
        
        intent_keywords = {
            "change_dst" : ["바꿔줘", "바꾸다", "바꿀래", "변경", "변경할","바꿔", "변경해", "다시"],
            "set_destination": ["가줘", "가자", "가고 싶어", "데려다줘", "이동", "갈래", "가고","가고싶어", "출발", "가야돼"],
            "get_eta": ["까지 ", "몇", "분이", "분이나", "얼마나", "도착", "시간", "걸릴까", "걸려", "남았어","얼마", "언제"],
            "get_location": ["어디야", "현재 위치", "어디를", "지나고", "위치","지금", "어디", "현재", "지나"],
            "confirm" : ["네", "예", "그래", "응", "아니오", "아니", "아닌데","맞아", "맞습니다", "그렇습니다", "아닙니다", "틀려", "틀렸어","아니야"],
        }

        destination = None
        intent = None

        for key, keywords in intent_keywords.items():
            if any(k in user_text for k in keywords):
                intent = key
                break

        if intent == "set_destination":
            destination = self.find_closest_destination(user_text)

        if intent == "get_eta": # planning 팀과 협업 
            print("get_eta logic 필요, action 호출 필요")
        if intent == "change_dst":
            print("목적지 변경")  
        return {
            "intent": intent,
            "destination": destination
        }
    

    def find_closest_destination(self, text):
        match = []
        for word in text.split():
            match.extend(difflib.get_close_matches(word, self.boosted_dst, n=1, cutoff=0.6))
        if match:
            return match
        else:
            return []
        
    def find_closest_change_dst(self, text):
        match = []
        for word in text.split():
            match.extend(difflib.get_close_matches(word, self.boosted_change_dst, n=1, cutoff=0.6))
        if match:
            return match
        else:
            return []
    
    def find_closest_howlong(self, text):
        match = []
        for word in text.split():
            match.extend(difflib.get_close_matches(word, self.boosted_howlong, n=1, cutoff=0.6))
        if match:
            return match
        else:
            return []
    def find_closest_togo(self, text):
        for word in text.split():
            match = difflib.get_close_matches(word, self.boosted_yesno, n=1, cutoff=0.6)
            if match:
                return match
        return []
    
    def find_closest_where(self, text):
        for word in text.split():
            match = difflib.get_close_matches(word, self.boosted_where, n=1, cutoff=0.6)
            if match:
                return match
        return []
    
    def find_closest_yesno(self, text):
        for word in text.split():
            match = difflib.get_close_matches(word, self.boosted_yesno, n=1, cutoff=0.6)
            if match:
                return match
        return []

    def generate_response(self, intent_info):
        intent = intent_info["intent"]
        destination = intent_info["destination"]

        # 의도에 맞는 텍스트가 아닌 의도만 /intent 토픽으로 전송
        if intent == "set_destination" and destination:
            self.send_intent_message(f"set_destination: {destination}")
            #여기서 dst 만 추출해서 정수로 바꿔서 전송하는 함수!! 
        elif intent == "change_dst":
            self.send_intent_message("change_dst")
        elif intent == "get_eta":
            self.send_intent_message("get_eta")
            #여기서는 planning 쪽에 지금 어디인지?? 이거 물어보는 action 만들어야됨(나중)
        elif intent == "get_location":
            self.send_intent_message("get_location")
        elif intent == "confirm":
            self.send_intent_message("confirm")
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

#내가 처음에는 목적지를 새천연관으로 설정했었는데 신공학관으로 바꾸고 싶어졌어요 
#엄 음 새천년관까지는 몇 분이나 걸릴지 한 번 말해보세요 
#새청년관까지 얼마나 걸리나요?
#어디를 어디 어디야 