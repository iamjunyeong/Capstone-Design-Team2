#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, UInt8
import difflib

class IntentNode(Node):
    def __init__(self):
        super().__init__('intent_node')
        self.subscription = self.create_subscription(String,'/stt_text',self.listener_callback,10)
        self.confirm_sub = self.create_subscription(UInt8, '/response_state', self.confirm_callback, 10)  # 확인 요청 상태 구독
        
        self.intentpub = self.create_publisher(String, '/intent_to_tts', 10)  # /intent_to_tts 토픽으로 의도 전송
        self.confirm_pub = self.create_publisher(UInt8, '/confirm_request', 10)  # 확인 요청 상태 전송
        self.dstpub = self.create_publisher(UInt8, '/dst',1) 
        self.building_id_pub = self.create_publisher(UInt8, '/voice/building_id', 1)  # 건물 ID 전송
        self.get_logger().info('Intent Node started. Waiting for STT input...')

        self.user_text = None
        self.response_state = 0 # 0:대기 중, 1: 확인 요청 진행 중, 2:확인 요청 완료, 3:거절, 재질의 
        self.dst_dict = {
            "신공" :0, 
            "신공학관" :0,
            "공학관" :1,
            "공대" :1,
            "새천년관" :2,
            "학생회관" :3,
            "학관":3,
            "법학관" :4,
            "정문" :5,
            "후문" :6,
            "문과대":7,
            "인문학관":7,
            "경영대" :8,
            "경영관" :8
        }
        # 키워드 목록
        self.boosted_dst = ["공학관","공대","신공", "신공학관", "새천년관", "학생회관","학관", "법학관","정문", "후문", "문과대","인문학관", "경영대", "경영관"]
        self.boosted_togo = ["가줘", "가자", "가고 싶어", "데려다줘", "이동", "갈래", "가고","가고싶어", "출발", "가야돼"]
        self.boosted_howlong = ["까지 ", "몇", "분이", "분이나", "얼마나", "도착", "시간", "걸릴까", "걸려", "남았어","얼마", "언제"]
        self.boosted_where = ["어디야", "현재 위치", "어디를", "지나고", "위치","지금", "어디", "현재", "지나"]
        self.boosted_yes = ["네", "예","어", "그래", "응","맞아", "맞습니다", "그렇습니다"]
        self.boosted_no = ["아니오", "아니", "아닌데","틀려", "틀렸어","아니야"]
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
     
    def listener_callback(self, msg):
        user_text = msg.data.strip()
        self.get_logger().info(f"STT로 수신: {user_text}")
        self.closest_text(user_text)
    
    def confirm_callback(self, msg):
        self.response_state = msg.data 
        self.get_logger().info(f"확인 요청 상태: {self.response_state}")

    def closest_text(self, msg): # 입력받은 stt를 의도파악 전 관련있는 단어들만 추출
        unprocessed_text = msg
        self.get_logger().info(f"의도 처리 시작: {unprocessed_text}")
        processed_text = []
        processed_text.extend(self.find_closest_word(unprocessed_text, self.boosted_dst))
        processed_text.extend(self.find_closest_word(unprocessed_text, self.boosted_togo))
        processed_text.extend(self.find_closest_word(unprocessed_text, self.boosted_howlong))
        processed_text.extend(self.find_closest_word(unprocessed_text, self.boosted_where))
        processed_text.extend(self.find_closest_word(unprocessed_text, self.boosted_yes))
        processed_text.extend(self.find_closest_word(unprocessed_text, self.boosted_no))
        processed_text.extend(self.find_closest_word(unprocessed_text, self.boosted_change_dst))
        user_text = ""
        
        #중복 제거 
        processed_text = list(set(processed_text))

        self.get_logger().info(f"추출된 키워드: {processed_text}")
        for item in processed_text:
            user_text += item + " "
        # self.get_logger().info(f"추출된 키워드: {user_text}")
        # self.listener_callback(user_text)

        intent_info = self.classify_intent(user_text)
        return intent_info

   
    def classify_intent(self, user_text):
        """의도 분류 및 목적지 추출"""
        intent_keywords = {
            "set_destination": ["가줘", "가자", "데려다줘", "이동", "갈래", "가고", "가고싶어", "출발", "가야돼"
                            "공학관","공대","신공", "신공학관", "새천년관", "학생회관","학관", "법학관","정문", "후문", "문과대", "경영대", "경영관"],
            "change_dst" : ["바꿔줘", "바꾸다", "바꿀래", "변경", "변경할","바꿔", "변경해", "다시"],
            "get_eta": ["까지 ", "몇", "분이", "분이나", "얼마나", "도착", "시간", "걸릴까", "걸려", "남았어","얼마", "언제"],
            "get_location": ["어디야", "현재 위치", "어디를", "지나고", "위치","지금", "어디", "현재", "지나"],
            "confirm_yes" : ["네", "예","어", "그래", "응","맞아", "맞습니다", "그렇습니다"],
            "confirm_no" : ["아니오", "아니", "아닌데","틀려", "틀렸어","아니야"],
        }

        destination = None
        intent = None
        #목적지 수령
        for dst in self.boosted_dst:
            if dst in user_text:
                destination = dst
                break
        #의도 수령
        for key, keywords in intent_keywords.items():
            if any(k in user_text for k in keywords):
                intent = key
                break
        
        if intent == None: 
            intent = "unknown"
            self.send_intent_message(intent)
        
        elif intent == "set_destination":
            print("목적지 설정") 
            self.send_intent_message(intent) #의도 전송
               
            if destination is not None:
                encoded_dst= self.dst_dict[destination] 
                self.response_state = 1 
                self.confirm_pub.publish(UInt8(data=self.response_state)) #재확인 요청 전송
                self.dstpub.publish(UInt8(data=encoded_dst))#목적지 전송
                #self.building_id_pub.publish(UInt8(data=encoded_dst))#목적지 전송
                self.get_logger().info(f"목적지 전송: {encoded_dst}")

        elif intent == "change_dst":
           
            print("목적지 변경")  
            self.send_intent_message(intent) #의도 전송

        elif intent == "get_eta": # planning 팀과 협업 
            print("hmi planning 노드로 상태 보내주기 필요 ")
            self.send_intent_message(intent) #의도 전송
             
        elif intent == "get_location":
            print("hmi planning 노드로 상태 보내주기 필요 ")
            self.send_intent_message(intent) #의도 전송

        elif intent == "confirm_yes":
            if self.response_state == 2: # 확인 요청 상태일 때만
                
                #self.send_intent_message(intent) #의도 전송
                self.response_state = 3 
                self.confirm_pub.publish(UInt8(data=self.response_state))  # 확인 요청 완료 상태 전송
                
            if destination:
                encoded_dst= self.dst_dict[destination] 
                self.building_id_pub.publish(UInt8(data=encoded_dst))#목적지 전송
                self.get_logger().info(f"목적지 전송: {encoded_dst}")
        
        elif intent == "confirm_no":
            print("no")
            self.response_state = 4 #재질의 알고리즘 실행 위해 tts로 상태 송신 
            self.confirm_pub.publish(UInt8(data=self.response_state))  # 확인 요청 완료 상태 전송
            self.send_intent_message(intent) #의도 전송
        
    def find_closest_word(self, text, boosted_list):
        match = []
        for word in text.split():
            match.extend(difflib.get_close_matches(word, boosted_list, n=1, cutoff=0.6))
        if match:
            return match
        else:
            return []

    def send_intent_message(self, intent):
        intent_msg = String()
        intent_msg.data = intent
        self.intentpub.publish(intent_msg)
        self.get_logger().info(f'의도 전송: "{intent}"')
        

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