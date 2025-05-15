import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, UInt8
import difflib
from hmi_interface.srv import IntentResponse
from hmi_interface.srv import IntentToPlanning
from hmi_interface.srv import IntentToTTS
class IntentNode(Node):
    def __init__(self):
        super().__init__('intent_node')
        self.subscription = self.create_subscription(String,'/stt_text',self.stt_callback,10)
        
        #srv client 
        self.cli = self.create_client(IntentResponse, '/confirm_service') 
        self.intent_planning_client = self.create_client(IntentToPlanning, '/intent_to_planning') #planning 노드로 의도 전송
        self.intent_tts_client = self.create_client(IntentToTTS, '/intent_to_tts_plan') #tts 노드로 의도 전송
        self.intentpub = self.create_publisher(String, '/intent_to_tts', 10)  # /intent_to_tts 토픽으로 의도 전송
        self.confirm_pub = self.create_publisher(UInt8, '/confirm_request', 10)  # 확인 요청 상태 전송
        self.dstpub = self.create_publisher(UInt8, '/dst',1) 
        self.building_id_pub = self.create_publisher(UInt8, '/voice/building_id', 1)  # 건물 ID 전송
        self.get_logger().info('Intent Node started. Waiting for STT input...')

        self.user_text = None
        
        self.dst_dict = {
            "신공" :0, 
            "신공학관" :0,
            "공학관" :1,
            "공대" :1,
            "새천년관" :2,
            "학생회관" :3,
            "법학관" :4,
            "정문" :5,
            "후문" :6,
            "문과대":7,
            "인문학관":7,
            "경영대" :8,
            "경영관" :8
        }
        # 키워드 목록
        self.boosted_dst = ["공학관","공대","신공", "신공학관", "새천년관", "학생회관", "법학관","정문", "후문", "문과대","인문학관", "경영대", "경영관"]
        self.boosted_togo = ["가줘", "가자", "가고 싶어", "데려다줘", "이동", "갈래", "가고","가고싶어", "출발", "가야돼"]
        self.boosted_howlong = ["까지 ", "몇", "분이", "분이나", "얼마나", "도착", "시간", "걸릴까", "걸려", "남았어","얼마", "언제"]
        self.boosted_where = ["어디야", "현재 위치", "어디를", "지나고", "위치","지금", "어디", "현재", "지나"]
        self.boosted_yes = ["네", "예","어", "그래", "응","맞아", "맞습니다", "그렇습니다"]
        self.boosted_no = ["아니오", "아니", "아닌데","틀려", "틀렸어","아니야"]
        self.boosted_change_dst = ["바꿔줘", "바꾸다", "바꿀래", "변경", "변경할","바꿔", "변경해", "다시"]
        self.intent = None
        self.dst = None
        self.response_state = 'IDLE'
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
     
    def stt_callback(self, msg):
        user_text = msg.data.strip()
        self.get_logger().info(f"STT로 수신: {user_text}")
        self.closest_text(user_text)

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

        
        #목적지 수령
        for dst in self.boosted_dst:
            if dst in user_text:
                self.dst = dst
                break
        #의도 수령
        for key, keywords in intent_keywords.items():
            if any(k in user_text for k in keywords):
                self.intent = key
                break
        
        if self.intent == None: 
            self.intent = "unknown"
            self.send_intent_message(self.intent)
        
        elif self.intent == "set_destination":
            print("목적지 설정") 
            
            self.response_state = 'REQUEST_CONFIRM'

            req = IntentResponse.Request()
            req.intent = self.intent
            req.dst = self.dst_dict[self.dst]

            self.future = self.cli.call_async(req)
            self.future.add_done_callback(self.tts_response_callback)

        elif self.intent == "change_dst":
            
            print("목적지 변경")  
            self.send_intent_message(self.intent) #의도 전송

        elif self.intent == "get_eta" or self.intent == "get_location": # planning 팀과 협업 
            print("hmi planning 노드로 상태 보내주기 필요 ")
            planning_req = IntentToPlanning.Request()
            planning_req.intent = self.intent
            
            future = self.intent_planning_client.call_async(planning_req)
            future.add_done_callback(self.planning_response_callback)

        elif self.intent == "confirm_yes" or self.intent == "confirm_no":
            if self.response_state != 'WATING_CONFIRM': # 확인 요청, WATING_CONFIRM 상태가 아닐 때는 무시
                return
            else: 
                if self.intent == "confirm_yes":
                    self.response_state = 'CONFIRMED'
                    self.confirm_action(True) #의도 전송
                    
                    #목적지 확정 후 planning 노드로 목적지 전송 
                    encoded_dst= self.dst_dict[self.dst] 
                    self.building_id_pub.publish(UInt8(data=encoded_dst))#목적지 전송
                    self.get_logger().info(f"목적지 전송: {encoded_dst}")

                else:
                    self.response_state = 'DENIED'
                    self.confirm_action(False) #의도 전송

    def confirm_action(self, yesorno):
        req = IntentResponse.Request()
        req.intent = "confirm_yes" if yesorno else "confirm_no"
        req.dst = self.dst_dict[self.dst]

        future = self.cli.call_async(req)
        future.add_done_callback(self.final_response_callback)

    def tts_response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"TTS 출력 완료")
            self.response_state = 'WATING_CONFIRM' 
        except Exception as e:
            self.get_logger().error(f"서비스 호출 실패: {e}")
            self.response_state = 'IDLE'
    
    def final_response_callback(self, future):
        try:
            response = future.result()
            if self.response_state == 'CONFIRMED':

                self.get_logger().info("응답 수신: 목적지 설정 완료, 안내 시작")
                self.response_state = 'COMPLETED'
            else: 
                self.get_logger().info("응답 수신: 거부")
                self.response_state = 'IDLE'
                self.send_intent_message("unknown") # 모르겠음. 처리 불가 
        except Exception as e:
            self.get_logger().error(f"서비스 호출 실패: {e}")
            self.response_state = 'IDLE'
            self.send_intent_message("unknown") # 모르겠음. 처리 불가
    
    def planning_response_callback(self,future):
        # planning 노드로부터 응답을 받는 콜백 함수(서비스 체이닝)
        response = future.result()
        self.get_logger().info(f"hmi_planning 노드로부터 서비스 응답 확인")
        
        tts_req = IntentToTTS.Request() 
        tts_req.intent = response.intent_response
        tts_req.estimated_time_remaining = response.estimated_time_remaining
        tts_req.closest_landmark = response.closest_landmark
        
        self.intent_tts_client.call_async(tts_req)
        self.get_logger().info(f"tts 노드로 서비스 호출: {tts_req.intent}")
        

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