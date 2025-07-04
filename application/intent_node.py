import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, UInt8
import difflib
from hmi_interface.srv import IntentResponse
from hmi_interface.srv import IntentToPlanning
from hmi_interface.srv import IntentToTTS
from hmi_interface.msg import IntentToPlanningmsg
class IntentNode(Node):
    def __init__(self):
        super().__init__('intent_node')
        #sub 
        self.subscription = self.create_subscription(String,'/stt_text',self.stt_callback,10)
        #srv client 
        self.main_intent_client = self.create_client(IntentResponse, '/confirm_service') # TTS 노드로 메인 의도 전송 
        self.intent_planning_client = self.create_client(IntentToPlanning, '/intent_to_planning') # hmi_planning 노드로 의도 전송
        self.intent_tts_client = self.create_client(IntentToTTS, '/intent_to_tts_plan') # TTS 노드로 주행 중 질문 전송 
        #pub
        self.confirm_pub = self.create_publisher(UInt8, '/confirm_request', 10)  # 확인 요청 상태 전송
        self.building_id_pub = self.create_publisher(IntentToPlanningmsg, '/voice/building_id', 1)  # 건물 ID 전송
        self.heartbeat_pub = self.create_publisher(UInt8, '/heartbeat/intent_node', 10)  # heartbeat 퍼블리셔
        #variables
        self.replanning = False #replanning flag 
        self.intent = None # intent now 
        self.dst = None #destination now 
        self.response_state = 'IDLE' # state flag 
        self.intentmemory = None # intent memory 
        self.dstmemory = None # destination memory

        # destination dict, for encoding
        self.dst_dict = {
            "신공" :1, 
            "신공학관" :1,
            "공학관" :5,
            "공대" :5,
            "학생회관" :8,
            "청심대" : 9,
            "법학관" :11,
            "종강" :11,
            "종합강의동" :11,
            "수의대" :15, 
            "수의학관":15,
            "동생대" :18,
            "동물생명과학관" :18,
            "입학정보관" :20,
            "정문":20 
        }
        # keywords for intent classification
        #self.boosted_dst = ["신공학관", "공학관","공대","신공", "새천년관", "학생회관", "법학관","정문", "후문", "문과대","인문학관", "경영대", "경영관"]
        self.boosted_dst = ["신공", "신공학관", "공학관", "공대", "학생회관","청심대", "법학관", "종강", "종합강의동", "수의대", "수의학관", "동생대", "동물생명과학관", "입학정보관","정문"]
        #self.boosted_togo = ["가줘", "가자", "가고 싶어", "데려다줘", "이동", "갈래", "가고","가고싶어", "출발", "가야돼"]
        self.boosted_howlong = ["까지 ", "몇", "분이", "분이나", "얼마나", "도착", "시간", "걸릴까", "걸려", "남았어","얼마", "언제"]
        self.boosted_where = ["어디야", "현재 위치", "어디를", "지나고", "위치","지금", "어디", "현재", "지나"]
        self.boosted_yes = ["네", "예","어", "그래", "응","맞아", "맞습니다", "그렇습니다"]
        self.boosted_no = ["아니오", "아니", "아닌데","틀려", "틀렸어","아니야"]
        self.boosted_change_dst = ["바꿔줘", "바꾸다", "바꿀래", "변경", "변경할","바꿔", "변경해", "다시", "목적지"]
        self.boosted_stop = ["튜토리얼","정지", "멈춰", "정지해", "멈춰줘"]

        self.get_logger().info('Intent Node started. Waiting for STT input...')

    # text from STT_node 
    def stt_callback(self, msg):
        user_text = msg.data.strip() 
        self.get_logger().info(f"STT로부터 수신: {user_text}")
        self.closest_text(user_text) # keywords matching 

    # 키워드 매칭
    def closest_text(self, msg): 
        unprocessed_text = msg
        self.get_logger().info(f"의도 처리 시작: {unprocessed_text}")
        processed_text = []
        processed_text.extend(self.find_closest_word(unprocessed_text, self.boosted_dst))
        #processed_text.extend(self.find_closest_word(unprocessed_text, self.boosted_togo))
        processed_text.extend(self.find_closest_word(unprocessed_text, self.boosted_howlong))
        processed_text.extend(self.find_closest_word(unprocessed_text, self.boosted_where))
        processed_text.extend(self.find_closest_word(unprocessed_text, self.boosted_yes))
        processed_text.extend(self.find_closest_word(unprocessed_text, self.boosted_no))
        processed_text.extend(self.find_closest_word(unprocessed_text, self.boosted_change_dst))
        processed_text.extend(self.find_closest_word(unprocessed_text, self.boosted_stop))
        user_text = ""
        
        #중복 제거 
        processed_text = list(set(processed_text))

        self.get_logger().info(f"추출된 키워드: {processed_text}")
        for item in processed_text:
            user_text += item + " "
        self.get_logger().info(f"user text stripped: {user_text}")
        # 가공된 문장으로 의도 분류 시작 
        intent_info = self.classify_intent(user_text) #user_text : list 
        
        return intent_info # list 리턴 

    # 의도 분류 및 목적지 추출 함수, 핵심 
    def classify_intent(self, user_text):
         #"가줘", "가자", "가고", "데려다줘", "이동", "갈래", "가고싶어", "가야돼",
        # 키워드 매칭을 위한 의도 키워드 사전
        intent_keywords = {
            "tutorial" : ["튜토리얼"],
            "set_destination": [
               
                                "새천년관", "신공학관", "공학관", "공대", "학생회관","청심대", "법학관", "종강", 
                                "종합강의동", "수의대", "수의학관", "동생대", "동물생명과학관", "입학정보관"],
            "change_dst" : ["바꿔", "목적지", "변경", "바꿔줘", "바꿀래",  "변경할", "변경해", "다시" ],
            "get_eta": ["까지", "몇", "분", "얼마나", "도착", "시간", "얼마", "남았어", "걸려", "언제", "예상"],
            "get_location": ["현재", "위치", "어디", "어디를", "어디야", "지금", "지나고", "지나"],
            "confirm_yes" : ["네", "예", "어", "그래", "응","맞아", "맞습니다", "그렇습니다", "좋아","조아"],
            "confirm_no" : ["아니오", "아니", "아닌데","틀려", "틀렸어", "아니야"],
            "stop" : ["정지", "멈춰", "정지해", "멈춰줘"],
            
        }

        
        # 목적지 수령 self.dst = string 
        for dst in self.boosted_dst:
            user_text = user_text.strip() # 공백 제거
            if dst in user_text:
                self.dst = dst
                break
            else: self.dst = "unknown" # 목적지 수령 실패 시
            
            

        # 기타 의도 수령
        for key, keywords in intent_keywords.items():
            if any(k in user_text for k in keywords):
                self.intent = key
                break

        if self.intent == None or self.intent == "unknown": 
            self.intent = "unknown"
            self.get_logger().info("알 수 없는 요청")
            
            req = IntentResponse.Request()
            req.intent = self.intent
            req.dst = 255 # default: 255 (uint8)
            future = self.main_intent_client.call_async(req) 
            self.memory_clear()
        elif self.intent == "tutorial": # 튜토리얼 의도
            self.get_logger().info("튜토리얼 시작")
            req = IntentResponse.Request()
            req.intent = self.intent
            req.dst = 255 # default: 255 (uint8)
            future = self.main_intent_client.call_async(req) # TTS 노드로 튜토리얼 의도 전송

        # 목적지 설정으로 분류됐을 경우 
        elif self.intent == "set_destination":
            self.get_logger().info(f"목적지 설정: {self.dst}")
            
            if self.dst == None or self.dst == "unknown": # 목적지 수령 실패 시
                self.get_logger().info("목적지 수령 실패, 기본 목적지로 설정")
                self.response_state = "IDLE"
                req = IntentResponse.Request()
                req.intent = "unknown"
                req.dst = 255 # default: 255 (uint8)
                
            else:
                self.response_state = 'REQUEST_CONFIRM'
                req = IntentResponse.Request()
                req.intent = self.intent
                req.dst = self.dst_dict[self.dst]
                
                
            future = self.main_intent_client.call_async(req)
            future.add_done_callback(self.tts_response_callback)
            self.memory_clear()

        # replanning으로 분류됐을 경우, 주행 중에만 가능 
        elif self.intent == "change_dst":
            self.replanning = True
            
            self.response_state = 'REPLANNING'

            req = IntentResponse.Request()
            req.intent = self.intent
            req.dst = 255

            future = self.main_intent_client.call_async(req)
            self.memory_clear()

        # 주행 중 질의 
        elif self.intent == "get_eta" or self.intent == "get_location": 
            if self.response_state == 'CONFIRMED':
                print("hmi planning 노드로 상태 보내주기 필요 ")
                planning_req = IntentToPlanning.Request()
                planning_req.intent = self.intent
                
                future = self.intent_planning_client.call_async(planning_req)
                future.add_done_callback(self.planning_response_callback)

        # 긍/부정 판단 
        elif self.intent == "confirm_yes" or self.intent == "confirm_no":
            if (self.response_state != 'WATING_CONFIRM'): # 확인 요청, WATING_CONFIRM 상태가 아닐 때는 무시
                return
            else: #WATING_CONFIRM 상태일 때(목적지 설정 확인용)
                if self.intent == "confirm_yes":
                    self.response_state = "CONFIRMED"
                    self.confirm_action(True) #의도 전송
                    
                else:
                    self.response_state = 'DENIED'
                    self.confirm_action(False) #의도 전송
       
        elif self.intent == "stop":
            self.get_logger().info("정지 요청")
            req = IntentResponse.Request()
            req.intent = self.intent
            req.dst = 255

            future = self.main_intent_client.call_async(req)
        else :
            self.get_logger().info("알 수 없는 요청")
            
            req = IntentResponse.Request()
            req.intent = "unknown"
            req.dst = 255 # default: 255 (uint8)

            future = self.main_intent_client.call_async(req)

    def confirm_action(self, yesorno):
        
        req = IntentResponse.Request()
        req.intent = "confirm_yes" if yesorno else "confirm_no"
        
        if self.dstmemory == "unknown" or self.dstmemory == None: # 목적지 수령 실패 시
            req.dst = 255
            self.response_state = "IDLE"
            self.get_logger().info("목적지 수령 실패 IDLE로 전환")
        else:
            req.dst = self.dst_dict[self.dstmemory]
        future = self.main_intent_client.call_async(req)
        future.add_done_callback(self.final_response_callback)

    def tts_response_callback(self, future):
        try:
            response = future.result()
            if response.response_code == 255:
                self.get_logger().error("TTS 노드 응답 실패: 잘못된 응답 코드")
                self.response_state = "IDLE"
            else: 
                self.response_state = "WATING_CONFIRM" 
                self.get_logger().info(f"!!!!!!!TTS 출력 완료, WATING_CONFIRM !!!!!!")
        except Exception as e:
            self.get_logger().error(f"서비스 호출 실패: {e}")
            self.response_state = "IDLE"
    
    def final_response_callback(self, future):
        try:
            response = future.result()
            if self.response_state == "CONFIRMED":
                #목적지 확정 후 planning 노드로 목적지 전송 
                encoded_dst= self.dst_dict[self.dstmemory] 
                msg = IntentToPlanningmsg()
                if self.replanning: 
                    msg.intent = "change_dst_confirmed"
                    self.replanning = False
                else: 
                    msg.intent = "set_destination_confirmed"
                msg.building_id = encoded_dst
                self.building_id_pub.publish(msg) #목적지 전송
                self.get_logger().info(f"목적지 전송: {encoded_dst}")
                self.response_state = "COMPLETED"
                self.get_logger().info("안내 시작.")
                self.memory_clear()

            else: 
                self.get_logger().info("응답 수신: 거부")
                self.response_state = "IDLE"
                self.memory_clear()

        except Exception as e:
            self.get_logger().error(f"서비스 호출 실패: {e}")
            self.response_state = "IDLE"
            self.memory_clear()
             

    
    def planning_response_callback(self,future):
        # planning 노드로부터 응답을 받는 콜백 함수(서비스 체이닝)
        response = future.result()
        self.get_logger().info(f"hmi_planning 노드로부터 서비스 응답 확인")
        
        tts_req = IntentToTTS.Request() 
        tts_req.intent = response.intent
        tts_req.estimated_time_remaining = response.estimated_time_remaining
        tts_req.closest_landmark = response.closest_landmark
        
        self.intent_tts_client.call_async(tts_req)
        self.get_logger().info(f"tts 노드로 서비스 호출: {tts_req.intent}")

    def find_closest_word(self, text, boosted_list):
        match = []
        for word in text.split():
            match.extend(difflib.get_close_matches(word, boosted_list, n=1, cutoff=0.66))
        if match:
            return match
        else:
            return []
    def memory_clear(self):
        self.intentmemory = self.intent 
        self.dstmemory = self.dst
        self.intent = None
        self.dst = None

def main(args=None):
    rclpy.init(args=args)
    node = IntentNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

