#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, UInt8, Int8
from gtts import gTTS
import queue
import threading
import pygame
import time
from hmi_interface.srv import IntentResponse
from hmi_interface.srv import IntentToTTS
class TTSNode(Node):
    def __init__(self):
        super().__init__('tts_node')
        
        #출력문구 딕셔너리
        self.output_text = {
            0: ', 이해하지 못했습니다. 다시 말씀해주시겠어요?',
            1: ', 안녕하세요, 보행 보조 모빌리티입니다.',
            2: ', 버튼을 누르고 목적지를 말씀해주세요',
            
            4: f', 안내 서비스를 시작합니다. 손잡이를 잡아주세요',
            5: ', 목적지 변경, 현재 위치 확인, 정지, 중 말씀해주세요',
            6 : ', 정지하겠습니다.',
            7: ', 가속하겠습니다.',
            8: ', 감속하겠습니다.',
            9: ', 보행 중입니다. 주의하여주세요.',
            10: ', 양쪽 손잡이를 꼭 잡아주세요.',
            11: ', 손잡이가 떨어져 정지하겠습니다. ',
            12: ', 비상정지합니다. 비상 정지합니다.',
            13: ', 주행이 완료되었습니다. 주차구역에서 대기하겠습니다.',
            14: ', 네',
            15: ', 전방에 장애물이 있습니다.'
        }
        self.tutorial_output = { 
            0 : ', 주행을 시작하시려면 손잡이 가운데 버튼을 눌러주세요. 스킵하려면 비상정지 버튼을 오른쪽으로 돌려 해제해주세요',
            1 : ', 튜토리얼을 시작하겠습니다.' , 
            2 : ', 먼저, 양쪽의 손잡이를 잡고 앞쪽에 만져지는 버튼 두개를 동시에 눌러주세요. ',
            3 : ', 잘 하셨습니다. 안전을 위해 주행중에는 이 손잡이 버튼들을 모두 누른 상태를 유지해주세요. ',  
            4 : ', 다음은 음성인식 버튼입니다. 왼쪽 손잡이 엄지 부분의 버튼을 누른 채로, 튜토리얼 이라고 말씀해주세요. ',
            5 : ', 잘 하셨습니다. 주행 중에도 음성인식 버튼을 누른 채로 저에게 말씀해주시면 됩니다.', 
            6 : ', 손잡이 중앙에 있는 비상정지 버튼을 오른쪽으로 돌려 해제해주세요. 이 버튼은 주행 중 누르면 주행이 중지됩니다.' ,
            7 : ', 튜토리얼이 종료되었습니다. 운행을 시작하시려면 음성인식 버튼을 누른 채로 목적지를 말씀해주세요. ',
        }
        #변수 선언
        self.intent = "unknown"
        self.dst_dict = {
            1: "신공학관",
            5: "공학관",
            8: "학생회관",
            9: "청심대",
            11: "법학관",
            15: "수의학관",
            18: "동물생명과학관",
            20: "입학정보관",
            
        }
        
        self.is_playing = False  # 현재 음성이 재생 중인지 여부
        self.talkbutton_state = False
        self.emergencybutton_pressed = False
        self.handlebutton_code = 0 
        self.lock = threading.Lock()
        self.driving_state = 'WAITING'
        self.vision_obstacle_info = 0  # 0(장애물 없음), 1(정적), 2(동적),3(둘다)
        self.sound_file = '/home/ubuntu/capstone_ws/src/Capstone-Design-Team2/application/effect.mp3'
        self.last_output_time = {}
        self.heartbeat = 0 # heartbeat 값 초기화
        self.tutorial_step = 0  # 튜토리얼 단계 초기화
        self.tutorial_last_time = 0
        # pygame mixer 초기화
        try:
            pygame.mixer.init()
            self.get_logger().info("----------------pygame mixer 초기화 완료------------------")
        except Exception as e:
            self.get_logger().error(f"$$$$$$$$$$$$$$$$$pygame mixer 초기화 실패:$$$$$$$$$$$$$$$$$$$$ {e}")
        # TTS 재생 우선순위 설정을 위한 큐 구조 
        self.request_queue = queue.PriorityQueue()
        
        #재생 스레드 시작 
        self.playback_thread = threading.Thread(target=self.process_queue) #스레드가 실행될 때 함수 실행하라는 의미
        self.playback_thread.daemon = True # 데몬 스레드는 메인 프로그램이 종료될 때 같이 종료
        self.playback_thread.start()
        
        # SUB 
        self.talkbutton_sub = self.create_subscription(Bool, '/talkbutton_pressed', self.talkbutton_callback,10)
        self.handlebutton_sub = self.create_subscription(UInt8, '/handlebutton_state', self.handlebutton_callback,10)
        self.emergencybutton_sub = self.create_subscription(Bool, '/emergency', self.emergency_button_callback,10)
        self.vision_obstacle_info_sub = self.create_subscription(Int8, '/obs_info', self.vision_callback, 10)  # 장애물 정보 수신용
        self.heartbeat_pub = self.create_publisher(UInt8, '/heartbeat/tts_node', 10)  # heartbeat 퍼블리셔
        # vision/obstacle_info 값 받아오는 sub 필요, callback에서 9번 출력 
        # (보류) 속도조절 스위치 값 받아오는 sub 필요, callback에서 조건에 따라 7,8번 출력
        #srv 
        self.req_server = self.create_service(IntentResponse, '/confirm_service', self.intent_confirm_callback)
        self.intent_tts_server = self.create_service(IntentToTTS, '/intent_to_tts_plan', self.intent_tts_callback)

        self.create_timer(1.0, self.heartbeat_callback)  # 1초마다 heartbeat 콜백 호출
        self.get_logger().info('TTS Node thread has started.')
        
    def intent_confirm_callback(self, request, response):
        self.intent = request.intent
        if request.dst == 255 :
            dst = None 
        else:
            dst = self.dst_dict[request.dst]
        if self.driving_state != 'TUTORIAL':
            if self.intent == "set_destination":
                if dst == None:
                    self.get_logger().info(f"요청 수신: 목적지 불분명")
                    self.stop_and_clear_queue()
                    self.request_queue.put((1, f", 목적지를 다시 말씀하여주세요"))
                    response.response_code = 255 
                else :
                    self.get_logger().info(f"요청 수신: 목적지={dst})")
                    self.stop_and_clear_queue()
                    self.request_queue.put((1, f", 목적지를 {dst} 으로 설정할까요? 맞으면 버튼을 누르고 '네', 아니면 '아니요'라고 말씀해주세요."))
                    self.get_logger().info(f"목적지를 {dst} 으로 설정할까요? 맞으면 버튼을 누르고 '네', 아니면 '아니요'라고 말씀해주세요.")
                    response.response_code = 1
                
            elif self.driving_state == "DRIVING" and self.intent == "change_dst":
                ######re-planning ##########
                self.get_logger().info(f"요청 수신: 목적지 변경")
                self.stop_and_clear_queue()
                self.request_queue.put((0, f" 버튼을 누르고 변경하실 목적지를 말씀해주세요"))
                self.get_logger().info("버튼을 누르고 변경하실 목적지를 말씀해주세요")
                response.response_code = 2
                
            elif self.driving_state == 'DRIVING' and (self.intent == "get_location" or self.intent == "get_eta"):
                #서비스에서 처리함
                pass
        
            elif self.intent == "confirm_yes":
                self.clear_queue()
                self.request_queue.put((0, f", {dst} 으로 안내를 시작합니다. 손잡이를 꼭 잡아주세요"))
                self.get_logger().info(f", {dst} 으로 안내를 시작합니다. 손잡이를 꼭 잡아주세요")
                response.response_code = 3 
                
                self.driving_state = 'DRIVING'

            elif self.intent == "confirm_no":
                
                self.clear_queue()
                self.request_queue.put((0,f"목적지를 정확히 말씀해주세요"))
                self.get_logger().info("목적지를 정확히 말씀해주세요")
                response.response_code = 4

            elif self.driving_state =='DRIVING' and self.intent == "stop":
                self.stop_and_clear_queue()
                self.request_queue.put((0, f", {self.output_text[6]}"))
                self.get_logger().info(f", {self.output_text[6]}")
                response.response_code = 5
                self.driving_state = 'STOP'

                #목적지 설정 시퀀스 재진입
            elif self.intent == "unknown":
                self.get_logger().info(f"요청 수신: {self.intent}")
                self.stop_and_clear_queue()
                self.request_queue.put((0, self.output_text[0])) 
                self.get_logger().info("이해하지 못했습니다. 다시 말씀해주시겠어요?")
        else: 
            if self.intent == "unknown":
                self.get_logger().info(f"요청 수신: {self.intent}")
                self.stop_and_clear_queue()
                self.request_queue.put((0, "이해하지 못했어요. 버튼을 누른 채 튜토리얼 이라고 말해보세요")) 
                self.get_logger().info("이해하지 못했어요. 버튼을 누른 채로 튜토리얼 ")
                response.response_code = 0     
        return response
    
    # 튜토리얼 시퀀스 실행
    # 튜토리얼 단계별로 음성 안내를 출력하고, 버튼 입력을 기다리는 로직
    def run_tutorial_sequence(self):
        now = time.time()
        
        if not self.emergencybutton_pressed:
            self.driving_state = 'TUTORIAL_OVER'
            self.stop_and_clear_queue()  # 현재 재생 중인 음성 정지
            self.get_logger().info("비상정지 해제로 인한 종료")
            self.request_queue.put((1, self.tutorial_output[7]))
            return
        
        if self.tutorial_step == 0:
            if self.can_output(201, cooldown=30):
                self.stop_and_clear_queue()  # 현재 재생 중인 음성 정지
                self.request_queue.put((1, self.tutorial_output[1]))  # 튜토리얼 시작 안내 
            self.tutorial_step += 1
            self.tutorial_last_time = now

        elif self.tutorial_step == 1 :
            if self.can_output(202, cooldown=30):
                
                self.request_queue.put((1, self.tutorial_output[2]))  # 손잡이 버튼 3개 동시에 안내
            self.tutorial_step += 1

        elif self.tutorial_step == 2:
            if self.handlebutton_code == 1:  # 3개 버튼을 동시에 누르면 1로 publish된다고 가정
                if self.can_output(203, cooldown=30):
                    self.stop_and_clear_queue()
                    self.request_queue.put((1, self.tutorial_output[3]))  # 잘 했습니다 안내
                self.tutorial_step += 1
                self.tutorial_last_time = now

        elif self.tutorial_step == 3 and now - self.tutorial_last_time > 2:
            if self.can_output(204, cooldown=15) and self.intent !="tutorial":
                self.request_queue.put((1, self.tutorial_output[4]))  # 음성인식 버튼 안내
            if self.intent == "tutorial":
                if self.can_output(205, cooldown=15):
                    self.stop_and_clear_queue()
                    self.request_queue.put((1, self.tutorial_output[5]))
                    self.tutorial_step += 1

        elif self.tutorial_step == 4:
            if self.can_output(206, cooldown=20):
                self.request_queue.put((1, self.tutorial_output[6]))
            self.tutorial_step += 1
            self.tutorial_last_time = now

        elif self.tutorial_step == 5:
            if not self.emergencybutton_pressed:  # 비상정지 해제됨
                self.request_queue.put((1, self.tutorial_output[7]))
                self.driving_state = 'TUTORIAL_OVER'  # 튜토리얼 종료 상태로 변경
                self.tutorial_step = -1  # 튜토리얼 종료

    def can_output(self, code, cooldown=10):
        """지정한 코드가 처음이면 True, 이후엔 cooldown 초 이내 출력 방지"""
        current_time = time.time()

        if code not in self.last_output_time:
            self.last_output_time[code] = current_time
            return True  

        last_time = self.last_output_time[code]
        if current_time - last_time >= cooldown:
            self.last_output_time[code] = current_time
            return True

        return False  # 🔸 cooldown 미만이면 False
    
    def handlebutton_callback(self, msg):
        self.handlebutton_code = msg.data
        if self.driving_state == 'DRIVING':
            if self.handlebutton_code == 1:
                # 정상주행
                pass

            elif self.handlebutton_code == 0:
                # 손잡이 해제 안내
                if self.can_output(111, 10):
                    self.stop_and_clear_queue()
                    self.request_queue.put((0, self.output_text[11])) 

            elif self.handlebutton_code == 2:
                # 손잡이 잡아달라는 안내
                if self.can_output(110, 10):
                    self.stop_and_clear_queue()
                    self.request_queue.put((0, self.output_text[10]))

    def talkbutton_callback(self, msg):
        if msg.data and not self.talkbutton_state:
            self.talkbutton_state = True  # 눌림 감지
            
            self.stop_and_clear_queue()  # 현재 재생 중인 음성 정지
            if self.driving_state == 'DRIVING':
                if self.intent == "change_dst" or self.intent == "set_destination":

                    self.effect_soundplay(sound_file=self.sound_file)
                    self.get_logger().info("효과음")
                else: 
                    self.request_queue.put((0, f", {self.output_text[5]}"))
                    self.get_logger().info("목적지 변경, 현재 위치 확인, 예상 시간 확인, 정지 기능이 있습니다. 말씀해주세요.")
            
            else:
                self.effect_soundplay(sound_file=self.sound_file)
                self.get_logger().info("효과음")

        elif not msg.data:
            self.talkbutton_state = False  # 버튼 떨어짐

    def emergency_button_callback(self, msg):
        current_state = msg.data
        
        if self.driving_state in ['WAITING', 'TUTORIAL','TUTORIAL_OVER']:
            self.emergencybutton_pressed = current_state
            return
        # 비상정지 시작 감지 (False → True 전이)
        if current_state and not self.emergencybutton_pressed:
            self.emergencybutton_pressed = True
            
            self.stop_and_clear_queue() 
            self.request_queue.put((0, f"{self.output_text[12]}"))  # 우선순위 0: 비상정지
            self.get_logger().info("비상 정지, 비상 정지")

        # 비상정지 해제 감지 (True → False 전이)
        elif not current_state and self.emergencybutton_pressed:
            self.emergencybutton_pressed = False
            
            # 필요 시 안내 음성 추가
            self.stop_and_clear_queue()  
            self.request_queue.put((0, "비상 정지가 해제되었습니다. 정상 주행을 재개합니다."))
            self.get_logger().info("비상 정지가 해제되었습니다. 정상 주행을 재개합니다.") 
    
    # 비전 장애물 받아오는 코드, 미완성!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!, 비전에서 토픽 고쳐야함. 지금은 obstacle_info 그대로 받아오면 너무 멀때 대처 x.
    def vision_callback(self, msg):
        self.vision_obstacle_info = msg.data

        if self.vision_obstacle_info == 0:
            pass
        elif self.vision_obstacle_info == 1:  # 정적 장애물
            if self.can_output(115 , cooldown=10):  # 5초 간격으로 출력
                self.clear_queue()
                self.request_queue.put((1, self.output_text[15]))
                self.get_logger().info("전방에 장애물이 있습니다.")

        elif self.vision_obstacle_info in (2, 3):  # 동적 장애물
            if self.can_output(109, cooldown=7):  # 5초 간격으로 출력
                self.clear_queue()
                self.request_queue.put((1, self.output_text[9]))
                self.get_logger().info("보행 중입니다. 주의하여주세요")
   
    def process_queue(self):
        """우선순위 큐에서 요청을 꺼내 순차적으로 재생"""
        while True:
            try:
                priority, text = self.request_queue.get(timeout=0.1)
                self.text_to_speech(text)

            except queue.Empty:
                time.sleep(0.05)
 
    def text_to_speech(self, text):
        try:
            file_name = 'output.mp3'
            tts = gTTS(text=text, lang='ko')
            tts.save(file_name)

            # 재생 시작
            if not pygame.mixer.get_init():
                pygame.mixer.init()
            pygame.mixer.music.load(file_name)
            pygame.mixer.music.play()
            self.is_playing = True

            # 재생이 끝날 때까지 대기
            while pygame.mixer.music.get_busy():
                time.sleep(0.1)

            self.is_playing = False

        except Exception as e:
            self.get_logger().error(f"TTS 오류: {e}")
            self.is_playing = False
        finally:
            self.is_playing = False
    
    def effect_soundplay(self, sound_file):
        try:
        # 재생 시작
            if not pygame.mixer.get_init():
                pygame.mixer.init()
            pygame.mixer.music.load(sound_file)
            pygame.mixer.music.play()
            self.is_playing = True

            # 재생이 끝날 때까지 대기
            while pygame.mixer.music.get_busy():
                time.sleep(0.1)

            self.is_playing = False

        except Exception as e:
            self.get_logger().error(f"효과음 출력 오류: {e}")
            self.is_playing = False
        finally:
            self.is_playing = False
    
    def intent_tts_callback(self, request, response):
        """
        intent 노드에서 planning 결과를 전달받아 TTS로 음성 출력하는 서비스 콜백 함수.
        """
        try:
            # 현재 재생 중인 TTS 중단
            self.stop_and_clear_queue()

            # intent에 따라 출력할 문장 구성 .
            if request.intent == "get_eta":
                text = f"도착까지 약 {request.estimated_time_remaining}분 남았습니다."
            elif request.intent == "get_location":
                text = f"{request.closest_landmark} 근처를 지나고 있습니다."

            # TTS 재생
            self.request_queue.put((2, text))
            
            response.success = True
            return response

        except Exception as e:
            self.get_logger().error(f"TTS 처리 중 오류 발생: {e}")
            response.success = False
            return response
    def heartbeat_callback(self):
        """
        1초마다 호출되는 콜백 함수로, TTS 노드의 상태를 확인하고 로그를 출력합니다.
        """
        if self.driving_state == 'WAITING':
            if self.can_output(101, cooldown=20) and self.can_output(200, cooldown=20):
                self.request_queue.put((2,self.output_text[1]))
                self.request_queue.put((2,self.tutorial_output[0]))
            if self.emergencybutton_pressed:
                self.driving_state = 'TUTORIAL'          
                self.tutorial_step = 0
                self.tutorial_last_time = time.time()
                self.get_logger().info("\n" + "="*80)
                self.get_logger().info("🎯🎯🎯        🚀🚀🚀   TUTORIAL SEQUENCE STARTED   🚀🚀🚀        🎯🎯🎯")
                self.get_logger().info("="*80 + "\n")
        elif self.driving_state == 'TUTORIAL':
            self.run_tutorial_sequence()
        elif self.driving_state == 'TUTORIAL_OVER':
            pass
            # if self.can_output(207, cooldown=20):
            #     self.request_queue.put((1, self.tutorial_output[7]))
            
        if self.is_playing:
            self.heartbeat = 1  # 현재 TTS가 재생 중임을 나타냄
        else:
            self.heartbeat = 0
        self.heartbeat_pub.publish(UInt8(data=self.heartbeat))

    def clear_queue(self):
        with self.request_queue.mutex:
            self.request_queue.queue.clear()
    def stop_and_clear_queue(self):
        pygame.mixer.music.stop()
        self.is_playing = False
        self.clear_queue()

def main(args=None):
    rclpy.init(args=args)
    node = TTSNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()