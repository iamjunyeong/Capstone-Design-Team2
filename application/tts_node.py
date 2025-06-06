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
            1: ', 안내사항',
            2: ', 버튼을 누르고 목적지를 말씀해주세요',
            
            4: f', 안내 서비스를 시작합니다. 손잡이를 잡아주세요',
            5: ', 목적지 변경, 현재 위치 확인, 정지, 중 말씀해주세요',
            6 : ', 정지하겠습니다.',
            7: ', 가속하겠습니다.',
            8: ', 감속하겠습니다.',
            9: ', 보행 중입니다. 주의하여주세요.',
            10: ', 양쪽 손잡이를 꼭 잡아주세요.',
            11: ', 잠시 정지하겠습니다. 주행을 시작하려면 손잡이를 바르게 잡아주세요.',
            12: ', 비상정지합니다. 비상 정지합니다.',
            13: ', 주행이 완료되었습니다. 주차구역에서 대기하겠습니다.',
            14: ', 네'
        }
        #변수 선언
        self.intent = None
        self.dst_dict = {
            1: "신공학관",
            2: "공학관",
            3: "새천년관",
            4: "학생회관",
            5: "법학관",
            6: "정문",
            7: "후문",
            8: "인문학관",
            9: "경영관"
        }
        self.talkbutton_pressed = False
        self.is_playing = False  # 현재 음성이 재생 중인지 여부
        self.last_talkbutton_state = False
        self.emergencybutton_pressed = False
        self.handlebutton_code = 0 
        self.lock = threading.Lock()
        self.driving_state = 'WAITING'
        self.vision_obstacle_info = 0  # 0(장애물 없음), 1(정적), 2(동적),3(둘다)
        self.sound_file = "response.mp3"  # 효과음 파일 경로
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
        self.handlebutton_sub = self.create_subscription(Bool, '/handlebutton_state', self.handlebutton_callback,10)
        self.emergencybutton_sub = self.create_subscription(Bool, '/emergency', self.emergency_button_callback,10)
        self.vision_obstacle_info_sub = self.create_subscription(Int8, '/obstacle_info', self.vision_callback, 10)  # 장애물 정보 수신용

        # vision/obstacle_info 값 받아오는 sub 필요, callback에서 9번 출력 
        # (보류) 속도조절 스위치 값 받아오는 sub 필요, callback에서 조건에 따라 7,8번 출력
        #srv 
        self.req_server = self.create_service(IntentResponse, '/confirm_service', self.intent_confirm_callback)
        self.intent_tts_server = self.create_service(IntentToTTS, '/intent_to_tts_plan', self.intent_tts_callback)

        self.get_logger().info('TTS Node thread has started.')

    def intent_confirm_callback(self, request, response):
        self.intent = request.intent
        if request.dst == 255 :
            dst = None 
        else:
            dst = self.dst_dict[request.dst]
        
        if self.intent == "set_destination":
            
            self.get_logger().info(f"요청 수신: 목적지={dst})")
            self.clear_queue()
            self.request_queue.put((1, f", 목적지를 {dst} 으로 설정할까요? 맞으면 버튼을 누르고 '네', 아니면 '아니요'라고 말씀해주세요."))
            self.get_logger().info(f"목적지를 {dst} 으로 설정할까요? 맞으면 버튼을 누르고 '네', 아니면 '아니요'라고 말씀해주세요.")
            response.response_code = 1
            

        elif self.driving_state == "DRIVING" and self.intent == "change_dst":
            ######re-planning ##########
            self.get_logger().info(f"요청 수신: 목적지 변경")
            self.clear_queue()
            self.request_queue.put((1, f" 버튼을 누르고 변경하실 목적지를 말씀해주세요"))
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

        elif self.intent == "stop":
            self.clear_queue()
            self.request_queue.put((0, f", {self.output_text[6]}"))
            self.get_logger().info(f", {self.output_text[6]}")
            response.response_code = 5
            #self.driving_state = 'STOP'



            #목적지 설정 시퀀스 재진입
        elif self.intent == "unknown":
            self.get_logger().info(f"요청 수신: {self.intent}")
            self.clear_queue()
            self.request_queue.put((1, self.output_text[0])) 
            self.get_logger().info("이해하지 못했습니다. 다시 말씀해주시겠어요?")

            response.response_code = 0
        
        return response
        
    def handlebutton_callback(self, msg):
        self.handlebutton_code = msg.data
        if self.handlebutton_code == 2:
            #정상주행.
            pass
        elif self.handlebutton_code == 0 or self.handlebutton_code == 1:
            #손잡이 잡아달라는 안내.
            
            self.request_queue.put((0, self.output_text[10]))
            
    def talkbutton_callback(self, msg):
        if msg.data and not self.last_talkbutton_state:
            self.last_talkbutton_state = True  # 눌림 감지
            
            self.stop_and_clear_queue()  # 현재 재생 중인 음성 정지
            if self.driving_state == 'DRIVING':
                if self.intent == "change_dst":

                    self.effect_soundplay(sound_file=self.sound_file)
                    self.get_logger().info("효과음")
                else: 
                    self.request_queue.put((0, f", {self.output_text[5]}"))
                    self.get_logger().info("목적지 변경, 현재 위치 확인, 예상 시간 확인, 정지 기능이 있습니다. 말씀해주세요.")
            else:
                self.effect_soundplay(sound_file=self.sound_file)
                self.get_logger().info("효과음")

        elif not msg.data:
            self.last_talkbutton_state = False  # 버튼 떨어짐

    def emergency_button_callback(self, msg):
        current_state = msg.data

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
        last_obstacle_info = self.vision_obstacle_info
        self.vision_obstacle_info = msg.data
        
        if self.vision_obstacle_info == 0: #장애물 없음
            self.get_logger().info("장애물 없음")
        elif self.vision_obstacle_info == 1: #정적 장애물
            self.get_logger().info("정적 장애물 감지")
        
        elif self.vision_obstacle_info == 2: #동적 장애물
            self.get_logger().info("동적 장애물 감지")
        elif self.vision_obstacle_info == 3: #둘다
            self.get_logger().info("정적 및 동적 장애물 감지")




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

            # intent에 따라 출력할 문장 구성
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