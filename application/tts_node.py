#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, UInt8
from gtts import gTTS
from collections import deque
import threading
import pygame
import time
from hmi_interface.srv import IntentResponse

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
        self.dst = None
        self.dst_dict = {
            0: "신공학관",
            1: "공학관",
            2: "새천년관",
            3: "학생회관",
            4: "법학관",
            5: "정문",
            6: "후문",
            7: "인문학관",
            8: "경영관"
        }
        self.talkbutton_pressed = False
        self.is_playing = False  # 현재 음성이 재생 중인지 여부
        self.last_talkbutton_state = False
        self.emergencybutton_pressed = False
        self.handlebutton_code = 0 
        self.lock = threading.Lock()
        self.response_state = 0   # 0:대기 중, 1: 확인 요청 진행 중, 2:확인 요청 완료 3:재질의 들어가기 
        
        # pygame mixer 초기화
        try:
            pygame.mixer.init()
            self.get_logger().info("----------------pygame mixer 초기화 완료------------------")
        except Exception as e:
            self.get_logger().error(f"$$$$$$$$$$$$$$$$$pygame mixer 초기화 실패:$$$$$$$$$$$$$$$$$$$$ {e}")
        # TTS 재생 우선순위 설정을 위한 큐 구조 
        self.request_queue = deque()
        
        # 우선순위에 따른 인터럽트 구현을 위한 변수 
        #self.emergency_stop_event = threading.Event() #비상정지 이벤트 정의 
        self.talkbutton_on = threading.Event() #상호작용 버튼 켜졌을 때 이벤트로 정의
        
        #재생 스레드 시작 
        self.playback_thread = threading.Thread(target=self.process_queue) #스레드가 실행될 때 함수 실행하라는 의미
        self.playback_thread.daemon = True # 데몬 스레드는 메인 프로그램이 종료될 때 같이 종료
        self.playback_thread.start()
        self.get_logger().info('TTS Node has started.')

        # SUB 
        self.talkbutton_sub = self.create_subscription(Bool, '/talkbutton_pressed', self.talkbutton_callback,10)
        self.handlebutton_sub = self.create_subscription(Bool, '/handlebutton_state', self.handlebutton_callback,10)
        self.emergencybutton_sub = self.create_subscription(Bool, '/emergency', self.emergency_button_callback,10)
        
        # vision/obstacle_info 값 받아오는 sub 필요, callback에서 9번 출력 
        # 속도조절 스위치 값 받아오는 sub 필요, callback에서 조건에 따라 7,8번 출력

        #srv 
        self.req_server = self.create_service(IntentResponse, '/confirm_service', self.intent_confirm_callback)

    def intent_confirm_callback(self, request, response):
        self.intent = request.intent
        dst = self.dst_dict[request.dst]
        if self.intent == "unknown":
            self.get_logger().info(f"요청 수신: {self.intent}")
            self.request_queue.append(self.output_text[0]) #이해하지 못했어요. 
            response.response_code = 0

        elif self.intent == "set_destination":
            self.get_logger().info(f"요청 수신: 목적지={dst})")
            self.request_queue.append(f", 목적지를 {dst} 으로 설정할까요? 맞으면 버튼을 누르고 '네', 아니면 '아니요'라고 말씀해주세요.")
            response.response_code = 1
        elif self.intent == "change_dst":
            self.get_logger().info(f"요청 수신: 목적지 변경 (현재 위치={dst})")
            self.request_queue.append(f", 목적지를 {dst}으로 변경할까요? 맞으면 버튼을 누르고 '네', 아니면 '아니요'라고 말씀해주세요.")
            response.response_code = 2
        elif self.intent == "get_location":
            pass
        elif self.intent == "get_eta":
            pass 
        elif self.intent == "confirm_yes":
            self.request_queue.append(f", {dst} 으로 안내를 시작합니다. 손잡이를 꼭 잡아주세요")
            response.response_code = 3 
        elif self.intent == "confirm_no":
            self.request_queue.append(f"목적지를 정확히 말씀해주세요")
            response.response_code = 4
            #목적지 설정 시퀀스 재진입
        return response
        """ """
    def handlebutton_callback(self, msg):
        self.handlebutton_code = msg.data
        if self.handlebutton_code == 2:
            #정상주행.
            pass
        elif self.handlebutton_code == 0 or self.handlebutton_code == 1:
            #손잡이 잡아달라는 안내.
            self.request_queue.append(self.output_text[10])
            
    def talkbutton_callback(self, msg):
        if msg.data and not self.last_talkbutton_state:
            self.last_talkbutton_state = True  # 눌림 감지
            self.get_logger().info("네")

            with self.lock:
                if self.is_playing:
                    pygame.mixer.music.stop()
                    self.is_playing = False
                threading.Thread(target=self.text_to_speech, args=("네",)).start()
        elif not msg.data:
            self.last_talkbutton_state = False  # 버튼 떨어짐
    def emergency_button_callback(self, msg):
        if msg.data:
            self.get_logger().info("비상 정지")
            with self.lock:
                    if self.emergencybutton_pressed:
                        pygame.mixer.music.stop()
                        self.is_playing = False
                    threading.Thread(target=self.text_to_speech, args=("비상 정지, 비상 정지",)).start()
                    
        elif not msg.data:
            self.emergencybutton_pressed = False  # 버튼 떨어짐     
        
    def process_queue(self):
        """큐에서 요청을 꺼내 순차적으로 처리"""
        while True:
            if self.request_queue:
                text = self.request_queue.popleft()
                self.text_to_speech(text)
            else:
                time.sleep(0.05)  # CPU 과점 방지

    def text_to_speech(self, text):
        try:
            time.sleep(0.0)
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

def main(args=None):
    rclpy.init(args=args)
    node = TTSNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

