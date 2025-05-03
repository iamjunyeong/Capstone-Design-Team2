#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, UInt8
import pygame
import time
import threading

class KeyboardNode(Node):
    def __init__(self):
        super().__init__('keyboard_node')
        self.talkbutton_pub = self.create_publisher(Bool, '/talkbutton_state',  10)
        self.handlebutton_pub = self.create_publisher(UInt8, '/handlebutton_state', 10)
        self.emergency_pub = self.create_publisher(Bool, '/emergencybutton_state', 10)
        self.emergency_pressed = False
        self.emergency_active = False
        self.run_keyboard_listener()

    # 키보드 입력 대체 리스너    
    def run_keyboard_listener(self):
        pygame.init()
        screen = pygame.display.set_mode((400, 300))
        pygame.display.set_caption("Keyboard Button Control")
        clock = pygame.time.Clock()

        pressed = False
        emergency_active = False
        talk_pub_time = 0.0
        handle_pub_time = 0.0
        emergency_pub_time = 0.0
        pub_rate = 0.1  # Hz
        
        running = True
        while rclpy.ok() and running:
            rclpy.spin_once(self, timeout_sec=0.0)
            screen.fill((255, 255, 255))

            keys = pygame.key.get_pressed()
            now = time.time()

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

            # 비상 정지: '2' 키
            
            if keys[pygame.K_2]:
                if not self.emergency_pressed:
                    self.emergency_pressed = True  # 키가 눌렸다는 플래그 설정
            else:
                if self.emergency_pressed:
                    # 키가 떨어진 순간, 즉 눌렀다 뗀 시점 → 토글 조건
                    self.emergency_active = not self.emergency_active
                    state = "비상 정지 시작" if self.emergency_active else "비상 정지 해제"
                    self.emergency_pressed = False  # 상태 초기화   

            if now - emergency_pub_time > pub_rate:     
                self.publish_emergency(self.emergency_active)
                emergency_pub_time = now
    
            # 음성 상호작용: '1' 키 (주기적 발행)
            if keys[pygame.K_1]:
                pressed = True
            else:
                pressed = False

            if now - talk_pub_time > pub_rate:
                self.publish_talk(pressed)
                talk_pub_time = now

            # 핸들 버튼 코드: 'a', 's' 키 감지
            code = 0
            if keys[pygame.K_a] and keys[pygame.K_s]:
                code = 2
            elif keys[pygame.K_a] or keys[pygame.K_s]:
                code = 1
            if now - handle_pub_time > pub_rate:
                self.publish_handle(code)
                handle_pub_time = now
            
            pygame.display.flip()
            clock.tick(120)

        pygame.quit()

    def publish_talk(self, state: bool):
        self.talkbutton_pub.publish(Bool(data=state))
        status = "음성 상호작용 시작" if state else "음성 상호작용 종료"
        #self.get_logger().info(f"[TALK] {status}")
    def publish_handle(self, state: int):
        self.handlebutton_pub.publish(UInt8(data=state))
        #self.get_logger().info(f"[HANDLE] {state}")
    
    def publish_emergency(self, state: bool):
        self.emergency_pub.publish(Bool(data=state))
        status = "비상 정지 시작" if state else "비상 정지 해제"
        #self.get_logger().info(f"[EMERGENCY] {status}")


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardNode()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
