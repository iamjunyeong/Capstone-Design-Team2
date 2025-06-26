#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import time  # ✅ 추가

class CrosswalkService(Node):
    def __init__(self):
        super().__init__('crosswalk_service_server')

        self.srv = self.create_service(
            Trigger,
            'detect_crosswalk',
            self.handle_detect_crosswalk
        )
        self.count = 0
        self.get_logger().info('🚦 Crosswalk service (file-read only) ready')

    def handle_detect_crosswalk(self, request, response):
        filename = '/home/ubuntu/capstone_ws/src/Capstone-Design-Team2/vision_bringup/vision_bringup/crosswalk_info_log_30.txt'

        while True:
            self.get_logger().info('📂 TXT 파일 읽는 중...')

            try:
                with open(filename, 'r') as f:
                    lines = [line.strip() for line in f.readlines()]
                    collected_values = [int(v) for v in lines if v in ('0', '1')]
            except FileNotFoundError:
                self.get_logger().warn(f'파일 없음: {filename}')
                response.success = False
                response.message = 'Log file not found'
                return response
            except Exception as e:
                self.get_logger().error(f'파일 읽기 오류: {e}')
                response.success = False
                response.message = 'File read error'
                return response
            # 0 포함 여부 판단
            if any(v == 0 for v in collected_values):
                self.get_logger().info(f'❌ 0 감지됨 → 1초 후 다시 확인')
                time.sleep(1.0)  # ✅ 1초 대기 추가
                count += 1
                continue
            else:
                self.get_logger().info(f'✅ 모두 1 → 응답 반환. [수집값: {collected_values}]')
                response.success = True
                response.message = f'1 (Values: {collected_values})'
                return response
            
            if (count >= 10):
                self.get_logger().info(f'✅ 10번 카운트 오버 그냥 지나갑니다.ㅜㅜㅜㅜㅜㅜㅜㅜㅜㅜㅜㅜㅜㅜㅜㅜㅜㅜㅜㅜㅜㅜㅜㅜㅜㅜㅜㅜㅜㅜㅜㅜㅜㅜ')
                response.success = True
                response.message = f'1 (Values: {collected_values})'
                return response


def main():
    rclpy.init()
    node = CrosswalkService()

    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
