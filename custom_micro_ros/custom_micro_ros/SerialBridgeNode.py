import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32
from geometry_msgs.msg import TwistWithCovarianceStamped
import serial
import struct
import threading
import math
from ackermann_msgs.msg import AckermannDrive     # ★ 신규
import time
from serial import SerialException

class SerialBridgeNode(Node):
    def __init__(self):
        super().__init__('custom_micro_ros')
        self.get_logger().info("Initializing SerialBridgeNode.")

        # 시리얼 포트 설정 (포트 이름은 환경에 따라 다를 수 있음)
        try:
            self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=0.1)
            self.get_logger().info("Serial port opened successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            return

        # 상태 저장용 변수 (초기화)
        self.last_ros_time = None
        self.last_arduino_time = None
        self.ros_loop_interval = 0.0
        self.arduino_loop_interval = 0.0
        self.ros_loop_hz = 0.0
        self.arduino_loop_hz = 0.0
        self.steer_filtered_angle = 0.0
        self.steer_target_PWM_compensated = 0.0
        self.speed_filtered = 0.0
        self.speed_target_PWM_compensated = 0.0
        self.auto_speed = 0.0
        self.auto_angle = 0.0

        # 초기값 설정
        self.nav2_speed = 0.0
        self.nav2_angle = 0.0
        # 이전 ROS 수신 시간 저장용 (self에 바인딩)
        if not hasattr(self, 'last_ros_time'):
            self.last_ros_time = None
            self.last_arduino_time = None

        self.create_subscription(
            AckermannDrive,
            '/ackermann_cmd',                      # 토픽 이름에 맞춰 수정
            self.ackermann_callback,
            10)
        
        # 퍼블리셔 설정
        self.twist_msgs_pub = self.create_publisher(TwistWithCovarianceStamped, '/encoder/twist', 10)

        # 주기적으로 아두이노에 데이터 전송 (10Hz)
        self.timer = self.create_timer(0.05, self.send_serial_data)

        # 상태 출력용 Timer (0.5초마다)
        self.status_timer = self.create_timer(0.25, self.print_status)

        # 시리얼 수신 스레드 시작
        self.serial_thread = threading.Thread(target=self.read_serial_loop, daemon=True)
        self.serial_thread.start()

    # ──────────────────────────────
    #  Ackermann 콜백
    # ──────────────────────────────
    def ackermann_callback(self, msg):       # ★
        self.nav2_angle = msg.steering_angle
        self.nav2_speed = msg.speed
    
    def convert_to_nav_msgs(self, speed, angle):
        linear_x = speed
        angular_z = speed * math.tan(math.radians(angle)) / 0.72

        twist_stamped = TwistWithCovarianceStamped()
        twist_stamped.header.stamp = self.get_clock().now().to_msg()
        twist_stamped.header.frame_id = 'encoder'  # 또는 'odom', 사용 목적에 따라
        twist_stamped.twist.twist.linear.x = linear_x
        twist_stamped.twist.twist.linear.y = 0.0        
        twist_stamped.twist.twist.linear.z = 0.0
        twist_stamped.twist.twist.angular.x = 0.0
        twist_stamped.twist.twist.angular.y = 0.0
        twist_stamped.twist.twist.angular.z = angular_z
        twist_stamped.twist.covariance = [
            0.01, 0.0,    0.0,    0.0,    0.0,    0.0,
            0.0,    0.01, 0.0,    0.0,    0.0,    0.0,
            0.0,    0.0,    0.01, 0.0,    0.0,    0.0,
            0.0,    0.0,    0.0,    0.01, 0.0,    0.0,
            0.0,    0.0,    0.0,    0.0,    0.01, 0.0,
            0.0,    0.0,    0.0,    0.0,    0.0,    0.05  # yaw만 약간 더 신뢰 낮음
        ]   

        self.twist_msgs_pub.publish(twist_stamped)
        self.get_logger().debug(f'Published Odometry: v={linear_x:.2f} m/s, omega={angular_z:.2f} rad/s')

    def send_serial_data(self):
        try:
            header = b'\xAA\x55'
            payload = struct.pack('<ff', self.nav2_speed, self.nav2_angle)

            # CRC: XOR-based
            crc = 0
            for b in payload:
                crc ^= b

            packet = header + payload + bytes([crc])
            self.serial_port.write(packet)
            self.get_logger().debug(f"[TX] Sent: speed={self.nav2_speed}, angle={self.nav2_angle}, crc={crc}")
        except Exception as e:
            self.get_logger().warn(f"Serial write failed: {e}")

    def print_status(self):
        self.get_logger().info(
            f"\n{'━'*60}\n"
            f"[LOOP]  | ARDUINO: {self.arduino_loop_interval:>2.0f} ms ({self.arduino_loop_hz:>3.1f} hz) | "
            f"ROS2: {self.ros_loop_interval:>3.1f} ms ({self.ros_loop_hz:>3.1f} hz)\n"
            f"[PWM]   | SPEED: {int(self.speed_target_PWM_compensated):>3}       | ANGLE: {int(self.steer_target_PWM_compensated):>3}\n"
            f"[AUTO]  | SPEED: {self.auto_speed:>6.2f}m/s | ANGLE: {self.auto_angle:>6.2f}°\n"
            f"[STATE] | SPEED: {self.speed_filtered:>6.2f}m/s | ANGLE: {self.steer_filtered_angle:>6.2f}°\n"
            f"{'━'*60}\n"
        )

    def read_serial_loop(self):
        HEADER = b'\xAA\x55'
        PACKET_SIZE = 31  # header(2) + payload(20) + crc(1)

        buffer = bytearray()

        while rclpy.ok():
            try:
                if self.serial_port.in_waiting:
                    data = self.serial_port.read(self.serial_port.in_waiting)
                    buffer.extend(data)
                    self.get_logger().debug(f"[Serial] Received {len(data)} bytes. Buffer length: {len(buffer)}")

                while len(buffer) >= PACKET_SIZE:
                    header_index = buffer.find(HEADER)
                    if header_index == -1:
                        self.get_logger().warn("[Sync] Header not found. Clearing buffer.")
                        buffer.clear()
                        break
                    elif header_index > 0:
                        self.get_logger().warn(f"[Sync] Header found at offset {header_index}. Dropping {header_index} bytes.")
                        del buffer[:header_index]

                    if len(buffer) < PACKET_SIZE:
                        self.get_logger().debug("[Wait] Incomplete packet. Waiting for more data.")
                        break

                    # 패킷 준비 완료
                    payload = buffer[2:PACKET_SIZE-1]
                    crc_received = buffer[PACKET_SIZE-1]
                    crc_calculated = 0
                    for b in payload:
                        crc_calculated ^= b

                    if crc_received != crc_calculated:
                        self.get_logger().warn(f"[CRC] Mismatch! Got {crc_received}, expected {crc_calculated}")
                        del buffer[:PACKET_SIZE]
                        continue

                    try:
                        unpacked = struct.unpack('<ff ff ff L', payload)
                        (
                            steer_filtered_angle, steer_target_PWM_compensated,
                            speed_filtered, speed_target_PWM_compensated,
                            auto_speed, auto_angle,
                            last_time,
                        ) = unpacked

                        now_ros_time = time.time() * 1000  # ms 단위 (float)
                        # ROS 루프 주기 계산
                        if self.last_ros_time is None:
                            ros_loop_interval = 0.0
                        else:
                            ros_loop_interval = now_ros_time - self.last_ros_time

                        self.last_ros_time = now_ros_time

                        # 아두이노 제어 주기 계산
                        if self.last_arduino_time is None:
                            arduino_loop_interval = 0
                        else:
                            arduino_loop_interval = last_time - self.last_arduino_time

                        self.last_arduino_time = last_time

                        ros_loop_hz = 1000.0 / ros_loop_interval if ros_loop_interval > 0 else 0.0
                        arduino_loop_hz = 1000.0 / arduino_loop_interval if arduino_loop_interval > 0 else 0.0

                        # 상태 변수 저장만 (출력 X)
                        self.steer_filtered_angle = steer_filtered_angle
                        self.steer_target_PWM_compensated = steer_target_PWM_compensated
                        self.speed_filtered = speed_filtered
                        self.speed_target_PWM_compensated = speed_target_PWM_compensated
                        self.auto_speed = auto_speed
                        self.auto_angle = auto_angle
                        self.ros_loop_interval = ros_loop_interval
                        self.arduino_loop_interval = arduino_loop_interval
                        self.ros_loop_hz = ros_loop_hz
                        self.arduino_loop_hz = arduino_loop_hz

                        # angle, speed 변환 및 퍼블리시
                        self.convert_to_nav_msgs(speed_filtered, steer_filtered_angle)

                    except struct.error as e:
                        self.get_logger().warn(f"[Unpack] Failed to unpack struct: {e}")

                    del buffer[:PACKET_SIZE]
                    self.get_logger().debug(f"[Buffer] Packet processed. Buffer trimmed to {len(buffer)} bytes.")
                    
            except (SerialException, OSError) as e:
                self.get_logger().error(f"[Serial] I/O error: {e}. Reconnecting…")
                # 포트를 닫았다가
                try:
                    self.serial_port.close()
                except:
                    pass
                time.sleep(1.0)
                # 같은 포트로 재연결 시도
                try:
                    self.serial_port.open()
                    self.get_logger().info("Serial port /dev/ttyACM0 reopened.")
                except Exception as ex:
                    self.get_logger().error(f"[Serial] Reopen failed: {ex}")
                continue

            except Exception as e:
                self.get_logger().error(f"[Serial] Unexpected error: {e}")
                time.sleep(0.1)
                continue

            # CPU 부담 완화
            # time.sleep(0.01)

def main(args=None):
    rclpy.init(args=args)
    node = SerialBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down SerialBridgeNode.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
