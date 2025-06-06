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

class SerialBridgeNode(Node):
    def __init__(self):
        super().__init__('serial_bridge_node')
        self.get_logger().info("Initializing SerialBridgeNode.")

        # 시리얼 포트 설정 (포트 이름은 환경에 따라 다를 수 있음)
        try:
            self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=0.1)
            self.get_logger().info("Serial port opened successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            return

        # 초기값 설정
        self.speed = 0.0
        self.angle = 0.0
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
        self.mode_pub = self.create_publisher(Int32, '/vehicle/mode', 10)

        # 주기적으로 아두이노에 데이터 전송 (10Hz)
        self.timer = self.create_timer(0.05, self.send_serial_data)

        # 시리얼 수신 스레드 시작
        self.serial_thread = threading.Thread(target=self.read_serial_loop, daemon=True)
        self.serial_thread.start()

    # ──────────────────────────────
    #  Ackermann 콜백
    # ──────────────────────────────
    def ackermann_callback(self, msg):       # ★
        self.angle = msg.steering_angle * 180 / math.pi
        self.speed = msg.speed
    
    def convert_to_nav_msgs(self, speed, angle):
        linear_x = speed
        angular_z = speed * math.tan(math.radians(angle)) / 0.72

        twist_stamped = TwistWithCovarianceStamped()
        twist_stamped.header.stamp = self.get_clock().now().to_msg()
        twist_stamped.header.frame_id = 'encoder'  # 또는 'odom', 사용 목적에 따라
        twist_stamped.twist.twist.linear.x = speed
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
        self.get_logger().debug(f'Published Odometry: v={speed:.2f} m/s, omega={angular_z:.2f} rad/s')

    def send_serial_data(self):
        try:
            header = b'\xAA\x55'
            payload = struct.pack('<ff', self.speed, self.angle)

            # CRC: XOR-based
            crc = 0
            for b in payload:
                crc ^= b

            packet = header + payload + bytes([crc])
            self.serial_port.write(packet)
            self.get_logger().debug(f"[TX] Sent: speed={self.speed}, angle={self.angle}, crc={crc}")
        except Exception as e:
            self.get_logger().warn(f"Serial write failed: {e}")

    def read_serial_loop(self):
        HEADER = b'\xAA\x55'
        PACKET_SIZE = 143  # header(2) + payload(64) + crc(1)

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
                        unpacked = struct.unpack('<iii ifff ffff ffff ifff ffff ffi ffff ff ff L', payload)
                        (
                            mode, aile, thro,
                            pot_val, steer_raw_angle, steer_filtered_angle, steer_target_angle, 
                            steer_error, steer_integral, steer_derivative, steer_pid_output, 
                            steer_pwm, steer_pwm_filtered,steer_cur_compensation,steer_target_PWM_compensated,
                            encoder_count, speed_raw, speed_filtered, speed_target, 
                            speed_error, speed_integral, speed_derivative, speed_pid_output, 
                            speed_ff_output, speed_total_output,speed_is_break,
                            speed_pwm, speed_pwm_filtered,speed_cur_compensation,speed_target_PWM_compensated,
                            remote_speed, remote_angle,
                            auto_speed, auto_angle,
                            last_time,
                        ) = unpacked

                        mode_str = "AUTONOMOUS" if mode == 1 else "MANUAL"
                        if abs(speed_target) < 0.1:
                            speed_match_percent = 0.0  # 정지 상태로 간주하거나 생략
                        else:
                            speed_match_percent = abs(speed_filtered) / abs(speed_target) * 100.0

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

                        self.get_logger().info(
                            f"\nROS2 ← Arduino\n"
                            f"{'━'*98}\n"
                            f"[MODE & REMOTE] | MODE: {mode_str:>12} | AILE: {aile:>5} | THRO: {thro:>5}\n"
                            f"{'━'*98}\n"
                            f"[LOOP INTERVAL] | ARDUINO: {arduino_loop_interval:>4} ms ({arduino_loop_hz:>4.1f} Hz) | "
                            f"ROS: {ros_loop_interval:>5.1f} ms ({ros_loop_hz:>4.1f} Hz)\n"
                            f"{'━'*98}\n"
                            f"[POTENTIOMETER] | COUNT: {pot_val:>8} | ANGLE: {steer_raw_angle:>9.2f}° | ANGLE_LPF: {steer_filtered_angle:>9.2f}° | TARGET: {steer_target_angle:>9.2f}°\n"
                            f"                | ERROR: {steer_error:>8.2f} | INT: {steer_integral:>12.2f} | DERIV: {steer_derivative:>14.2f} | PID: {steer_pid_output:>13.2f}\n"
                            f"                | PWM: {steer_pwm:>10.2f} | PWM_LPF: {steer_pwm_filtered:>8.2f} | COMP: {steer_cur_compensation:>15.2f} | FINAL_PWM: {int(steer_target_PWM_compensated):>7}\n"
                            f"{'━'*98}\n"
                            f"[ENCODER]       | COUNT: {encoder_count:>8} | SPEED: {speed_raw:>7.4f}m/s | SPEED_LPF: {speed_filtered:>7.4f}m/s | TARGET: {speed_target:>7.4f}m/s\n"
                            f"                | ERROR: {speed_error:>8.4f} | INT: {speed_integral:>12.4f} | DERIV: {speed_derivative:>14.4f} | PID: {speed_pid_output:>13.4f}\n"
                            f"                | FF: {speed_ff_output:>11.4f} | TOTAL: {speed_total_output:>10.4f} | SPEED/TARGET: {speed_match_percent:>6.2f}% | BREAK: {speed_is_break:>11}\n"
                            f"                | PWM: {speed_pwm:>10.2f} | PWM_LPF: {speed_pwm_filtered:>8.2f} | COMP: {speed_cur_compensation:>15.2f} | FINAL_PWM: {int(speed_target_PWM_compensated):>7}\n"
                            f"{'━'*98}\n"
                            f"[REMOTE CMD]    | SPEED: {remote_speed:>8.4f}m/s | ANGLE: {remote_angle:>8.4f}°\n"
                            f"[AUTO CMD]      | SPEED: {auto_speed:>8.4f}m/s | ANGLE: {auto_angle:>8.4f}°\n"
                            f"[CURRENT STATE] | SPEED: {speed_filtered:>8.4f}m/s | ANGLE: {steer_filtered_angle:>8.4f}°\n"
                            f"{'━'*98}"
                        )

                        # angle, speed 변환 및 퍼블리시
                        self.convert_to_nav_msgs(speed_filtered, steer_filtered_angle)

                        # mode publish
                        self.mode_pub.publish(Int32(data=mode))

                    except struct.error as e:
                        self.get_logger().warn(f"[Unpack] Failed to unpack struct: {e}")

                    del buffer[:PACKET_SIZE]
                    self.get_logger().debug(f"[Buffer] Packet processed. Buffer trimmed to {len(buffer)} bytes.")
                    
                time.sleep(0.005)

            except Exception as e:
                self.get_logger().warn(f"[Serial Error] Read loop failed: {e}")

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
