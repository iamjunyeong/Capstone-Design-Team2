# 파일: plotter_final.py

import serial
import threading
import collections
import time
import sys
import csv
import datetime # 시간 기록을 위해 추가
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import matplotlib.dates as mdates # 시간 축 포맷팅을 위해 추가
from matplotlib.animation import FuncAnimation

# --- 설정 ---
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200
MAX_DATA_POINTS = 200
NUM_DATA_STREAMS = 20
DATA_TIMEOUT_SECONDS = 5.0

class SerialDataSource:
    """
    타임아웃 감지 및 CSV 로깅 기능이 추가된 안정적인 시리얼 데이터 관리 클래스.
    """
    def __init__(self, port, baudrate, num_streams):
        self.port = port
        self.baudrate = baudrate
        self.num_streams = num_streams
        self.is_running = False
        self.serial_connection = None
        self.thread = None
        self.data_lock = threading.Lock()
        
        # X축 데이터를 샘플 카운트 대신 datetime 객체로 저장합니다.
        self.time_points = collections.deque(maxlen=MAX_DATA_POINTS)
        self.data_streams = [collections.deque(maxlen=MAX_DATA_POINTS) for _ in range(self.num_streams)]
        
        self.last_receive_time = time.time()
        self.csv_writer = None
        self.csv_file = None
        self.csv_header = []

    def start(self):
        # ... (이전과 동일) ...
        print(f"'{self.port}'에 연결을 시도합니다...")
        try:
            self.serial_connection = serial.Serial(self.port, self.baudrate, timeout=1)
            print("포트 연결 성공. 아두이노 리셋 대기 중...")
            time.sleep(2)
            self.serial_connection.reset_input_buffer()
            
            self.is_running = True
            self.thread = threading.Thread(target=self._read_loop, daemon=True)
            self.thread.start()
            print("데이터 수신 스레드를 시작합니다.")
            return True
        except serial.SerialException as e:
            print(f"오류: 시리얼 포트를 열 수 없습니다. '{e}'")
            return False

    def _read_loop(self):
        # ... (이전과 동일) ...
        try:
            header_line = self.serial_connection.readline().decode('utf-8').strip()
            print(f"수신된 헤더: {header_line}")
            self.csv_header = ['iso_timestamp'] + header_line.split(',')
            self._setup_csv_file()
        except (UnicodeDecodeError, serial.SerialException):
            print("경고: 헤더를 수신하지 못했습니다. 기본 헤더를 사용합니다.")
            self.csv_header = ['iso_timestamp'] + [f'value_{i+1}' for i in range(self.num_streams)]
            self._setup_csv_file()

        self.last_receive_time = time.time()

        while self.is_running:
            if time.time() - self.last_receive_time > DATA_TIMEOUT_SECONDS:
                print(f"경고: {DATA_TIMEOUT_SECONDS}초 동안 데이터 수신 없음. 아두이노 연결 또는 데이터 전송 상태를 확인하세요.")
                self.last_receive_time = time.time() 

            try:
                line = self.serial_connection.readline().decode('utf-8').strip()
                if line:
                    self.last_receive_time = time.time() 
                    parts = line.split(',')
                    if len(parts) == self.num_streams:
                        now = datetime.datetime.now() # 데이터 수신 시점의 시간 기록
                        values = [float(p) for p in parts]
                        
                        if self.csv_writer:
                            self.csv_writer.writerow([now.isoformat()] + values)

                        with self.data_lock:
                            for i in range(self.num_streams):
                                self.data_streams[i].append(values[i])
                            self.time_points.append(now) # 시간 객체를 X축 데이터로 추가
            except (ValueError, UnicodeDecodeError):
                continue
            except serial.SerialException:
                print("오류: 시리얼 연결이 끊어졌습니다.")
                self.is_running = False
    
    def _setup_csv_file(self):
        # ... (이전과 동일) ...
        try:
            filename = f"sensor_log_{time.strftime('%Y%m%d_%H%M%S')}.csv"
            self.csv_file = open(filename, 'w', newline='', encoding='utf-8')
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(self.csv_header)
            print(f"데이터를 '{filename}' 파일에 저장합니다.")
        except IOError as e:
            print(f"오류: CSV 파일을 생성할 수 없습니다. '{e}'")

    def get_data(self):
        # ... (이전과 동일) ...
        with self.data_lock:
            return list(self.time_points), [list(d) for d in self.data_streams]

    def stop(self):
        # ... (이전과 동일) ...
        if self.is_running:
            print("데이터 수신 스레드를 종료합니다...")
            self.is_running = False
            if self.thread:
                self.thread.join()
        if self.serial_connection and self.serial_connection.is_open:
            self.serial_connection.close()
            print("시리얼 포트를 닫았습니다.")
        if self.csv_file:
            self.csv_file.close()
            print("CSV 파일을 닫았습니다.")
            
def main():
    # --- 수정 ---
    # 데이터 스트림 개수가 20개로 늘어났으므로, SerialDataSource 생성자를 수정합니다.
    data_source = SerialDataSource(SERIAL_PORT, BAUD_RATE, NUM_DATA_STREAMS)
    if not data_source.start():
        sys.exit(1)

    # --- 수정 ---
    # 그래프 레이아웃을 2x2로 변경하고, 전체 x축을 공유합니다.
    fig, axes = plt.subplots(2, 2, figsize=(14.5, 10), sharex=True)
    (ax1, ax3), (ax2, ax4) = axes # 2x2 축 배열
    
    # 전체 라벨 목록 (속도 8개 + 조향 12개)
    labels = [
        # Speed (0-7)
        'Filtered Speed (m/s)', 'Target Speed (m/s)', 'PID Output', 'FeedForward Output',
        'Total Output', 'Is Break', 'Filtered PWM', 'Final PWM',
        # Steering (8-19)
        'Potentiometer', 'Raw Angle', 'Filtered Angle', 'Target Angle',
        'Steer Error', 'Steer Integral', 'Steer Derivative', 'Steer PID',
        'Steer Target PWM', 'Steer Filt PWM', 'Steer Comp', 'Steer Final PWM'
    ]

    # --- 수정 ---
    # 기존 속도 그래프 라인 (ax1, ax2)
    speed_lines = [
        ax1.plot([], [], lw=2, label=labels[0])[0],          # Filtered Speed
        ax1.plot([], [], lw=2, linestyle='--', label=labels[1])[0], # Target Speed
        ax2.plot([], [], lw=1, alpha=0.8, label=labels[2])[0],# PID Output
        ax2.plot([], [], lw=1, alpha=0.8, label=labels[3])[0],# FF Output
        ax2.plot([], [], lw=2, label=labels[4])[0],          # Total Output
        ax2.plot([], [], lw=2, label=labels[7])[0]           # Final PWM
    ]

    steer_lines = [
        # ax3: 조향 각도 관련 (기존과 동일)
        ax3.plot([], [], lw=2, label=labels[10])[0],         # Filtered Angle
        ax3.plot([], [], lw=2, linestyle='--', label=labels[11])[0], # Target Angle
        
        # ax4: PID 튜닝 관련 항목들
        ax4.plot([], [], lw=1, alpha=0.7, label=labels[12])[0],# Steer Error
        ax4.plot([], [], lw=1, alpha=0.7, label=labels[13])[0],# Steer Integral
        ax4.plot([], [], lw=1, alpha=0.7, label=labels[14])[0],# Steer Derivative
        ax4.plot([], [], lw=2, alpha=0.9, label=labels[15])[0],# Steer PID Output (추가)
        ax4.plot([], [], lw=2, label=labels[19])[0]           # Steer Final PWM
    ]
    lines = speed_lines + steer_lines # 모든 라인을 하나의 리스트로 관리

    # --- 수정 ---
    # 4개의 모든 축에 대해 설정 적용
    for ax in [ax1, ax2, ax3, ax4]:
        ax.legend(loc='upper left')
        ax.grid(True)

    # 각 축 제목 및 Y라벨 설정
    ax1.set_title('Speed Control')
    ax1.set_ylabel('Speed (m/s)')
    ax2.set_title('Speed Control Output')
    ax2.set_ylabel('Value / PWM')
    # --- 추가 ---
    ax3.set_title('Steering Control')
    ax3.set_ylabel('Angle (deg)')
    ax4.set_title('Steering Control Output')
    ax4.set_ylabel('Value / PWM')
    
    # Y축 범위 적용 (조향 각도 및 PWM 범위는 임의로 설정, 필요시 조절)
    ax1.set_ylim(-1.4, 1.4)
    ax2.set_ylim(-260, 260)
    # --- 추가 ---
    ax3.set_ylim(-35, 35) # 예: -35도 ~ +35도
    ax4.set_ylim(-260, 260)

    # X축 시간 포맷 설정 (하단 그래프들에만 xlabel 표시)
    for ax in [ax2, ax4]:
        ax.set_xlabel('Time (HH:MM:SS)')
        xfmt = mdates.DateFormatter('%H:%M:%S')
        ax.xaxis.set_major_formatter(xfmt)
    fig.autofmt_xdate()

    # --- 수정 ---
    # 실시간 데이터 HUD 설정 (총 4개)
    hud_style = dict(fontsize=9, va='top', ha='right', fontfamily='monospace',
                     bbox=dict(boxstyle='round,pad=0.4', fc='wheat', alpha=0.7))
    
    hud_ax1 = ax1.text(0.98, 0.98, '', transform=ax1.transAxes, **hud_style)
    hud_ax2 = ax2.text(0.98, 0.98, '', transform=ax2.transAxes, **hud_style)
    # --- 추가 ---
    hud_ax3 = ax3.text(0.98, 0.98, '', transform=ax3.transAxes, **hud_style)
    hud_ax4 = ax4.text(0.98, 0.98, '', transform=ax4.transAxes, **hud_style)
    
    all_huds = [hud_ax1, hud_ax2, hud_ax3, hud_ax4]

    fig.tight_layout(pad=2.0)

    def update_plot(frame):
        time_data, streams_data = data_source.get_data()

        if not time_data:
            return lines + all_huds

        # --- 수정 ---
        # 기존 속도 데이터 라인 업데이트 (인덱스 동일)
        speed_lines[0].set_data(time_data, streams_data[0])
        speed_lines[1].set_data(time_data, streams_data[1])
        speed_lines[2].set_data(time_data, streams_data[2])
        speed_lines[3].set_data(time_data, streams_data[3])
        speed_lines[4].set_data(time_data, streams_data[4])
        speed_lines[5].set_data(time_data, streams_data[7])

        # 신규 조향 데이터 라인 업데이트 (ax4 내용 변경)
        # ax3 (각도)
        steer_lines[0].set_data(time_data, streams_data[10]) # Filtered Angle
        steer_lines[1].set_data(time_data, streams_data[11]) # Target Angle
        
        # ax4 (PID 튜닝)
        steer_lines[2].set_data(time_data, streams_data[12]) # Steer Error
        steer_lines[3].set_data(time_data, streams_data[13]) # Steer Integral
        steer_lines[4].set_data(time_data, streams_data[14]) # Steer Derivative
        steer_lines[5].set_data(time_data, streams_data[15]) # Steer PID Output (추가)
        steer_lines[5].set_data(time_data, streams_data[19]) # Steer Final PWM
        
        # --- 수정 ---
        # 모든 축의 데이터 범위를 다시 계산
        for ax in [ax1, ax2, ax3, ax4]:
            ax.relim()
        
        # X축 범위 업데이트
        latest_time = time_data[-1]
        # 표시할 시간 범위를 15초로 늘림
        start_time = latest_time - datetime.timedelta(seconds=15)
        ax1.set_xlim(start_time, latest_time)

        # --- 수정 ---
        # HUD 텍스트 내용 업데이트
        
        # 속도 HUD (ax1, ax2)
        hud_ax1.set_text(
            f"{'Target:':<10}{streams_data[1][-1]:>7.2f} m/s\n"
            f"{'Actual:':<10}{streams_data[0][-1]:>7.2f} m/s"
        )
        break_status = "ON" if streams_data[5][-1] > 0.5 else "OFF"
        hud_ax2.set_text(
            f"{'PID:':<8}{streams_data[2][-1]:>8.1f}\n"
            f"{'FF:':<8}{streams_data[3][-1]:>8.1f}\n"
            f"{'Total:':<8}{streams_data[4][-1]:>8.1f}\n"
            f"{'PWM:':<8}{streams_data[7][-1]:>8.0f}\n"
            f"{'Brake:':<8}{break_status:>8s}"
        )

        # --- 추가 ---
        # 조향 HUD (ax3, ax4)
        hud_ax3.set_text(
            f"{'Target:':<10}{streams_data[11][-1]:>7.2f} deg\n"
            f"{'Actual:':<10}{streams_data[10][-1]:>7.2f} deg\n"
            f"{'Potentiometer:':<8}{streams_data[8][-1]:>7.0f} ADC"
        )
        hud_ax4.set_text(
            f"{'Error:':<10}{streams_data[12][-1]:>8.2f}\n"
            f"{'Integral:':<10}{streams_data[13][-1]:>8.2f}\n"
            f"{'Derivative:':<10}{streams_data[14][-1]:>8.2f}\n"
            f"{'PID Out:':<10}{streams_data[15][-1]:>8.2f}\n"
            f"{'FinalPWM:':<10}{streams_data[19][-1]:>8.0f}"
        )

        return lines + all_huds
    
    # ... (FuncAnimation 호출 및 나머지 부분은 동일) ...
    ani = FuncAnimation(fig, update_plot, interval=50, blit=False, cache_frame_data=False)

    try:
        plt.show()
    except KeyboardInterrupt:
        print("사용자 요청으로 프로그램을 종료합니다.")
    finally:
        data_source.stop()

if __name__ == '__main__':
    main()