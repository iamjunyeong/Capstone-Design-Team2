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
NUM_DATA_STREAMS = 8
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
    data_source = SerialDataSource(SERIAL_PORT, BAUD_RATE, NUM_DATA_STREAMS)
    if not data_source.start():
        sys.exit(1)

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10), sharex=True)
    labels = [
        'Filtered Speed (m/s)', 'Target Speed (m/s)', 'PID Output', 'FeedForward Output',
        'Total Output', 'Is Break', 'Filtered PWM', 'Final PWM'
    ]
    lines = [
        ax1.plot([], [], lw=2, label=labels[0])[0],
        ax1.plot([], [], lw=2, linestyle='--', label=labels[1])[0],
        ax2.plot([], [], lw=1, alpha=0.8, label=labels[2])[0],
        ax2.plot([], [], lw=1, alpha=0.8, label=labels[3])[0],
        ax2.plot([], [], lw=2, label=labels[4])[0],
        ax2.plot([], [], lw=2, label=labels[7])[0]
    ]

    for ax in [ax1, ax2]:
        # --- 1. 범례(legend)를 좌측 상단으로 이동 ---
        ax.legend(loc='upper left')
        ax.grid(True)
    ax1.set_title('Speed Control')
    ax1.set_ylabel('Speed (m/s)')
    ax2.set_title('Control Output & PWM')
    
    # Y축 범위 적용
    ax1.set_ylim(-1.4, 1.4)
    ax2.set_ylim(-255, 255)

    ax2.set_xlabel('Time (HH:MM:SS)')
    xfmt = mdates.DateFormatter('%H:%M:%S')
    ax2.xaxis.set_major_formatter(xfmt)
    fig.autofmt_xdate()

    # --- 2. 실시간 데이터 HUD를 우측 상단에 하나의 박스로 생성 ---
    hud_style = dict(transform=ax1.transAxes, fontsize=9, va='top', ha='right',
                     fontfamily='monospace', # 텍스트 정렬을 위해 고정폭 글꼴 사용
                     bbox=dict(boxstyle='round,pad=0.4', fc='wheat', alpha=0.7))
    
    # 상단 그래프(ax1)의 데이터 창
    hud_ax1 = ax1.text(0.98, 0.98, '', **hud_style)
    
    # 하단 그래프(ax2)의 데이터 창
    hud_style['transform'] = ax2.transAxes # 기준 축을 ax2로 변경
    hud_ax2 = ax2.text(0.98, 0.98, '', **hud_style)

    fig.tight_layout()

    def update_plot(frame):
        time_data, streams_data = data_source.get_data()

        if not time_data:
            return lines + [hud_ax1, hud_ax2]

        # 데이터 라인 업데이트
        lines[0].set_data(time_data, streams_data[0])
        lines[1].set_data(time_data, streams_data[1])
        lines[2].set_data(time_data, streams_data[2])
        lines[3].set_data(time_data, streams_data[3])
        lines[4].set_data(time_data, streams_data[4])
        lines[5].set_data(time_data, streams_data[7])
        
        for ax in [ax1, ax2]:
            ax.relim()

        # X축 범위 업데이트
        latest_time = time_data[-1]
        start_time = latest_time - datetime.timedelta(seconds=10)
        ax1.set_xlim(start_time, latest_time)

        # --- 3. HUD 텍스트 내용 업데이트 (이름과 값을 한 줄에 표시) ---
        # 상단 그래프 (속도) 값
        s_target = streams_data[1][-1]
        s_actual = streams_data[0][-1]
        hud_ax1.set_text(
            f"{'Target:':<10}{s_target:>7.2f} m/s\n"
            f"{'Actual:':<10}{s_actual:>7.2f} m/s"
        )

        # 하단 그래프 (제어 출력) 값
        s_pid      = streams_data[2][-1]
        s_ff       = streams_data[3][-1]
        s_total    = streams_data[4][-1]
        s_is_break = streams_data[5][-1]
        s_pwm      = streams_data[7][-1]
        break_status = "ON" if s_is_break > 0.5 else "OFF"
        
        hud_ax2.set_text(
            f"{'PID:':<8}{s_pid:>8.1f}\n"
            f"{'FF:':<8}{s_ff:>8.1f}\n"
            f"{'Total:':<8}{s_total:>8.1f}\n"
            f"{'PWM:':<8}{s_pwm:>8.0f}\n"
            f"{'Brake:':<8}{break_status:>8s}"
        )

        return lines + [hud_ax1, hud_ax2]

    ani = FuncAnimation(fig, update_plot, interval=50, blit=False, cache_frame_data=False)

    try:
        plt.show()
    except KeyboardInterrupt:
        print("사용자 요청으로 프로그램을 종료합니다.")
    finally:
        data_source.stop()

if __name__ == '__main__':
    # ... (리눅스 환경 점검 안내는 이전과 동일) ...
    main()