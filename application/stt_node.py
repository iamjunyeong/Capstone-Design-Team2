import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool,UInt8
import numpy as np
import noisereduce as nr
import threading
import logging
import time
import grpc
import queue
import webrtcvad
import sounddevice as sd
import vito_stt_client_pb2 as pb
import vito_stt_client_pb2_grpc as pb_grpc
from requests import Session
from hmi_interface.msg import Heartbeat
from builtin_interfaces.msg import Time
# === 설정 ===
API_BASE = "https://openapi.vito.ai"
GRPC_SERVER_URL = "grpc-openapi.vito.ai:443"
CLIENT_ID = "0KBBH-jC0UykJe1mdZRM"
CLIENT_SECRET = "4mgz7xDH4DLHVTpkWhaK70oANULIEb26_9RZvPku"

SAMPLE_RATE = 16000
ENCODING = pb.DecoderConfig.AudioEncoding.LINEAR16
CHANNELS = 1
FRAME_DURATION_MS = 30
FRAME_SIZE = int(SAMPLE_RATE * FRAME_DURATION_MS / 1000)
MAX_SILENCE_DURATION = 2

boosted_keywords = [
    "가줘:5.0", "가자:5.0", "가고:1.0", "데려다줘:5.0", "이동:5.0", "갈래:5.0", "가고싶어:5.0", "가야돼:5.0",

    "공학관:5.0","공대:5.0","신공:5.0", "신공학관:5.0", "학생회관:5.0", "청심대:5.0", "법학관:5.0","종강","종합강의동","수의대:5.0", "수의학관:5.0", "동생대:5.0","동물생명과학관:5.0", "입학정보관:5.0",
    
    "바꿔:5.0", "목적지:5.0", "변경:5.0", "바꿔줘", "바꿀래", "변경할", "변경해", "다시", 

    "까지", "몇:3.0","분:2.0", "얼마나:5.0", "도착:5.0", "시간:5.0", "얼마:5.0", "남았어:5.0", "걸려:5.0","언제:5.0","예상:5.0",
     
    "현재:5.0", "위치:5.0", "어디:5.0", "어디를", "어디야:5.0", "지나:5.0", "지금:5.0", "지나고:5.0",

    "네:5.0", "예:5.0", "어:1.0", "그래:5.0", "응:5.0", "맞아:5.0", "맞습니다:5.0", "그렇습니다:5.0", "좋아",

    "아니오:5.0", "아니:5.0", "아닌데:5.0", "틀려:5.0", "틀렸어:5.0", "아니야:5.0",

    "안녕하세요:5.0", "확인:5.0", "정지:5.0", "멈춰:5.0", "정지해:5.0", "멈춰줘:5.0",
]
 
# === 마이크 VAD 스트리머 ===
class VADMicStreamer:
    def __init__(self,stt_node):
        self.q = queue.Queue()
        self.vad = webrtcvad.Vad(2)
        self.max_silence_frames = int(MAX_SILENCE_DURATION * 1000 / FRAME_DURATION_MS)
        self.stt = stt_node
        
    def __enter__(self):
        self.stream = sd.InputStream(
            samplerate=SAMPLE_RATE,
            channels=CHANNELS,
            dtype='int16',
            blocksize=FRAME_SIZE,
            callback=self.callback
        )
        self.stream.start()
        return self

    def callback(self, indata, frames, time_info, status):
        if status:
            print("", status)
        self.q.put(bytes(indata))

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stream.stop()
        self.stream.close()

    def read_stream(self):
        silence = 0
        while True:
            if not self.stt.talkbutton_pressed:
                print("버튼 눌리지 않아 cut함")
                return
            try:
                frame = self.q.get(timeout=0.1)
            except queue.Empty:
                continue

            if len(frame) < 2:
                continue

            # VAD용 원본 그대로 사용
            is_speech = self.vad.is_speech(frame, SAMPLE_RATE)

            # STT용 전처리
            processed_frame = self.preprocess_audio(frame)

            yield processed_frame  # 전처리된 것을 STT에 전송

            if is_speech:
                silence = 0
            else:
                silence += 1
                if silence >= self.max_silence_frames:
                    print("무음 감지, 스트리밍 종료")
                    return

    def preprocess_audio(self, frame):  # 전처리 함수
        try:
            audio_np_int16 = np.frombuffer(frame, dtype=np.int16)

            if len(audio_np_int16) < 480:
                return frame

            def is_all_frames_too_quiet(audio_np, frame_size=480, hop_size=240, threshold_db=60.0):
                for i in range(0, len(audio_np) - frame_size, hop_size):
                    frame = audio_np[i:i+frame_size].astype(np.float32)
                    rms = np.sqrt(np.mean(frame**2) + 1e-10)
                    db = 20 * np.log10(rms)
                    if db >= threshold_db:
                        return False
                return True

            # 너무 조용하면 원본 프레임 그대로 반환
            if is_all_frames_too_quiet(audio_np_int16):
                return frame

            audio_np_float = audio_np_int16.astype(np.float32) / 32767.0

            def highpass_filter(data, cutoff=50, fs=16000, order=5):
                from scipy.signal import butter, lfilter
                b, a = butter(order, cutoff / (0.5 * fs), btype='high', analog=False)
                return lfilter(b, a, data)

            audio_highpassed = highpass_filter(audio_np_float)

            def bandstop_filter(data, lowcut=213.0, highcut=215.0, fs=16000, order=2):
                from scipy.signal import butter, filtfilt
                nyq = 0.5 * fs
                low = lowcut / nyq
                high = highcut / nyq
                b, a = butter(order, [low, high], btype='bandstop')
                return filtfilt(b, a, data)

            audio_bandstopped = bandstop_filter(audio_highpassed)

            reduced_float = nr.reduce_noise(
                y=audio_bandstopped,
                sr=16000,
                prop_decrease=0.6,
                n_fft=240,
                win_length=480,
                hop_length=240,
                stationary=True
            )

            reduced_int16 = np.clip(reduced_float * 32768, -32768, 32767).astype(np.int16)
            processed_frame = reduced_int16.tobytes()

            return processed_frame

        except Exception as e:
            print(f"Noise reduction 에러 무시: {e}")
            return frame
        
# === STT 노드 ===
class STTNode(Node):
    def __init__(self):
        super().__init__('stt_node')
        self.publisher_ = self.create_publisher(String, '/stt_text', 10)
        self.talkbutton_sub = self.create_subscription(Bool,'/talkbutton_pressed',self.talk_button_callback,10)
        self.heartbeat_pub = self.create_publisher(Heartbeat, '/heartbeat/stt_node', 10)
        self.token = None
        self._sess = Session()
        self.get_logger().info("세션 생성 완료")
        self.is_processing = False  # 중복 실행 방지용
        self.talkbutton_pressed = False

        self.heartbeat = 0 

        # 모니터 루프 스레드 시작
        self.monitor_thread = threading.Thread(target=self.monitor_loop)
        self.monitor_thread.daemon = True
        self.monitor_thread.start()
        self.get_logger().info("STT 모니터 루프 시작, STT NODE START")
        self.create_timer(1.0, self.pub_heartbeat)  # 1초마다 heartbeat 퍼블리시
    
    def talk_button_callback(self, msg):
        self.talkbutton_pressed = msg.data

        # if self.talkbutton_pressed:
        #     self.get_logger().info("Talk button ON")
        # else: 
        #     self.get_logger().info("Talk button OFF")
    
    def monitor_loop(self):
        while rclpy.ok():
            
            self.heartbeat = 1 

            #self.get_logger().info("STT 모니터 루프 실행 중")
            if self.talkbutton_pressed and not self.is_processing:
                self.get_logger().info("🎤 STT 실행 시작")
                self.is_processing = True
                
                config = pb.DecoderConfig(
                    sample_rate=SAMPLE_RATE,
                    encoding=ENCODING,
                    use_itn=True,
                    use_disfluency_filter=False,
                    use_profanity_filter=False,
                    keywords=boosted_keywords
                )
                try:
                    with grpc.secure_channel(GRPC_SERVER_URL, grpc.ssl_channel_credentials()) as channel:
                        stub = pb_grpc.OnlineDecoderStub(channel)
                        cred = grpc.access_token_call_credentials(self.get_token())

                        final_result = ""

                        def req_iterator():
                            yield pb.DecoderRequest(streaming_config=config)
                            with VADMicStreamer(self) as mic:
                                for chunk in mic.read_stream():
                                    if not self.talkbutton_pressed:
                                        yield pb.DecoderRequest(audio_content=chunk)
                                        self.get_logger().info("🛑 버튼 떨어짐: 마지막 프레임 전송")
                                        break
                                    yield pb.DecoderRequest(audio_content=chunk)

                        for resp in stub.Decode(req_iterator(), credentials=cred):
                            
                            for result in resp.results:
                                if not result.alternatives:
                                    self.get_logger().info("결과 없음")
                                    continue
                                text = result.alternatives[0].text.strip()
                                
                                if text and result.is_final:
                                    final_result = text

                        if final_result:
                            self.get_logger().info(f"✅ 최종 인식: {final_result}")
                            msg = String()
                            msg.data = final_result
                            self.publisher_.publish(msg)
                            self.is_processing = False
                            self.get_logger().info("🔁 STT 실행 종료 및 초기화 완료")

                except Exception as e:
                    self.get_logger().error(f"STT 오류: {e}")
                    self.heartbeat = 0 
                finally:
                    self.is_processing = False
                    self.get_logger().info(" STT 실행 종료 및 초기화 완료")
    
    def pub_heartbeat(self):
        msg = Heartbeat()
        now = self.get_clock().now().to_msg()

        msg.timestamp = now 
        msg.code = self.heartbeat
       
        self.heartbeat_pub.publish(msg)

    def get_token(self):
        if self.token is None or self.token["expire_at"] < time.time():
            resp = self._sess.post(
                API_BASE + "/v1/authenticate",
                data={"client_id": CLIENT_ID, "client_secret": CLIENT_SECRET}
            )
            resp.raise_for_status()
            self.token = resp.json()
        return self.token["access_token"]

# === main ===
def main(args=None):
    rclpy.init(args=args)
    node = STTNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
