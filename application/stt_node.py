import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
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
MAX_SILENCE_DURATION = 1

boosted_keywords = [
    "공학관:5.0","공대:5.0","신공:5.0", "신공학관:5.0", "새천년관:5.0", "학생회관:5.0", "법학관:5.0","정문:5.0", "후문:5.0", "문과대:5.0","인문학관:5.0", "경영대:5.0", "경영관:5.0",
    "가줘:5.0", "가자:5.0", "가고:2.0", "싶어:5.0", "데려다줘:5.0", "이동:5.0", "목적지:5.0",
    "몇:3.0","분:2.0", "얼마나:5.0", "도착:5.0", "시간:5.0", "얼마:5.0", "남았어:5.0", "걸려:5.0",
    "어디야:5.0", "어디:5.0","현재:5.0", "위치:5.0", "지나:5.0"
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
            else:
                try:
                    frame = self.q.get(timeout=0.1) 
                except queue.Empty:
                    continue

                if len(frame) < 2:
                    continue
                
                frame = self.preprocess_audio(frame) # (지욱) 전처리 추가

                is_speech = self.vad.is_speech(frame, SAMPLE_RATE)
                yield frame
                
                if is_speech:
                    silence = 0
                else:
                    silence += 1
                    if silence >= self.max_silence_frames:
                        print("무음 감지, 스트리밍 종료")
                        return
    
    def preprocess_audio(self, frame): # 전처리 함수
        try:
            if len(frame) < 1024:  # 짧은 건 패스
                return frame

            # bytes -> numpy array
            audio_np = np.frombuffer(frame, dtype=np.int16)

            if len(audio_np) < 512:
                return frame

            # 잡음 제거
            reduced_noise = nr.reduce_noise(
                y=audio_np,
                sr=SAMPLE_RATE,
                prop_decrease=0.5,
                n_fft=512,
                win_length=512,
                hop_length=256,
                n_std_thresh=1.5,
                stationary=True,
                use_tensorflow=True #기똥차네
            )

            # [핵심] 항상 int16로 변환해서 다시 bytes로
            reduced_noise = reduced_noise.astype(np.int16)
            processed_frame = reduced_noise.tobytes()

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
        self.get_logger().info('STT Node has started.')
        self.token = None
        self._sess = Session()
        self.is_processing = False  # 중복 실행 방지용
        self.talkbutton_pressed = False

        self.token = None
        self._sess = Session()

        # 모니터 루프 스레드 시작
        self.monitor_thread = threading.Thread(target=self.monitor_loop)
        self.get_logger().info("STT 모니터 루프 시작")
        self.monitor_thread.daemon = True
        self.monitor_thread.start()

    def talk_button_callback(self, msg):
        self.talkbutton_pressed = msg.data
        if self.talkbutton_pressed:
            self.get_logger().info("Talk button ON")
        else: 
            self.get_logger().info("Talk button OFF")
    
    def monitor_loop(self):
        while rclpy.ok():
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
                                    continue
                                text = result.alternatives[0].text.strip()
                                if text and result.is_final:
                                    final_result = text

                        if final_result:
                            self.get_logger().info(f"✅ 최종 인식: {final_result}")
                            msg = String()
                            msg.data = final_result
                            self.publisher_.publish(msg)

                except Exception as e:
                    self.get_logger().error(f"STT 오류: {e}")

                self.is_processing = False
                self.get_logger().info("🕓 STT 처리 완료")
            time.sleep(0.1)

    def get_token(self):
        if self.token is None or self.token["expire_at"] < time.time():
            resp = self._sess.post(
                API_BASE + "/v1/authenticate",
                data={"client_id": CLIENT_ID, "client_secret": CLIENT_SECRET}
            )
            resp.raise_for_status()
            self.token = resp.json()
        return self.token["access_token"]
'''
    def run_stt(self):
        self.get_logger().info("Run STT")
        config = pb.DecoderConfig(
            sample_rate=SAMPLE_RATE,
            encoding=ENCODING,
            use_itn=True,
            use_disfluency_filter=False,
            use_profanity_filter=False,
            keywords=boosted_keywords
        )

        with grpc.secure_channel(GRPC_SERVER_URL, grpc.ssl_channel_credentials()) as channel:
            stub = pb_grpc.OnlineDecoderStub(channel)
            cred = grpc.access_token_call_credentials(self.get_token())

            final_result = ""  # 마지막 결과 저장용

            def req_iterator():
                yield pb.DecoderRequest(streaming_config=config)
                self.get_logger().info("스트리밍 설정 전송")

                with VADMicStreamer(self) as mic:
                    try:
                        for chunk in mic.read_stream():
                            if not self.talkbutton_pressed:
                                yield pb.DecoderRequest(audio_content=chunk)
                                self.get_logger().info("마지막 프레임 전송 후 종료")
                                break
                            else:
                                yield pb.DecoderRequest(audio_content=chunk)
                    except Exception as e:
                        self.get_logger().error(f"요청 제너레이터 오류: {e}")
                    finally:
                        self.get_logger().info("스트리밍 종료")

            # 음성 인식 응답 수신
            for resp in stub.Decode(req_iterator(), credentials=cred):
                for result in resp.results:
                    if not result.alternatives:
                        continue
                    text = result.alternatives[0].text
                    if not text.strip():
                        continue
                    if result.is_final:
                        final_result = text  # 🔑 마지막 결과 저장

            # 요청 루프가 끝났을 때 마지막 결과가 있다면 publish
            if final_result:
                self.get_logger().info(f"STT 종료, 최종 인식: {final_result}")
                msg = String()
                msg.data = final_result
                self.publisher_.publish(msg)
                self.is_processing = False
        self.is_processing = False
'''
# === main ===
def main(args=None):
    rclpy.init(args=args)
    node = STTNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
