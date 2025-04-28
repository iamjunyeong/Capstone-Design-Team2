import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import numpy as np
import noisereduce as nr

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
MAX_SILENCE_DURATION = 5

boosted_keywords = [
    "공학관:5.0", "신공학관:5.0", "새천년관:5.0", "학생회관:5.0", "법학관:5.0", "수의대:5.0",
    "가줘:5.0", "가자:5.0", "가고:5.0", "싶어:5.0", "데려다줘:5.0", "이동:5.0", "목적지:5.0",
    "몇:3.0","분:2.0", "얼마나:5.0", "도착:5.0", "시간:5.0", "얼마:5.0", "남았어:5.0", "걸려:5.0",
    "어디야:5.0", "어디:5.0","현재:5.0", "위치:5.0", "지나:5.0"
]

# === 마이크 VAD 스트리머 ===
class VADMicStreamer:
    def __init__(self):
        self.q = queue.Queue()
        self.vad = webrtcvad.Vad(2)
        self.max_silence_frames = int(MAX_SILENCE_DURATION * 1000 / FRAME_DURATION_MS)
        
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
            print("⚠️", status)
        self.q.put(bytes(indata))

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stream.stop()
        self.stream.close()

    def read_stream(self, stop_condition_fn):
        silence = 0
        while True:
            if stop_condition_fn():  # 외부 종료 조건 검사
                print("버튼 off")
                break
            frame = self.q.get()
            if len(frame) < 2:
                continue
            
            frame = self.preprocess_audio(frame) # (지욱) 전처리 추가

            is_speech = self.vad.is_speech(frame, SAMPLE_RATE)
            yield frame
            if is_speech:
                silence = 0
            else:
                silence += 1
                if silence > self.max_silence_frames:
                    break
    
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
                use_tensorflow=True
            )

            # [핵심] 항상 int16로 변환해서 다시 bytes로
            reduced_noise = reduced_noise.astype(np.int16)
            processed_frame = reduced_noise.tobytes()

            return processed_frame

        except Exception as e:
            print(f"⚠️ Noise reduction 에러 무시: {e}")
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
    
    def talk_button_callback(self, msg):
        self.talkbutton_pressed = msg.data
        self.get_logger().info(f"Talk button 상태: {self.talkbutton_pressed}")
        if msg.data and not self.is_processing:
            self.get_logger().info("버튼 눌림 감지됨. STT 실행 시작.")
            self.is_processing = True
            try:
                self.run_stt()
            finally:
                self.is_processing = False
        elif not msg.data:
            self.get_logger().info("버튼 떨어짐, STT정지요청")

    def get_token(self):
        if self.token is None or self.token["expire_at"] < time.time():
            resp = self._sess.post(
                API_BASE + "/v1/authenticate",
                data={"client_id": CLIENT_ID, "client_secret": CLIENT_SECRET}
            )
            resp.raise_for_status()
            self.token = resp.json()
        return self.token["access_token"]

    def run_stt(self):
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

            def req_iterator():
                yield pb.DecoderRequest(streaming_config=config)
                with VADMicStreamer() as mic:
                    for chunk in mic.read_stream(lambda: not self.talkbutton_pressed):  # 종료 조건
                        yield pb.DecoderRequest(audio_content=chunk)

            self.get_logger().info("음성 인식 대기 중...")
            for resp in stub.Decode(req_iterator(), credentials=cred):
                for result in resp.results:
                    text = result.alternatives[0].text
                    if not text.strip():
                        continue
                    if result.is_final:
                        self.get_logger().info(f"최종 인식: {text}")
                        msg = String()
                        msg.data = text
                        self.publisher_.publish(msg)

# === main ===
def main(args=None):
    rclpy.init(args=args)
    node = STTNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
