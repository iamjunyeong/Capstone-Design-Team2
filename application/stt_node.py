import rclpy
from rclpy.node import Node
from std_msgs.msg import String

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
MAX_SILENCE_DURATION = 0.6

boosted_keywords = [
    "공학관", "신공학관", "새천년관", "학생회관", "법학관",
    "가줘", "가자", "가고 싶어", "데려다줘", "이동", "목적지",
    "몇 분", "얼마나", "도착 시간", "시간", "얼마 걸려", "남았어",
    "어디야", "지금 어디", "현재 위치", "어디를 지나", "위치"
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

    def read_stream(self):
        silence = 0
        while True:
            frame = self.q.get()
            if len(frame) < 2:
                continue
            is_speech = self.vad.is_speech(frame, SAMPLE_RATE)
            yield frame
            if is_speech:
                silence = 0
            else:
                silence += 1
                if silence > self.max_silence_frames:
                    break

# === STT 노드 ===
class STTNode(Node):
    def __init__(self):
        super().__init__('stt_node')
        self.publisher_ = self.create_publisher(String, '/stt_text', 10)
        self.get_logger().info('🎤 STT Node has started.')
        self.token = None
        self._sess = Session()
        self.run_stt()

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
                    for chunk in mic.read_stream():
                        yield pb.DecoderRequest(audio_content=chunk)

            self.get_logger().info("🗣️ 음성 인식 대기 중...")
            for resp in stub.Decode(req_iterator(), credentials=cred):
                for result in resp.results:
                    text = result.alternatives[0].text
                    if not text.strip():
                        continue
                    if result.is_final:
                        self.get_logger().info(f"✅ 최종 인식: {text}")
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

