from launch import LaunchDescription
from launch.actions import OpaqueFunction
import subprocess
import time

# 사용할 mountpoint 목록
MOUNTPOINT_LIST = [
    "SUWN-RTCM31",
    "YONS-RTCM31",
    "DAEJ-RTCM31"
]

MAX_RETRIES_PER_MOUNT = 3

def try_launch_rtk(context, *args, **kwargs):
    for mountpoint in MOUNTPOINT_LIST:
        for attempt in range(MAX_RETRIES_PER_MOUNT):
            print(f"[RTK] Attempt {attempt+1}/{MAX_RETRIES_PER_MOUNT} with mountpoint: {mountpoint}")
            try:
                proc = subprocess.Popen([
                    'ros2', 'launch', 'sensor_bringup', 'rtk_gps.launch.py',
                    f'mountpoint:={mountpoint}'
                ], stdout=subprocess.PIPE, stderr=subprocess.PIPE)

                time.sleep(6)  # RTK 노드 안정화 시간

                if proc.poll() is None:
                    print(f"[RTK] Launched successfully with mountpoint: {mountpoint}")
                    return []  # 런치 성공 시 종료

                # 실패 로그 출력
                out, err = proc.communicate(timeout=3)
                print(f"[RTK] Failed: {err.decode()}")

            except Exception as e:
                print(f"[RTK] Exception: {e}")
            finally:
                if proc and proc.poll() is None:
                    proc.terminate()
                    time.sleep(1)

        print(f"[RTK] All retries failed for mountpoint: {mountpoint}, trying next...")
    
    print("[RTK] All mountpoints failed. Could not establish RTK connection.")
    return []

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=try_launch_rtk)
    ])
