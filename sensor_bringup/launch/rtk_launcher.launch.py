from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess
from launch.substitutions import LaunchConfiguration
import subprocess
import time

# 사용할 mountpoint 목록
MOUNTPOINT_LIST = [
    "SONP-RTCM31",
    "SUWN-RTCM31",
    "YONS-RTCM31",
    "DAEJ-RTCM31"
]

MAX_RETRIES_PER_MOUNT = 3

# 이 변수에 선택된 mountpoint를 저장해 다음 ExecuteProcess에서 사용
_selected_mountpoint = None

def try_launch_rtk(context, *args, **kwargs):
    global _selected_mountpoint

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
                    proc.terminate()  # 이제 respawn 방식으로 다시 실행하므로 종료
                    _selected_mountpoint = mountpoint
                    return []
                
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
    _selected_mountpoint = None
    return []

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=try_launch_rtk),

        # RTK 노드 감시 및 자동 재시작 실행
        # OpaqueFunction 이후에 mountpoint가 설정되어 있어야 동작
        ExecuteProcess(
            cmd=[
                'ros2', 'launch', 'sensor_bringup', 'rtk_gps.launch.py',
                f'mountpoint:={_selected_mountpoint or MOUNTPOINT_LIST[0]}'
            ],
            name='rtk_persistent_launcher',
            shell=False,
            output='screen',
            respawn=True,
            respawn_delay=3.0
        )
    ])
