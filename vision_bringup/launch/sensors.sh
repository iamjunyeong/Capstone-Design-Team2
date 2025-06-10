#!/bin/bash
set -e  # 오류 발생 시 즉시 종료

# LiDAR 노드 실행 (백그라운드로 실행)
echo "[INFO] Starting SLLIDAR node..."
ros2 launch sllidar_ros2 view_sllidar_a3_launch.py &
LIDAR_PID=$!

# Realsense 카메라 실행 (백그라운드로 실행)
echo "[INFO] Starting Realsense camera node..."
ros2 run realsense2_camera realsense2_camera_node --ros-args \
  -p enable_color:=true \
  -p enable_depth:=true \
  -p enable_sync:=true \
  -p align_depth.enable:=true \
  -p enable_infra1:=false \
  -p enable_infra2:=false &
REALSENSE_PID=$!

# URDF xacro 경로 계산
xacro_file=$(ros2 pkg prefix realsense2_description)/share/realsense2_description/urdf/test_d455_camera.urdf.xacro

# robot_state_publisher 실행
echo "[INFO] Starting robot_state_publisher..."
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args -p robot_description:="$(xacro ${xacro_file} use_nominal_extrinsics:=true add_plug:=true)" &
STATE_PUB_PID=$!

# 모든 PID 출력
echo "[INFO] Launched nodes:"
echo "  SLLIDAR PID:              $LIDAR_PID"
echo "  Realsense Camera PID:     $REALSENSE_PID"
echo "  Robot State Publisher PID: $STATE_PUB_PID"

# Ctrl+C 누를 때 백그라운드 노드 모두 종료
trap "echo '[INFO] Shutting down...'; kill $LIDAR_PID $REALSENSE_PID $STATE_PUB_PID" SIGINT

# foreground 대기
wait

