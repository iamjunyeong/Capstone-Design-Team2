#!/bin/bash
set -e  # 오류 발생 시 즉시 종료

##########################
# 1. SLLIDAR A3
##########################
# echo "[INFO] Starting SLLIDAR node..."
# ros2 launch sllidar_ros2 view_sllidar_a3_launch.py &
# LIDAR_PID=$!

##########################
# 2. Intel RealSense D455
##########################
echo "[INFO] Starting RealSense camera node..."
ros2 run realsense2_camera realsense2_camera_node --ros-args \
  -p enable_color:=true \
  -p enable_depth:=true \
  -p enable_sync:=true \
  -p align_depth.enable:=true \
  -p enable_infra1:=false \
  -p enable_infra2:=false &
REALSENSE_PID=$!

##########################
# 3. USB cameras (two namespaces)
##########################
echo "[INFO] Starting USB camera nodes..."
ros2 run usb_cam usb_cam_node_exe --ros-args \
  --remap __ns:=/usb_cam_0 \
  --params-file /home/ubuntu/capstone_ws/src/Capstone-Design-Team2/usb_cam/config/params_1.yaml &
USB_CAM0_PID=$!

ros2 run usb_cam usb_cam_node_exe --ros-args \
  --remap __ns:=/usb_cam_1 \
  --params-file /home/ubuntu/capstone_ws/src/Capstone-Design-Team2/usb_cam/config/params_2.yaml &
USB_CAM1_PID=$!

##########################
# 4. robot_state_publisher
##########################
xacro_file=$(ros2 pkg prefix realsense2_description)/share/realsense2_description/urdf/test_d455_camera.urdf.xacro
echo "[INFO] Starting robot_state_publisher..."
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args -p robot_description:="$(xacro ${xacro_file} use_nominal_extrinsics:=true add_plug:=true)" &
STATE_PUB_PID=$!

##########################
# Summary
##########################
echo "[INFO] Launched nodes:"
printf "  SLLIDAR PID:               %s\n" "$LIDAR_PID"
printf "  RealSense Camera PID:      %s\n" "$REALSENSE_PID"
printf "  USB Cam 0 PID:             %s\n" "$USB_CAM0_PID"
printf "  USB Cam 1 PID:             %s\n" "$USB_CAM1_PID"
printf "  Robot State Publisher PID: %s\n" "$STATE_PUB_PID"

##########################
# Clean shutdown handler
##########################
trap 'echo "[INFO] Shutting down..."; \
      kill $LIDAR_PID $REALSENSE_PID $USB_CAM0_PID $USB_CAM1_PID $STATE_PUB_PID' SIGINT

##########################
# Wait (keeps script alive)
##########################
wait
