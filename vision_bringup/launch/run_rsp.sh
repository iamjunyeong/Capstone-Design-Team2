#!/bin/bash

# D455 Xacro 경로 정의
xacro_file=$(ros2 pkg prefix realsense2_description)/share/realsense2_description/urdf/test_d455_camera.urdf.xacro

# robot_state_publisher 실행
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args \
  -p robot_description:="$(xacro ${xacro_file} use_nominal_extrinsics:=true add_plug:=true)"


