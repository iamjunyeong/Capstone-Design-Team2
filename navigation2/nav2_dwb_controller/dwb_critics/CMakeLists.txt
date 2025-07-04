cmake_minimum_required(VERSION 3.5)
project(dwb_critics)

################################################################################
# ▸ 공통 설정 -------------------------------------------------------------------
################################################################################
# Nav2 패키지 매크로: C++17 플래그·warnings·export 설정 자동 적용
find_package(nav2_common REQUIRED)
nav2_package()                         # ← 이미 nav2_util 경로ㆍ정책 포함

################################################################################
# ▸ 의존성 ----------------------------------------------------------------------
################################################################################
find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(costmap_queue REQUIRED)
find_package(dwb_core REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_2d_msgs REQUIRED)
find_package(nav_2d_utils REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_util REQUIRED)       # nav2_util::nav2_util 사용

# 한 번에 관리하기 위한 리스트
set(dependencies
  angles
  nav2_costmap_2d
  costmap_queue
  dwb_core
  geometry_msgs
  nav_2d_msgs
  nav_2d_utils
  pluginlib
  rclcpp
  sensor_msgs
  nav2_util
)

################################################################################
# ▸ 빌드 대상 -------------------------------------------------------------------
################################################################################
include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/alignment_util.cpp
  src/map_grid.cpp
  src/goal_dist.cpp
  src/path_dist.cpp
  src/goal_align.cpp
  src/path_align.cpp
  src/base_obstacle.cpp
  src/obstacle_footprint.cpp
  src/oscillation.cpp
  src/prefer_forward.cpp
  src/rotate_to_goal.cpp
  src/twirling.cpp
  # 커스텀 크리틱 두 개 ↓
  src/obstacle_speed_critic.cpp
  src/high_speed_critic.cpp
)

ament_target_dependencies(${PROJECT_NAME} ${dependencies})

################################################################################
# ▸ 설치 ------------------------------------------------------------------------
################################################################################
install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

install(DIRECTORY include/
        DESTINATION include/)

# pluginlib XML (커스텀 플러그인 선언 포함)
install(FILES default_critics.xml
        DESTINATION share/${PROJECT_NAME})

################################################################################
# ▸ 테스트(선택) ----------------------------------------------------------------
################################################################################
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)      # 예: BSD 라이선스 문구 스킵
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)
endif()

################################################################################
# ▸ export ---------------------------------------------------------------------
################################################################################
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

pluginlib_export_plugin_description_file(dwb_core default_critics.xml)

ament_package()
