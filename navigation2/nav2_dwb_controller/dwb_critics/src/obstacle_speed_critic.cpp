#include "dwb_critics/obstacle_speed_critic.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/exceptions.hpp"

namespace dwb_critics {

ObstacleSpeedCritic::ObstacleSpeedCritic()
  : full_speed_distance_(5.0), stop_distance_(2.5), scale_(1000.0),
    yaw_scale_(1000.0 /* 기본값: 선속도와 동일 비중 */) {}

ObstacleSpeedCritic::~ObstacleSpeedCritic() {}

// ------------------------------------------------------------
// onInit : declare & get parameters (★ yaw_scale 추가)
// ------------------------------------------------------------
void ObstacleSpeedCritic::onInit() {
  auto node = node_.lock();
  if (!node) throw std::runtime_error("Failed to lock node");

  // perception 노드가 주입하는 전역 파라미터 (state / distance)
  node->declare_parameter("FollowPath.ObstacleSpeedCritic.obstacle_state", 0);
  node->declare_parameter("FollowPath.ObstacleSpeedCritic.obstacle_distance", 100.0);

  // critic 전용 파라미터들
  node->declare_parameter(name_ + ".full_speed_distance", full_speed_distance_);
  node->declare_parameter(name_ + ".stop_distance", stop_distance_);
  node->declare_parameter(name_ + ".scale", scale_);
  node->declare_parameter(name_ + ".yaw_scale", yaw_scale_);          // ★NEW

  node->get_parameter(name_ + ".full_speed_distance", full_speed_distance_);
  node->get_parameter(name_ + ".stop_distance", stop_distance_);
  node->get_parameter(name_ + ".scale", scale_);
  node->get_parameter(name_ + ".yaw_scale", yaw_scale_);              // ★NEW

  RCLCPP_INFO(node->get_logger(),
              "%s initialized (full: %.2f, stop: %.2f, scale: %.1f, yaw_scale: %.1f)",
              name_.c_str(), full_speed_distance_, stop_distance_, scale_, yaw_scale_);
}

// ------------------------------------------------------------
// scoreTrajectory : main penalty logic (★ 선속도·각속도 비례 벌점)
// ------------------------------------------------------------
double ObstacleSpeedCritic::scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj) {
  auto node = node_.lock();
  if (!node) return 0.0;

  // ① perception 노드가 갱신한 전역 파라미터 읽기
  int8_t obstacle_state = 0;
  double obstacle_distance = 100.0;
  node->get_parameter("FollowPath.ObstacleSpeedCritic.obstacle_state", obstacle_state);
  node->get_parameter("FollowPath.ObstacleSpeedCritic.obstacle_distance", obstacle_distance);

  double penalty = 0.0;

  // ② 동적(2)·혼합(3) 상태에서만 활성화
  if (obstacle_state == 2 || obstacle_state == 3) {
    // (A) 완전 정지 구간 ---------------------------------------
    if (obstacle_distance <= stop_distance_) {
      penalty += scale_ * 1e6; // 선속도 : 사실상 금지
      penalty += std::abs(traj.velocity.theta) * yaw_scale_ * 1e6; // 각속도도 금지
      return penalty;
    }

    // (B) 감속 구간 -------------------------------------------
    if (obstacle_distance < full_speed_distance_) {
      double ratio = (obstacle_distance - stop_distance_) /
                     (full_speed_distance_ - stop_distance_); // 0 (가까움) ~ 1 (멀리감)
      ratio = std::clamp(ratio, 0.0, 1.0);

      // ★ 선속도 비례 벌점 : 빠를수록 불리
      penalty += std::abs(traj.velocity.x) * scale_ * (1.0 - ratio);

      // ★ 각속도 비례 벌점 : yaw_scale_으로 독립 조정
      penalty += std::abs(traj.velocity.theta) * yaw_scale_ * (1.0 - ratio);
    }
  }
  return penalty;   // (C) free‑speed zone → 0
}

// ------------------------------------------------------------
// prepare / reset : 기존 구현 그대로
// ------------------------------------------------------------
bool ObstacleSpeedCritic::prepare(const geometry_msgs::msg::Pose2D&,
                                  const nav_2d_msgs::msg::Twist2D&,
                                  const geometry_msgs::msg::Pose2D&,
                                  const nav_2d_msgs::msg::Path2D&) {
  return true;
}

void ObstacleSpeedCritic::reset() {}

}  // namespace dwb_critics

// pluginlib 등록 매크로 : 변경 없음
PLUGINLIB_EXPORT_CLASS(dwb_critics::ObstacleSpeedCritic, dwb_core::TrajectoryCritic)
