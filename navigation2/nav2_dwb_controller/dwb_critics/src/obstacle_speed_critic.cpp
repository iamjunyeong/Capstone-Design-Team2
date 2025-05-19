#include "dwb_critics/obstacle_speed_critic.hpp"
#include "dwb_msgs/msg/trajectory2_d.hpp"  // ✅ 필수 헤더 추가
#include "pluginlib/class_list_macros.hpp"
#include <string>
#include "rclcpp/exceptions.hpp"

namespace dwb_critics {

ObstacleSpeedCritic::ObstacleSpeedCritic()
  : full_speed_distance_(5.0), stop_distance_(2.5), scale_(1000.0) {}

ObstacleSpeedCritic::~ObstacleSpeedCritic() {}

void ObstacleSpeedCritic::onInit() {
  auto node = node_.lock();
  if (!node) throw std::runtime_error("Failed to lock node");

  node->declare_parameter("FollowPath.ObstacleSpeedCritic.obstacle_state", 0);
  node->declare_parameter("FollowPath.ObstacleSpeedCritic.obstacle_distance", 100.0);

  node->declare_parameter(name_ + ".full_speed_distance", 5.0);
  node->declare_parameter(name_ + ".stop_distance", 2.5);
  node->declare_parameter(name_ + ".scale", 1000.0);

  node->get_parameter(name_ + ".full_speed_distance", full_speed_distance_);
  node->get_parameter(name_ + ".stop_distance", stop_distance_);
  node->get_parameter(name_ + ".scale", scale_);

  RCLCPP_INFO(node->get_logger(), "%s initialized (full: %.2f, stop: %.2f, scale: %.2f)",
             name_.c_str(), full_speed_distance_, stop_distance_, scale_);
}

double ObstacleSpeedCritic::scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj) {
  int8_t obstacle_state = 0;
  float obstacle_distance = 100.0;
  auto node = node_.lock();
  if (!node) return 0.0;

  node->get_parameter("FollowPath.ObstacleSpeedCritic.obstacle_state", obstacle_state);
  node->get_parameter("FollowPath.ObstacleSpeedCritic.obstacle_distance", obstacle_distance);

  double penalty = 0.0;

  // 1. 동적/혼합 장애물 모드인 경우에만 제어
  if (obstacle_state == 2 || obstacle_state == 3) {
    if (obstacle_distance <= stop_distance_) {
      // ✅ 선속도 + 각속도 완전 정지
      penalty += scale_ * 1e6; // linear.x=0
      penalty += std::abs(traj.velocity.theta) * scale_ * 1000.0; // angular.z=0
    } else if (obstacle_distance < full_speed_distance_) {
      // ✅ 감속 구간: 선속도 + 각속도 점진적 제어
      double ratio = (obstacle_distance - stop_distance_) / (full_speed_distance_ - stop_distance_);
      ratio = std::max(0.0, std::min(1.0, ratio)); // 0~1 범위 제한
      
      // 선속도 패널티
      penalty += scale_ * (1.0 - ratio) * 1000.0;
      
      // 각속도 패널티 (거리 비례)
      penalty += std::abs(traj.velocity.theta) * scale_ * 500.0 * (1.0 - ratio);
    }
  }

  return penalty;
}

void ObstacleSpeedCritic::reset() {}
bool ObstacleSpeedCritic::prepare(const geometry_msgs::msg::Pose2D&, 
                                const nav_2d_msgs::msg::Twist2D&,
                                const geometry_msgs::msg::Pose2D&, 
                                const nav_2d_msgs::msg::Path2D&) { 
  return true; 
}

} // namespace dwb_critics

PLUGINLIB_EXPORT_CLASS(dwb_critics::ObstacleSpeedCritic, dwb_core::TrajectoryCritic)
