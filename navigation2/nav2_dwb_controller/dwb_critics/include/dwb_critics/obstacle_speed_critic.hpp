#ifndef DWB_CRITICS__OBSTACLE_SPEED_CRITIC_HPP_
#define DWB_CRITICS__OBSTACLE_SPEED_CRITIC_HPP_

#include "dwb_core/trajectory_critic.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav_2d_msgs/msg/path2_d.hpp"
#include "nav_2d_msgs/msg/twist2_d.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"

namespace dwb_critics {

class ObstacleSpeedCritic : public dwb_core::TrajectoryCritic {
public:
  ObstacleSpeedCritic();
  virtual ~ObstacleSpeedCritic();

  void onInit() override;
  double scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj) override;
  void reset() override;
  bool prepare(
    const geometry_msgs::msg::Pose2D & pose,
    const nav_2d_msgs::msg::Twist2D & vel,
    const geometry_msgs::msg::Pose2D & goal,
    const nav_2d_msgs::msg::Path2D & global_plan) override;

private:
  double full_speed_distance_;
  double stop_distance_;
  double scale_;
};

} // namespace dwb_critics

#endif // DWB_CRITICS__OBSTACLE_SPEED_CRITIC_HPP_
