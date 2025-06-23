#pragma once
#include "dwb_core/trajectory_critic.hpp"

namespace dwb_critics
{

class ObstacleSpeedCritic : public dwb_core::TrajectoryCritic
{
public:
  ObstacleSpeedCritic();
  ~ObstacleSpeedCritic() override;

  // TrajectoryCritic API
  void onInit() override;
  bool prepare(const geometry_msgs::msg::Pose2D &,
               const nav_2d_msgs::msg::Twist2D &,
               const geometry_msgs::msg::Pose2D &,
               const nav_2d_msgs::msg::Path2D &) override {return true;}
  double scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj) override;
  void reset() override {}

private:
  double full_speed_distance_;
  double stop_distance_;
  double scale_;
  double yaw_scale_;
};

}  // namespace dwb_critics
