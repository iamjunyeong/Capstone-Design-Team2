#pragma once
#include "dwb_core/trajectory_critic.hpp"
namespace dwb_critics {

class HighSpeedCritic : public dwb_core::TrajectoryCritic
{
public:
  void onInit() override;
  double scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj) override;
  bool prepare(const geometry_msgs::msg::Pose2D&, const nav_2d_msgs::msg::Twist2D&,
               const geometry_msgs::msg::Pose2D&, const nav_2d_msgs::msg::Path2D&) override
  { return true; }
  void reset() override {}

private:
  double scale_;
  double v_max_;   // 로봇의 max_vel_x 와 동일하게
};

}  // namespace dwb_critics