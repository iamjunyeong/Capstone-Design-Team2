#pragma once
#include "dwb_core/trajectory_critic.hpp"

namespace dwb_critics
{

class HighSpeedCritic : public dwb_core::TrajectoryCritic
{
public:
  HighSpeedCritic();
  ~HighSpeedCritic() override = default;

  void onInit() override;
  bool prepare(const geometry_msgs::msg::Pose2D &,
               const nav_2d_msgs::msg::Twist2D &,
               const geometry_msgs::msg::Pose2D &,
               const nav_2d_msgs::msg::Path2D &) override {return true;}
  double scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj) override;
  void reset() override {}

private:
  double scale_;
  double v_max_;
};

}  // namespace dwb_critics
