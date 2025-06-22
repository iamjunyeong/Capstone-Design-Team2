#include "dwb_plugins/critics/high_speed_critic.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace dwb_plugins {

void HighSpeedCritic::onInit()
{
  auto n = node_.lock();
  n->declare_parameter(name_ + ".scale", 500.0);
  n->declare_parameter(name_ + ".v_max", 1.0);
  n->get_parameter(name_ + ".scale", scale_);
  n->get_parameter(name_ + ".v_max", v_max_);
}

double HighSpeedCritic::scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj)
{
  const double vx = std::abs(traj.velocity.x);
  return (v_max_ - vx) * scale_;      // |vx|가 클수록 penalty ↓
}

}  // namespace dwb_plugins

PLUGINLIB_EXPORT_CLASS(dwb_plugins::HighSpeedCritic,
                       dwb_core::TrajectoryCritic)