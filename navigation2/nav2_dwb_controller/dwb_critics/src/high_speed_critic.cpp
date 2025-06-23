#include "dwb_critics/high_speed_critic.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <cmath>

namespace dwb_critics
{

HighSpeedCritic::HighSpeedCritic()
: scale_(0.01), v_max_(0.8) {}

void HighSpeedCritic::onInit()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("HighSpeedCritic::onInit(): node expired");
  }

  using nav2_util::declare_parameter_if_not_declared;
  const std::string pfx = "FollowPath." + name_ + ".";

  declare_parameter_if_not_declared(node, pfx + "scale",
                                    rclcpp::ParameterValue(scale_));
  declare_parameter_if_not_declared(node, pfx + "v_max",
                                    rclcpp::ParameterValue(v_max_));

  node->get_parameter(pfx + "scale", scale_);
  node->get_parameter(pfx + "v_max", v_max_);

  RCLCPP_INFO(node->get_logger(),
              "%s init ‣ v_max:%.2f scale:%.2f",
              name_.c_str(), v_max_, scale_);
}

double HighSpeedCritic::scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj)
{
  const double v = std::abs(traj.velocity.x);
  if (v <= v_max_) {return 0.0;}
  return (v - v_max_) * scale_;     // 초과분에 비례한 페널티
}

PLUGINLIB_EXPORT_CLASS(dwb_critics::HighSpeedCritic,
                       dwb_core::TrajectoryCritic)

}  // namespace dwb_critics
