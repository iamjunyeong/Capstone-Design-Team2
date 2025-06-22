#ifndef DWB_CRITICS_OBSTACLE_SPEED_CRITIC_HPP_
#define DWB_CRITICS_OBSTACLE_SPEED_CRITIC_HPP_

#include <string>
#include "dwb_core/trajectory_critic.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav_2d_msgs/msg/path2_d.hpp"
#include "nav_2d_msgs/msg/twist2_d.hpp"
#include "dwb_msgs/msg/trajectory2_d.hpp"

namespace dwb_critics {

class ObstacleSpeedCritic : public dwb_core::TrajectoryCritic
{
public:
  ObstacleSpeedCritic();
  ~ObstacleSpeedCritic() override;

  // TrajectoryCritic interface ----------------------------------
  void onInit() override;
  double scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj) override;
  bool prepare(const geometry_msgs::msg::Pose2D & pose,
               const nav_2d_msgs::msg::Twist2D & velocity,
               const geometry_msgs::msg::Pose2D & goal,
               const nav_2d_msgs::msg::Path2D & global_path) override;
  void reset() override;

private:
  // ---------- critic parameters ----------
  double full_speed_distance_;   // ≥ : no speed limit
  double stop_distance_;         // ≤ : full stop only
  double scale_;                 // linear‑speed penalty weight
  double yaw_scale_;             // ★NEW 각속도 penalty weight
};

}  // namespace dwb_critics

#endif  // DWB_CRITICS_OBSTACLE_SPEED_CRITIC_HPP_