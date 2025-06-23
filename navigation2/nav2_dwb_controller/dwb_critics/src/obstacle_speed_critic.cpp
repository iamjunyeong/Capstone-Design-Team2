/*
 * ObstacleSpeedCritic
 * ──────────────────────────────────────────────────────────────
 * 장애물과의 거리·속도·각속도를 함께 고려해 페널티를 주는 DWB critic
 */
 #include "dwb_critics/obstacle_speed_critic.hpp"
 #include "nav2_util/node_utils.hpp"
 #include "pluginlib/class_list_macros.hpp"
 #include <rclcpp/rclcpp.hpp>
 #include <cmath>
 #include <algorithm>
 #include <stdexcept>
 
 namespace dwb_critics
 {
 
 ObstacleSpeedCritic::ObstacleSpeedCritic()
 : full_speed_distance_(4.0),
   stop_distance_(2.0),
   scale_(1.0),
   yaw_scale_(0.2)   // 선속도와 동일 비중
 {}
 
 ObstacleSpeedCritic::~ObstacleSpeedCritic() = default;
 
 // ──────────────────────────────────────────────────────────────
 void ObstacleSpeedCritic::onInit()
 {
   auto node = node_.lock();
   if (!node) {
     throw std::runtime_error("ObstacleSpeedCritic::onInit(): node expired");
   }
 
   using nav2_util::declare_parameter_if_not_declared;
   const std::string pfx = "FollowPath." + name_ + ".";
 
   // 1) 선언 (없으면 기본값 등록)
   declare_parameter_if_not_declared(node, pfx + "full_speed_distance",
                                     rclcpp::ParameterValue(full_speed_distance_));
   declare_parameter_if_not_declared(node, pfx + "stop_distance",
                                     rclcpp::ParameterValue(stop_distance_));
   declare_parameter_if_not_declared(node, pfx + "scale",
                                     rclcpp::ParameterValue(scale_));
   declare_parameter_if_not_declared(node, pfx + "yaw_scale",
                                     rclcpp::ParameterValue(yaw_scale_));
 
   // 2) 실제 값 읽기
   node->get_parameter(pfx + "full_speed_distance", full_speed_distance_);
   node->get_parameter(pfx + "stop_distance",       stop_distance_);
   node->get_parameter(pfx + "scale",               scale_);
   node->get_parameter(pfx + "yaw_scale",           yaw_scale_);
 
   // perception → critic 전역 파라미터 (장애물 상태·거리)
   node->declare_parameter("FollowPath.ObstacleSpeedCritic.obstacle_state",    0);
   node->declare_parameter("FollowPath.ObstacleSpeedCritic.obstacle_distance", 100.0);
 
   RCLCPP_INFO(node->get_logger(),
               "%s init ‣ full:%.2f stop:%.2f scale:%.2f yaw_scale:%.2f",
               name_.c_str(), full_speed_distance_, stop_distance_,
               scale_, yaw_scale_);
 }
 
 // ──────────────────────────────────────────────────────────────
 double ObstacleSpeedCritic::scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj)
 {
   auto node = node_.lock();
   if (!node) {return 0.0;}
 
   /* perception 노드가 주기적으로 업데이트하는 전역 상태 */
   int8_t obstacle_state   = 0;
   double obstacle_dist_m  = 100.0;
   node->get_parameter("FollowPath.ObstacleSpeedCritic.obstacle_state",   obstacle_state);
   node->get_parameter("FollowPath.ObstacleSpeedCritic.obstacle_distance", obstacle_dist_m);
 
   double penalty = 0.0;
 
   // 동적(2)·혼합(3) 상태에서만 속도 제한 활성화
   if (obstacle_state == 2 || obstacle_state == 3)
   {
     /* A. 완전 정지 영역 */
     if (obstacle_dist_m <= stop_distance_) {
       if (std::abs(traj.velocity.x) >= 0.01 || std::abs(traj.velocity.theta) >= 0.01) {
         return 10000;          // 즉시 불허
       }
     }
 
     /* B. 감속 영역 */
     if (obstacle_dist_m < full_speed_distance_) {
       const double ratio = std::clamp(
         (obstacle_dist_m - stop_distance_) /
         (full_speed_distance_ - stop_distance_), 0.0, 1.0);
 
       // ▸ 선속도 페널티
       penalty += std::abs(traj.velocity.x) * scale_ * (1.0 - ratio);
 
       // ▸ 각속도(yaw) 페널티
       penalty += std::abs(traj.velocity.theta) * yaw_scale_ * (1.0 - ratio);
     }
   }
 
   return penalty;   // free-speed 영역이면 0
 }
 
 // ──────────────────────────────────────────────────────────────
 PLUGINLIB_EXPORT_CLASS(dwb_critics::ObstacleSpeedCritic,
                        dwb_core::TrajectoryCritic)
 
 }  // namespace dwb_critics
 