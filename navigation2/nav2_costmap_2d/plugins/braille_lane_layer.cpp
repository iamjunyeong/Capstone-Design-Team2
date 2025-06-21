#include "nav2_costmap_2d/BrailleLaneLayer.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/point.hpp"

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <vector>
#include <cmath>
#include <limits>
#include <algorithm>

namespace nav2_costmap_2d
{

static constexpr double EPSILON = 1e-3;      // 거리 비교를 위한 작은 값
static constexpr double BOUND_MARGIN = 0.5;  // updateBounds에 적용할 여유 공간 (미터)

// 점(px, py)과 선분(ax, ay) -> (bx, by) 사이의 최단 유클리드 거리 계산
double pointToSegmentDistance(double px, double py,
                              double ax, double ay,
                              double bx, double by)
{
  double dx = bx - ax;
  double dy = by - ay;
  if (std::abs(dx) < EPSILON && std::abs(dy) < EPSILON) {
    dx = px - ax;
    dy = py - ay;
    return std::sqrt(dx * dx + dy * dy);
  }
  double t = ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy);
  t = std::max(0.0, std::min(1.0, t));
  double proj_x = ax + t * dx;
  double proj_y = ay + t * dy;
  double dist_x = px - proj_x;
  double dist_y = py - proj_y;
  return std::sqrt(dist_x * dist_x + dist_y * dist_y);
}

BrailleLaneLayer::BrailleLaneLayer()
: CostmapLayer(),
  enabled_(true),
  need_recalculation_(false)
{
}

BrailleLaneLayer::~BrailleLaneLayer()
{
}

void BrailleLaneLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    RCLCPP_ERROR(rclcpp::get_logger("BrailleLaneLayer"),
                 "onInitialize(): Failed to lock node pointer");
    return;
  }
  RCLCPP_INFO(node->get_logger(), "BrailleLaneLayer::onInitialize() called.");

  declareParameter("enabled", rclcpp::ParameterValue(true));
  std::string enabled_param_name = name_ + "." + "enabled";
  node->get_parameter(enabled_param_name, enabled_);

  current_ = true;
  need_recalculation_ = true;
  matchSize();

  RCLCPP_INFO(node->get_logger(),
              "BrailleLaneLayer onInitialize(): layer_name='%s', enabled=%s",
              name_.c_str(), enabled_ ? "true" : "false");
}

void BrailleLaneLayer::activate()
{
  auto node = node_.lock();
  if (!node) return;

  auto qos = rclcpp::SensorDataQoS();
  // Point -> PointCloud2 로 바꿉니다.
  points_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/center_line_point",
    qos,
    std::bind(&BrailleLaneLayer::cloudCallback, this, std::placeholders::_1)
  );

  RCLCPP_INFO(node->get_logger(),
    "BrailleLaneLayer activated! Subscribed to %s",
    points_sub_->get_topic_name());
}

void BrailleLaneLayer::deactivate()
{
  RCLCPP_INFO(rclcpp::get_logger("BrailleLaneLayer"), "BrailleLaneLayer::deactivate() called.");
  if (points_sub_) {
    points_sub_.reset();
    RCLCPP_INFO(rclcpp::get_logger("BrailleLaneLayer"),
                "BrailleLaneLayer deactivated: subscription reset!");
  }
}

void BrailleLaneLayer::reset()
{
  RCLCPP_INFO(rclcpp::get_logger("BrailleLaneLayer"), "BrailleLaneLayer::reset() called.");
  std::lock_guard<std::mutex> lock(mutex_);
  received_points_.clear();
  current_ = false;
  need_recalculation_ = true;
  RCLCPP_INFO(rclcpp::get_logger("BrailleLaneLayer"),
              "reset(): received_points_ cleared, need_recalculation set to true");
}

bool BrailleLaneLayer::isClearable()
{
  return true;
}

void BrailleLaneLayer::cloudCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  received_points_.clear();

  if (msg->width * msg->height == 0) {
    return;
  }

  geometry_msgs::msg::TransformStamped transform;
  try {
    // getGlobalFrameID()를 사용하는 수정된 코드
    transform = tf_->lookupTransform(
      layered_costmap_->getGlobalFrameID(), msg->header.frame_id, msg->header.stamp,
      rclcpp::Duration::from_seconds(0.5));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(
      rclcpp::get_logger("BrailleLaneLayer"),
      "Could not transform %s to %s: %s",
      msg->header.frame_id.c_str(), layered_costmap_->getGlobalFrameID().c_str(), ex.what());
    return;
  }

  // 이 부분부터는 원래 코드와 동일합니다.
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    geometry_msgs::msg::Point p_in;
    p_in.x = *iter_x;
    p_in.y = *iter_y;
    p_in.z = *iter_z;

    geometry_msgs::msg::Point p_out;
    tf2::doTransform(p_in, p_out, transform);

    received_points_.push_back(p_out);
  }

  need_recalculation_ = true;
}

void BrailleLaneLayer::updateBounds(double robot_x, double robot_y, double robot_yaw,
                                      double* min_x, double* min_y, double* max_x, double* max_y)
{
  (void)robot_x;
  (void)robot_y;
  (void)robot_yaw;

  if (!enabled_) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (received_points_.empty()) {
    return;
  }

  double layer_min_x = std::numeric_limits<double>::infinity();
  double layer_min_y = std::numeric_limits<double>::infinity();
  double layer_max_x = -std::numeric_limits<double>::infinity();
  double layer_max_y = -std::numeric_limits<double>::infinity();

  for (const auto &pt : received_points_) {
    layer_min_x = std::min(layer_min_x, pt.x);
    layer_min_y = std::min(layer_min_y, pt.y);
    layer_max_x = std::max(layer_max_x, pt.x);
    layer_max_y = std::max(layer_max_y, pt.y);
  }

  *min_x = std::min(*min_x, layer_min_x - BOUND_MARGIN);
  *min_y = std::min(*min_y, layer_min_y - BOUND_MARGIN);
  *max_x = std::max(*max_x, layer_max_x + BOUND_MARGIN);
  *max_y = std::max(*max_y, layer_max_y + BOUND_MARGIN);
}

void BrailleLaneLayer::updateCosts(Costmap2D & master_grid,
                                     int min_i, int min_j,
                                     int max_i, int max_j)
{
  if (!enabled_) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (received_points_.size() < 2) {
    return;
  }

  unsigned int size_x = master_grid.getSizeInCellsX();
  unsigned int size_y = master_grid.getSizeInCellsY();

  min_i = std::max(0, min_i);
  min_j = std::max(0, min_j);
  max_i = std::min((int)size_x, max_i);
  max_j = std::min((int)size_y, max_j);

  // 점자블록 좌표로부터의 거리에 따른 비용 설정을 위한 상수
  const double distance_threshold = 0.2; // 20cm
  const unsigned char cost_within_threshold = 0;
  const unsigned char cost_outside_threshold = 150;


  for (int j = min_j; j < max_j; ++j) {
    for (int i = min_i; i < max_i; ++i) {
      double wx, wy;
      master_grid.mapToWorld(i, j, wx, wy);

      double min_dist = std::numeric_limits<double>::infinity();
      for (size_t k = 0; k < received_points_.size() - 1; ++k) {
        double dist = pointToSegmentDistance(wx, wy,
                                             received_points_[k].x, received_points_[k].y,
                                             received_points_[k + 1].x, received_points_[k + 1].y);
        min_dist = std::min(min_dist, dist);
      }

      unsigned char calculated_cost = nav2_costmap_2d::FREE_SPACE;
      if (min_dist != std::numeric_limits<double>::infinity()) {
        // ### 수정된 부분 ###
        // 최소 거리가 20cm보다 작거나 같으면 cost를 0으로, 크면 150으로 설정
        if (min_dist <= distance_threshold) {
          calculated_cost = cost_within_threshold;
        } else {
          calculated_cost = cost_outside_threshold;
        }
      }

      unsigned char old_cost = master_grid.getCost(i, j);
      if (old_cost != nav2_costmap_2d::LETHAL_OBSTACLE && old_cost != nav2_costmap_2d::NO_INFORMATION) {
        master_grid.setCost(i, j, std::max(calculated_cost, old_cost));
      }
    }
  }

  need_recalculation_ = false;
  current_ = true;
}

} // namespace nav2_costmap_2d

PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::BrailleLaneLayer, nav2_costmap_2d::Layer)
