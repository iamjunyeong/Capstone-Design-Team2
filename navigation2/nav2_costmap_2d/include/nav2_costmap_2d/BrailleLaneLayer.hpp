#ifndef NAV2_COSTMAP_2D__BRAILLE_LANE_LAYER_HPP_
#define NAV2_COSTMAP_2D__BRAILLE_LANE_LAYER_HPP_

#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>      // PointCloud2 추가
#include <vector>
#include <mutex>
#include <memory>
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
namespace nav2_costmap_2d
{

class BrailleLaneLayer : public CostmapLayer
{
public:
  BrailleLaneLayer();
  ~BrailleLaneLayer() override;

  void onInitialize() override;
  void activate() override;
  void deactivate() override;
  void reset() override;
  bool isClearable() override;

private:
  // PointCloud2 콜백으로 변경
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  void updateBounds(double robot_x, double robot_y, double robot_yaw,
                    double* min_x, double* min_y, double* max_x, double* max_y) override;
  void updateCosts(Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) override;

  bool enabled_;
  bool need_recalculation_;

  // Subscription 타입을 PointCloud2로 변경
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_sub_;

  // 내부 처리를 위한 geometry_msgs::Point 벡터는 그대로 유지
  std::vector<geometry_msgs::msg::Point> received_points_;

  std::mutex mutex_;
  double max_influence_radius_;   // 파라미터: 최대 영향 거리
  unsigned char max_cost_;        // 파라미터: 최대 비용 값
};

} // namespace nav2_costmap_2d

#endif // NAV2_COSTMAP_2D__BRAILLE_LANE_LAYER_HPP_

