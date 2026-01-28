#pragma once

#include "nav2_costmap_2d/obstacle_layer.hpp"
#include <unordered_map>

namespace decay_costmap_layer
{

struct ObstacleInfo {
  double x, y;               // global coordinates
  rclcpp::Time last_seen;    // timestamp
};

class DecayObstacleLayer : public nav2_costmap_2d::ObstacleLayer
{
public:
  void onInitialize() override;

  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y,
    double * max_x, double * max_y) override;

  void reset() override;

protected:
  void removeStaleObstacles(
    double * min_x, double * min_y,
    double * max_x, double * max_y);
  
  // List of all obstacles with their global coordinates and last seen timestamp
  std::vector<ObstacleInfo> obstacle_last_seen_;
  
  double obstacle_timeout_{2.0};
};

}  // namespace decay_costmap_layer
