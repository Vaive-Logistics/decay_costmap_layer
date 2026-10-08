#pragma once

#include "nav2_costmap_2d/obstacle_layer.hpp"
#include <unordered_map>

namespace decay_costmap_layer
{

struct PendingObstacle
{
    double x, y;     // global coordinates
    rclcpp::Time first_seen;
    rclcpp::Time last_seen;
    bool confirmed{false};
};

class PersistentObstacleLayer : public nav2_costmap_2d::ObstacleLayer
{
public:
  void onInitialize() override;

  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y,
    double * max_x, double * max_y) override;

  void reset() override;

protected:
  
  // List of all obstacles with their map coordinates and first + last seen timestamp
  std::unordered_map<unsigned int, PendingObstacle> pending_obstacles_;
  
  double obstacle_confirmation_time_{2.0};
  double obstacle_observation_timeout_{0.2};
};

}  // namespace decay_costmap_layer
