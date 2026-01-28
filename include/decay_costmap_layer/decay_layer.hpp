#pragma once

#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "rclcpp/rclcpp.hpp"

namespace decay_costmap_layer
{

class DecayLayer : public nav2_costmap_2d::Layer
{
public:
  DecayLayer();
  virtual void onInitialize() override;
  virtual void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double* min_x, double* min_y, double* max_x, double* max_y) override;

  virtual void updateCosts(
    nav2_costmap_2d::Costmap2D& master_grid,
    int min_i, int min_j, int max_i, int max_j) override;

  virtual void reset() override;

  virtual bool isClearable() override;

private:
  double decay_time_;
  std::vector<rclcpp::Time> cell_times_;
};

}  // namespace decay_costmap_layer
