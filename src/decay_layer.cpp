#include "decay_costmap_layer/decay_layer.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace decay_costmap_layer
{

DecayLayer::DecayLayer() = default;

void DecayLayer::onInitialize()
{
  auto node = node_.lock();

  declareParameter("decay_time", rclcpp::ParameterValue(0.5));
  node->get_parameter(name_ + ".decay_time", decay_time_);

  matchSize();

  auto * costmap = layered_costmap_->getCostmap();
  unsigned int size_x = costmap->getSizeInCellsX();
  unsigned int size_y = costmap->getSizeInCellsY();

  // 0 timestamp = never observed
  cell_times_.assign(
    size_x * size_y,
    rclcpp::Time(0, 0, RCL_ROS_TIME));

  current_ = true;
}

void DecayLayer::reset()
{
  auto node = node_.lock();
  std::fill(
    cell_times_.begin(),
    cell_times_.end(),
    rclcpp::Time(0, 0, RCL_ROS_TIME));
}

void DecayLayer::updateBounds(
  double, double, double,
  double * min_x, double * min_y,
  double * max_x, double * max_y)
{
  // Only affect the region already updated by obstacle layer
  // So we don’t modify bounds
}

void DecayLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master,
  int min_i, int min_j,
  int max_i, int max_j)
{
  auto node = node_.lock();
  auto now = node->now();

  auto * costmap = layered_costmap_->getCostmap();
  unsigned int size =
    costmap->getSizeInCellsX() *
    costmap->getSizeInCellsY();

  // Handle rolling window resize
  if (cell_times_.size() != size) {
    cell_times_.assign(
      size,
      rclcpp::Time(0, 0, RCL_ROS_TIME));
  }

  constexpr unsigned char DYNAMIC_THRESHOLD = nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;

  for (int j = min_j; j < max_j; ++j) {
    for (int i = min_i; i < max_i; ++i) {

      unsigned int idx = master.getIndex(i, j);
      unsigned char cost = master.getCost(i, j);

      // Refresh timestamp for dynamic obstacles
      if (cost >= DYNAMIC_THRESHOLD &&
          cost != nav2_costmap_2d::NO_INFORMATION && 
          !(cell_times_[idx].nanoseconds() > 0) )
      {
        cell_times_[idx] = now;
      }

      // Decay only cells previously marked by this layer
      if (cell_times_[idx].nanoseconds() > 0) {
        if ((now - cell_times_[idx]).seconds() > decay_time_) {
          master.setCost(i, j, nav2_costmap_2d::FREE_SPACE);
          cell_times_[idx] = rclcpp::Time(0, 0, RCL_ROS_TIME);
        }
      }
    }
  }
}

bool DecayLayer::isClearable()
{
  return true;
}

}  // namespace decay_costmap_layer

PLUGINLIB_EXPORT_CLASS(
  decay_costmap_layer::DecayLayer,
  nav2_costmap_2d::Layer)
