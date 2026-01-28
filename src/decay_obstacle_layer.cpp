#include "decay_costmap_layer/decay_obstacle_layer.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace decay_costmap_layer
{

void DecayObstacleLayer::onInitialize()
{
    nav2_costmap_2d::ObstacleLayer::onInitialize();

    declareParameter("obstacle_timeout", rclcpp::ParameterValue(2.0));

    auto node = node_.lock();
    node->get_parameter(name_ + ".obstacle_timeout", obstacle_timeout_);

    RCLCPP_INFO(
        logger_,
        "Obstacle decay enabled: timeout = %.2f seconds",
        obstacle_timeout_);
}

void DecayObstacleLayer::reset()
{
    obstacle_last_seen_.clear();
    nav2_costmap_2d::ObstacleLayer::reset();
}

void DecayObstacleLayer::removeStaleObstacles(
    double * min_x, double * min_y,
    double * max_x, double * max_y)
{
    auto node = node_.lock();
    rclcpp::Time now = node->get_clock()->now();

    for (auto it = obstacle_last_seen_.begin(); it != obstacle_last_seen_.end();) {
        if ((now - it->last_seen).seconds() > obstacle_timeout_) {

            // convert global coordinates to rolling costmap cell
            unsigned int mx, my;
            if (worldToMap(it->x, it->y, mx, my)) {
                unsigned int index = getIndex(mx, my);
                costmap_[index] = nav2_costmap_2d::FREE_SPACE;
                touch(it->x, it->y, min_x, min_y, max_x, max_y);

                RCLCPP_DEBUG(
                    logger_,
                    "Decayed obstacle at %.2f, %.2f (index %u)",
                    it->x, it->y, index);
            }

            it = obstacle_last_seen_.erase(it);
        } else {
            ++it;
        }
    }
}

void DecayObstacleLayer::updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y,
    double * max_x, double * max_y)
{
    std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());

    if (rolling_window_) {
        updateOrigin(
            robot_x - getSizeInMetersX() / 2,
            robot_y - getSizeInMetersY() / 2);
    }

    if (!enabled_) {
        return;
    }

    useExtraBounds(min_x, min_y, max_x, max_y);

    // --- Remove stale obstacles FIRST ---
    removeStaleObstacles(min_x, min_y, max_x, max_y);

    bool current = true;
    std::vector<nav2_costmap_2d::Observation> observations;
    std::vector<nav2_costmap_2d::Observation> clearing_observations;

    current &= getMarkingObservations(observations);
    current &= getClearingObservations(clearing_observations);
    current_ = current;

    // --- Raytrace freespace ---
    for (const auto & obs : clearing_observations) {
        raytraceFreespace(obs, min_x, min_y, max_x, max_y);
    }

    auto node = node_.lock();
    rclcpp::Time now = node->get_clock()->now();

    // --- Mark obstacles (COPIED & MODIFIED FROM NAV2) ---
    for (const auto & obs : observations) {
        const auto & cloud = *(obs.cloud_);

        const double sq_max = obs.obstacle_max_range_ * obs.obstacle_max_range_;
        const double sq_min = obs.obstacle_min_range_ * obs.obstacle_min_range_;

        sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
            const double px = *iter_x;
            const double py = *iter_y;
            const double pz = *iter_z;

            if (pz < min_obstacle_height_ || pz > max_obstacle_height_) {
                continue;
            }

            const double dx = px - obs.origin_.x;
            const double dy = py - obs.origin_.y;
            const double dz = pz - obs.origin_.z;
            const double sq_dist = dx * dx + dy * dy + dz * dz;

            if (sq_dist >= sq_max || sq_dist < sq_min) {
                continue;
            }

            unsigned int mx, my;
            if (!worldToMap(px, py, mx, my)) {
                continue;
            }

            unsigned int index = getIndex(mx, my);
            costmap_[index] = nav2_costmap_2d::LETHAL_OBSTACLE;
            touch(px, py, min_x, min_y, max_x, max_y);

            // Store global position for rolling map decay
            obstacle_last_seen_.push_back({px, py, now});
        }
    }

    updateFootprint(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
}

}  // namespace decay_costmap_layer

PLUGINLIB_EXPORT_CLASS(
    decay_costmap_layer::DecayObstacleLayer,
    nav2_costmap_2d::Layer)
