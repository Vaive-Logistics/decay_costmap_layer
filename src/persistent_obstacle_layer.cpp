#include "decay_costmap_layer/persistent_obstacle_layer.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

#include <algorithm>
#include <cmath>

namespace decay_costmap_layer
{

void PersistentObstacleLayer::onInitialize()
{
    nav2_costmap_2d::ObstacleLayer::onInitialize();

    declareParameter("obstacle_confirmation_time", rclcpp::ParameterValue(2.0));
    declareParameter("obstacle_observation_timeout", rclcpp::ParameterValue(0.2));

    auto node = node_.lock();
    node->get_parameter(name_ + ".obstacle_confirmation_time", obstacle_confirmation_time_);
    node->get_parameter(name_ + ".obstacle_observation_timeout", obstacle_observation_timeout_);

    RCLCPP_DEBUG(
        logger_,
        "Obstacle persistent layer enabled: confirmation_time = %.2f seconds",
        obstacle_confirmation_time_);
}

void PersistentObstacleLayer::reset()
{
    pending_obstacles_.clear();
    nav2_costmap_2d::ObstacleLayer::reset();
}

void PersistentObstacleLayer::updateBounds(
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

            // --- Look for an existing pending obstacle at this detection ---
            const unsigned int index = getIndex(mx, my);

            // --- New obstacle candidate ---
            auto [it, inserted] = 
                pending_obstacles_.try_emplace(index,
                                                PendingObstacle{px, py, now, now, false});

            auto & obstacle = it->second;

            obstacle.last_seen = now;
            
            // set lethal costmap for confirmed obstacles for every tick (until they disappear)
            if ((now - obstacle.first_seen).seconds() >= obstacle_confirmation_time_)
            {
                costmap_[index] = nav2_costmap_2d::LETHAL_OBSTACLE;
                touch(px, py, min_x, min_y, max_x, max_y);

                RCLCPP_DEBUG(
                    logger_,
                    "Added persistent obstacle at (%f, %f): "
                    "observed for %.2f seconds",
                    px,
                    py,
                    (now - obstacle.first_seen).seconds());
                
                obstacle.confirmed = true;
            }
        }
    }

    // --- Remove candidates that disappeared before confirmation ---
    for (auto it = pending_obstacles_.begin();
        it != pending_obstacles_.end();)
    {
        const auto & obstacle = it->second;

        if ((now - obstacle.last_seen).seconds() >
            obstacle_observation_timeout_)
        {
            RCLCPP_DEBUG(
                logger_,
                "Discarding obstacle (confirmed: %i) at (%f, %f): "
                "not observed for %.2f seconds",
                obstacle.confirmed,
                obstacle.x,
                obstacle.y,
                (now - obstacle.last_seen).seconds());

            if(obstacle.confirmed) // clear costmap cell
            {
                unsigned int mx, my;
                if (worldToMap(obstacle.x, obstacle.y, mx, my)) {
                    const unsigned int index = getIndex(mx, my);
                    costmap_[index] = nav2_costmap_2d::FREE_SPACE;
                }
            }

            it = pending_obstacles_.erase(it);
        }
        else {
            ++it;
        }
    }

    updateFootprint(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
}

}  // namespace decay_costmap_layer

PLUGINLIB_EXPORT_CLASS(
    decay_costmap_layer::PersistentObstacleLayer,
    nav2_costmap_2d::Layer)
