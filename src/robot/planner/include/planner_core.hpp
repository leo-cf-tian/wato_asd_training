#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

namespace robot
{

    class PlannerCore {
    public:
        explicit PlannerCore(const rclcpp::Logger& logger);

        std::vector<std::pair<float, float>> aStar(nav_msgs::msg::OccupancyGrid::SharedPtr map, nav_msgs::msg::Odometry::SharedPtr odometry, geometry_msgs::msg::PointStamped::SharedPtr goal);
        bool goalReached(nav_msgs::msg::OccupancyGrid::SharedPtr map, nav_msgs::msg::Odometry::SharedPtr odometry, geometry_msgs::msg::PointStamped::SharedPtr goal);

    private:
        rclcpp::Logger logger_;
    };

}

#endif  
