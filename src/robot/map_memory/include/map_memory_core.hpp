#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <vector>

namespace robot
{

    class MapMemoryCore {
    public:
        explicit MapMemoryCore(const rclcpp::Logger& logger, float width, float height, float resolution);

        std::vector<std::vector<float>> syncMap(const nav_msgs::msg::OccupancyGrid::SharedPtr costmap, const nav_msgs::msg::Odometry::SharedPtr odometry);
    private:
        rclcpp::Logger logger_;

        std::vector<std::vector<float>> globalMap;

        std::pair<float, float> dimensions;
        float resolution;
    };

}

#endif  
