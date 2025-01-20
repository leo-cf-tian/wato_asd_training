#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace robot
{

    class CostmapCore {
    public:
        // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
        explicit CostmapCore(const rclcpp::Logger& logger);
        std::vector<std::vector<float>> createOccupancyGrid(const sensor_msgs::msg::LaserScan::SharedPtr scan, float cellSize);
        std::vector<std::vector<float>> createCostmap(std::vector<std::vector<float>> occupancyGrid, float inflationRadius);

    private:
        rclcpp::Logger logger_;

    };

}

#endif  