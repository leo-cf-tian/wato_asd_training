#include <chrono>
#include <memory>

#include "costmap_node.hpp"

CostmapNode::CostmapNode()
    : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
    // Initialize the constructs and their parameters
    costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/lidar", 10, std::bind(&CostmapNode::receiveLidarScan, this, std::placeholders::_1));
}

void CostmapNode::receiveLidarScan(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    std::vector<std::vector<float>> occupancyGrid = costmap_.createOccupancyGrid(scan, 0.1f);
    std::vector<std::vector<float>> costmap = costmap_.createCostmap(occupancyGrid, 1 / 0.1f);

    std::vector<int8_t> costmapArray(costmap.size() * costmap[0].size());

    for (int i = 0; i < costmap.size(); ++i) {
        for (int j = 0; j < costmap[i].size(); ++j) {
            costmapArray[i * costmap.size() + j] = (int)costmap[i][j];
        }
    }

    nav_msgs::msg::OccupancyGrid occupancyMsg;

    occupancyMsg.data = costmapArray;
    occupancyMsg.info.width = costmap.size();
    occupancyMsg.info.height = costmap[0].size();
    occupancyMsg.info.resolution = 0.1f;


    occupancyMsg.info.origin.position.x = -(costmap.size() / 2 * 0.1);
    occupancyMsg.info.origin.position.y = -(costmap[0].size() / 2 * 0.1);

    occupancyMsg.header.frame_id = "robot/chassis/lidar";

    costmap_pub_->publish(occupancyMsg);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CostmapNode>());
    rclcpp::shutdown();
    return 0;
}