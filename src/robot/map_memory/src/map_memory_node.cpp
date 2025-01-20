#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode()
    : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger(), 30, 30, 0.1f))
{
    costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/costmap", 10, std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));
    odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom/filtered", 10, std::bind(&MapMemoryNode::odometryCallback, this, std::placeholders::_1));

    map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&MapMemoryNode::syncMap, this));
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr costmap) {
    latest_costmap_ = costmap;
    costmap_updated_ = true;
}

void MapMemoryNode::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr odometry) {
    double x = odometry->pose.pose.position.x;
    double y = odometry->pose.pose.position.y;

    if (latest_odometry_ == nullptr) {
        latest_odometry_ = odometry;
        should_update_map_ = true;
        return;
    }

    double last_x = latest_odometry_->pose.pose.position.x;
    double last_y = latest_odometry_->pose.pose.position.y;

    double distance = std::sqrt(std::pow(x - last_x, 2) + std::pow(y - last_y, 2));
    if (distance >= update_threshold_ && !should_update_map_) {
        latest_odometry_ = odometry;
        should_update_map_ = true;
    }
}

void MapMemoryNode::syncMap() {
    if (should_update_map_ && costmap_updated_) {
        std::vector<std::vector<float>> globalMap = map_memory_.syncMap(latest_costmap_, latest_odometry_);

        std::vector<int8_t> globalArray(globalMap.size() * globalMap[0].size());

        for (int i = 0; i < globalMap.size(); ++i) {
            for (int j = 0; j < globalMap[i].size(); ++j) {
                globalArray[i * globalMap.size() + j] = (int)globalMap[i][j];
            }
        }

        nav_msgs::msg::OccupancyGrid occupancyMsg;

        occupancyMsg.data = globalArray;
        occupancyMsg.info.width = globalMap.size();
        occupancyMsg.info.height = globalMap[0].size();
        occupancyMsg.info.resolution = 0.1f;


        occupancyMsg.info.origin.position.x = -(globalMap.size() / 2 * 0.1);
        occupancyMsg.info.origin.position.y = -(globalMap[0].size() / 2 * 0.1);

        occupancyMsg.header.frame_id = "/sim_world";

        map_pub_->publish(occupancyMsg);

        should_update_map_ = false;
        costmap_updated_ = false;
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapMemoryNode>());
    rclcpp::shutdown();
    return 0;
}
