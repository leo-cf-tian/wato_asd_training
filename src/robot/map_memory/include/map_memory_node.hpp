#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "map_memory_core.hpp"

class MapMemoryNode : public rclcpp::Node {
public:
    MapMemoryNode();

private:
    robot::MapMemoryCore map_memory_;

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;

    rclcpp::TimerBase::SharedPtr timer_;

    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr costmap);
    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr odometry);

    void syncMap();

    nav_msgs::msg::OccupancyGrid::SharedPtr latest_costmap_;
    nav_msgs::msg::Odometry::SharedPtr latest_odometry_;

    bool costmap_updated_ = false;
    std::pair<float, float> lastSyncPos;
    bool should_update_map_ = true;

    float update_threshold_ = 3.0f;
};

#endif 
