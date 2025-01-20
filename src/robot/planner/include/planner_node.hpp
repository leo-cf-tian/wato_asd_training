#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

#include "planner_core.hpp"

class PlannerNode : public rclcpp::Node {
public:
    PlannerNode();

private:
    enum class State { NO_GOAL, MOVING_TO_GOAL, REACHED_GOAL };

    robot::PlannerCore planner_;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr point_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;

    rclcpp::TimerBase::SharedPtr timer_;

    State state_ = State::NO_GOAL;
    geometry_msgs::msg::PointStamped::SharedPtr goal_;
    nav_msgs::msg::Odometry::SharedPtr latest_odometry_;
    nav_msgs::msg::OccupancyGrid::SharedPtr latest_map_;

    void mapCallback(nav_msgs::msg::OccupancyGrid::SharedPtr map);
    void odometryCallback(nav_msgs::msg::Odometry::SharedPtr odometry);
    void pointCallback(geometry_msgs::msg::PointStamped::SharedPtr point);

    void timerCallback();
    void planPath();
};

#endif 
