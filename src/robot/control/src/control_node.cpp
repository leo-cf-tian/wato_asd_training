#include "control_node.hpp"

ControlNode::ControlNode() : Node("control"), control_(robot::ControlCore(this->get_logger()))
{
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>("/path", 10, [this](const nav_msgs::msg::Path::SharedPtr msg)
        { path_ = msg; });
    odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom/filtered", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg)
        { odometry_ = msg; });
    vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&ControlNode::controlLoop, this));
}

void ControlNode::controlLoop()
{
    // Skip control if no path or odometry data is available
    if (!path_ || !odometry_)
    {
        return;
    }

    // Find the lookahead point
    auto lookahead_point = control_.findLookaheadPoint(path_, odometry_);
    if (!lookahead_point)
    {
        vel_pub_->publish(geometry_msgs::msg::Twist());
        return; // No valid lookahead point found
    }

    // Compute velocity command
    auto cmd_vel = control_.computeVelocity(odometry_, *lookahead_point);

    // Publish the velocity command
    vel_pub_->publish(cmd_vel);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControlNode>());
    rclcpp::shutdown();
    return 0;
}
