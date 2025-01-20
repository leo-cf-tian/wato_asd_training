#ifndef CONTROL_NODE_HPP_
#define CONTROL_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "control_core.hpp"

class ControlNode : public rclcpp::Node {
  public:
    ControlNode();

  private:
    robot::ControlCore control_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;

    rclcpp::TimerBase::SharedPtr timer_;

    void pathCallback(nav_msgs::msg::Path::SharedPtr path);
    void odometryCallback(nav_msgs::msg::Odometry::SharedPtr odometry);

    void controlLoop();

    nav_msgs::msg::Path::SharedPtr path_;
    nav_msgs::msg::Odometry::SharedPtr odometry_;
};

#endif
