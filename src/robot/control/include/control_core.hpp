#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace robot
{

    class ControlCore {
    public:
        // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
        ControlCore(const rclcpp::Logger& logger);

        std::optional<geometry_msgs::msg::PoseStamped> findLookaheadPoint(nav_msgs::msg::Path::SharedPtr path, nav_msgs::msg::Odometry::SharedPtr odometry);
        geometry_msgs::msg::Twist computeVelocity(nav_msgs::msg::Odometry::SharedPtr odometry, const geometry_msgs::msg::PoseStamped& target);


    private:
        rclcpp::Logger logger_;

        float lookahead_distance_ = 1.0f;  // Lookahead distance
        float goal_tolerance_ = 0.1f;     // Distance to consider the goal reached
        float linear_speed_ = 0.5f;       // Constant forward speed

        double computeDistance(const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b);
        double extractYaw(const geometry_msgs::msg::Quaternion& quat);

    };

}

#endif 
