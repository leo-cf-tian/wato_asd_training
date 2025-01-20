#include "control_core.hpp"

namespace robot
{

    ControlCore::ControlCore(const rclcpp::Logger& logger)
        : logger_(logger) {
    }


    std::optional<geometry_msgs::msg::PoseStamped> ControlCore::findLookaheadPoint(
        nav_msgs::msg::Path::SharedPtr path,
        nav_msgs::msg::Odometry::SharedPtr odometry
    ) {
        const geometry_msgs::msg::Point robot_pose = odometry->pose.pose.position;
        const geometry_msgs::msg::Quaternion robot_orientation = odometry->pose.pose.orientation;

        // Check if the robot is close enough to the goal
        const auto& goal_pose = path->poses.back().pose.position;
        if (computeDistance(robot_pose, goal_pose) <= goal_tolerance_) {
            return std::nullopt;
        }

        // Find the first point on the path that is approximately the lookahead distance away
        for (const auto& pose : path->poses) {
            const auto& path_point = pose.pose.position;
            if (computeDistance(robot_pose, path_point) >= lookahead_distance_) {
                return pose;
            }
        }

        // If no point meets the criteria, return the goal as the fallback lookahead point
        return path->poses.back();
    }

    geometry_msgs::msg::Twist ControlCore::computeVelocity(
        nav_msgs::msg::Odometry::SharedPtr odometry,
        const geometry_msgs::msg::PoseStamped& target
    ) {
        geometry_msgs::msg::Twist cmd_vel;

        // Extract robot's current position and orientation
        float robot_x = odometry->pose.pose.position.x;
        float robot_y = odometry->pose.pose.position.y;

        float robot_yaw = extractYaw(odometry->pose.pose.orientation);

        // Extract target position
        float target_x = target.pose.position.x;
        float target_y = target.pose.position.y;

        // Compute the relative position of the target in the robot's frame
        float dx = target_x - robot_x;
        float dy = target_y - robot_y;

        float target_x_robot = std::cos(-robot_yaw) * dx - std::sin(-robot_yaw) * dy;
        float target_y_robot = std::sin(-robot_yaw) * dx + std::cos(-robot_yaw) * dy;

        // Calculate curvature (inverse of turning radius)
        float curvature = 2 * target_y_robot / (lookahead_distance_ * lookahead_distance_);

        // Compute velocity commands
        cmd_vel.linear.x = linear_speed_;
        cmd_vel.angular.z = linear_speed_ * curvature;

        return cmd_vel;
    }

    double ControlCore::computeDistance(const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b) {
        return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
    }

    double ControlCore::extractYaw(const geometry_msgs::msg::Quaternion& quat) {
        return std::atan2(2.0 * (quat.w * quat.z), 1.0 - 2.0 * (quat.z * quat.z));
    }

}
