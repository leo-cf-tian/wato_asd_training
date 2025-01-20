#include "planner_node.hpp"

PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger())) {
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
  odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom/filtered", 10, std::bind(&PlannerNode::odometryCallback, this, std::placeholders::_1));
  point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>("/goal_point", 10, std::bind(&PlannerNode::pointCallback, this, std::placeholders::_1));

  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

  timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&PlannerNode::timerCallback, this));
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    latest_map_ = msg;
    if (state_ == State::MOVING_TO_GOAL) {
        planPath();
    }
}

void PlannerNode::pointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
    goal_ = msg;
    state_ = State::MOVING_TO_GOAL;
    planPath();
}

void PlannerNode::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    latest_odometry_ = msg;
}

void PlannerNode::timerCallback() {
    if (state_ == State::MOVING_TO_GOAL) {
        if (planner_.goalReached(latest_map_, latest_odometry_, goal_)) {
            RCLCPP_INFO(this->get_logger(), "Goal reached!");
            state_ = State::NO_GOAL;
        } else {
            RCLCPP_INFO(this->get_logger(), "Replanning due to timeout or progress...");
            planPath();
        }
    }
}

void PlannerNode::planPath() {
    std::vector<std::pair<float, float>> pathCoords = planner_.aStar(latest_map_, latest_odometry_, goal_);

    nav_msgs::msg::Path path;

    path.header.stamp = this->get_clock()->now();
    path.header.frame_id = "sim_world";

    path.poses = {};
    for (auto& coord : pathCoords) {
      geometry_msgs::msg::PoseStamped poseMsg;

      poseMsg.header = path.header;
      poseMsg.pose.position.x = coord.first;
      poseMsg.pose.position.y = coord.second;

      path.poses.push_back(poseMsg);
    }

    path_pub_->publish(path);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
