#include "planner_core.hpp"

#include <queue>
#include <map>

namespace robot
{

PlannerCore::PlannerCore(const rclcpp::Logger& logger) 
: logger_(logger) {}

struct Node {
    int x, y;
    float cost;

    bool operator>(const Node &other) const {
        return cost > other.cost;
    }
};

float heuristic(int x1, int y1, int x2, int y2) {
    // Use Euclidean distance as the heuristic
    return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
}

bool isValid(int x, int y, const nav_msgs::msg::OccupancyGrid::SharedPtr &map) {
    if (x < 0 || y < 0 || x >= (int)(map->info.width) || y >= (int)(map->info.height)) {
        return false;
    }
    int index = y * map->info.width + x;
    return map->data[index] == 0;  // Assuming 0 indicates free space
}

std::vector<std::pair<float, float>> reconstructPath(
    const std::map<std::pair<int, int>, std::pair<int, int>> &cameFrom,
    std::pair<int, int> current,
    float resolution,
    float originX,
    float originY) {
    std::vector<std::pair<float, float>> path;
    while (cameFrom.find(current) != cameFrom.end()) {
        // Convert grid coordinates to real-world coordinates
        float realX = current.first * resolution + originX;
        float realY = current.second * resolution + originY;
        path.emplace_back(realX, realY);
        current = cameFrom.at(current);
    }
    // Add the starting point in real-world coordinates
    float realX = current.first * resolution + originX;
    float realY = current.second * resolution + originY;
    path.emplace_back(realX, realY);

    std::reverse(path.begin(), path.end());
    return path;
}

std::vector<std::pair<float, float>> PlannerCore::aStar(nav_msgs::msg::OccupancyGrid::SharedPtr map, 
                                           nav_msgs::msg::Odometry::SharedPtr odometry, 
                                           geometry_msgs::msg::PointStamped::SharedPtr goal) {
    // Extract map properties
    float resolution = map->info.resolution;
    float originX = map->info.origin.position.x;
    float originY = map->info.origin.position.y;

    // Extract start and goal positions
    int startX = static_cast<int>((odometry->pose.pose.position.x - originX) / resolution);
    int startY = static_cast<int>((odometry->pose.pose.position.y - originY) / resolution);
    int goalX = static_cast<int>((goal->point.x - originX) / resolution);
    int goalY = static_cast<int>((goal->point.y - originY) / resolution);

    // Priority queue for open set (min-heap based on cost)
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> openSet;
    openSet.push({startX, startY, 0.0});

    // Maps to track costs and paths
    std::map<std::pair<int, int>, float> gScore;
    std::map<std::pair<int, int>, std::pair<int, int>> cameFrom;

    gScore[{startX, startY}] = 0.0;

    // Direction vectors for neighbors (4-connectivity or 8-connectivity)
    std::vector<std::pair<int, int>> directions = {{1, 0}, {0, 1}, {-1, 0}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};

    while (!openSet.empty()) {
        Node current = openSet.top();
        openSet.pop();

        if (current.x == goalX && current.y == goalY) {
            // Goal reached, reconstruct path
            return reconstructPath(cameFrom, {goalX, goalY}, resolution, originX, originY);
        }

        for (const auto &dir : directions) {
            int neighborX = current.x + dir.first;
            int neighborY = current.y + dir.second;

            if (!isValid(neighborX, neighborY, map)) {
                continue;
            }

            float tentativeGScore = gScore[{current.x, current.y}] + heuristic(current.x, current.y, neighborX, neighborY);

            if (gScore.find({neighborX, neighborY}) == gScore.end() || tentativeGScore < gScore[{neighborX, neighborY}]) {
                cameFrom[{neighborX, neighborY}] = {current.x, current.y};
                gScore[{neighborX, neighborY}] = tentativeGScore;
                float fScore = tentativeGScore + heuristic(neighborX, neighborY, goalX, goalY);
                openSet.push({neighborX, neighborY, fScore});
            }
        }
    }

    // If the goal was not reached, return an empty path
    return {};
}

bool PlannerCore::goalReached(nav_msgs::msg::OccupancyGrid::SharedPtr map, nav_msgs::msg::Odometry::SharedPtr odometry, geometry_msgs::msg::PointStamped::SharedPtr goal) {
    
    double positionX = odometry->pose.pose.position.x;
    double positionY = odometry->pose.pose.position.y;
    
    double goalX = goal->point.x;
    double goalY = goal->point.y;

    return std::sqrt(std::pow(positionX - goalX, 2) + std::pow(positionY - goalY, 2)) < map->info.resolution * 2;
}

} 
