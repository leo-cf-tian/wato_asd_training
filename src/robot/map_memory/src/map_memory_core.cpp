#include "map_memory_core.hpp"

namespace robot
{

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger, float width, float height, float resolution)
  : logger_(logger), dimensions(std::make_pair(width, height)), resolution(resolution)
{
  int gridWidth = (int)std::ceil(width / resolution);
  int gridHeight = (int)std::ceil(height / resolution);
  globalMap = std::vector<std::vector<float>>(gridHeight, std::vector<float>(gridWidth, 0));
}

std::vector<std::vector<float>> MapMemoryCore::syncMap(const nav_msgs::msg::OccupancyGrid::SharedPtr costmap, const nav_msgs::msg::Odometry::SharedPtr odometry) {
    int width = costmap->info.width;
    int height = costmap->info.height;
    float resolution = costmap->info.resolution;
    float originX = costmap->info.origin.position.x;
    float originY = costmap->info.origin.position.y;

    float robotX = odometry->pose.pose.position.x;
    float robotY = odometry->pose.pose.position.y;
    float robotYaw = 2.0 * atan2(odometry->pose.pose.orientation.z, odometry->pose.pose.orientation.w);

    float cosYaw = cos(robotYaw);
    float sinYaw = sin(robotYaw);

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            float cost_value = costmap->data[y * width + x];
            if (cost_value < 0) continue;

            float localX = originX + x * resolution;
            float localY = originY + y * resolution;

            float globalX = cosYaw * localX - sinYaw * localY + robotX;
            float globalY = sinYaw * localX + cosYaw * localY + robotY;

            int gridX = (int)((globalX + dimensions.first / 2) / resolution);
            int gridY = (int)((globalY + dimensions.second / 2) / resolution);

            if (gridX >= 0 && gridX < globalMap[0].size() && gridY >= 0 && gridY < globalMap.size()) {
                globalMap[gridY][gridX] = std::max(globalMap[gridY][gridX], cost_value);
            }
        }
    }

    return globalMap;
}

}