#include "costmap_core.hpp"
#include <algorithm>

namespace robot
{

    CostmapCore::CostmapCore(const rclcpp::Logger& logger) : logger_(logger) {}

    std::vector<std::vector<float>> CostmapCore::createOccupancyGrid(const sensor_msgs::msg::LaserScan::SharedPtr scan, float cellSize) {

        float gridLength = scan->range_max - scan->range_min;
        int gridSize = (int)std::ceil(gridLength / cellSize);

        std::vector<std::vector<float>> occupancyGrid(gridSize, std::vector<float>(gridSize, 0));

        for (size_t i = 0; i < scan->ranges.size(); ++i) {
            double angle = scan->angle_min + i * scan->angle_increment;
            double range = scan->ranges[i];
            if (range < scan->range_max && range > scan->range_min) {
                int x_grid = (int)std::floor((range * std::cos(angle) + gridLength / 2) / cellSize);
                int y_grid = (int)std::floor((range * std::sin(angle) + gridLength / 2) / cellSize);

                if (x_grid >= 0 && x_grid < gridSize && y_grid >= 0 && y_grid < gridSize)
                    occupancyGrid[y_grid][x_grid] = 100;
            }
        }

        return occupancyGrid;
    }

    std::vector<std::vector<float>> CostmapCore::createCostmap(std::vector<std::vector<float>> occupancyGrid, float inflationRadius) {

        int gridSize = occupancyGrid.size();

        std::vector<std::vector<float>> costmap(gridSize, std::vector<float>(gridSize, 0));

        for (int y = 0; y < gridSize; ++y) {
            for (int x = 0; x < gridSize; ++x) {
                // Only apply inflation around non-zero cost (obstacle) cells
                if (occupancyGrid[y][x] > 0) {
                    for (int dy = -inflationRadius; dy <= inflationRadius; ++dy) {
                        for (int dx = -inflationRadius; dx <= inflationRadius; ++dx) {
                            int newY = y + dy;
                            int newX = x + dx;

                            if (newY >= 0 && newY < gridSize && newX >= 0 && newX < gridSize) {
                                float distance = sqrt(dy * dy + dx * dx);

                                if (distance <= inflationRadius) {
                                    float decayFactor = 1.0 - (distance / inflationRadius);
                                    costmap[newY][newX] = std::max(costmap[newY][newX], occupancyGrid[y][x] * decayFactor);
                                }
                            }
                        }
                    }
                }
            }
        }

        return costmap;
    }
}