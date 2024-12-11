#include "obstacle_avoidance.h"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    double target_distance;
    std::cout << "Enter the target distance (in meters): ";
    std::cin >> target_distance;

    if (target_distance <= 0) {
        std::cerr << "Error: Target distance must be greater than 0." << std::endl;
        return 1;
    }

    auto node = std::make_shared<ObstacleAvoidance>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}