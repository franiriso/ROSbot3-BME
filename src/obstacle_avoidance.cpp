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

    auto options = rclcpp::NodeOptions().parameter_overrides({{"target_distance", target_distance}});
    auto node = std::make_shared<ObstacleAvoidance>(options);
    while (rclcpp::ok() && !node->GoalReached()) {
        rclcpp::spin_some(node);
    }

    rclcpp::shutdown();
    return 0;
}

/*
To test this example, execute the following lines:
colcon build --packages-select rosbot_3_bme
source install/setup.bash

ros2 run rosbot_3_bme obstacle_avoidance_node
*/