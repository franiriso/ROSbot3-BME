#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <cmath>
#include <vector>
#include <iostream>

class LidarObjectPrinter : public rclcpp::Node {
public:
    LidarObjectPrinter() : Node("lidar_object_printer") {
        // Subscribe to the LiDAR topic
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&LidarObjectPrinter::scan_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "LidarObjectPrinter initialized. Listening to LiDAR data...");
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        // Define the target angular ranges (in degrees)
        double range1_start = 135.0;
        double range1_end = 180.0;
        double range2_start = -180.0;
        double range2_end = -135.0;

        // Convert angles to radians
        double range1_start_rad = range1_start * M_PI / 180.0;
        double range1_end_rad = range1_end * M_PI / 180.0;
        double range2_start_rad = range2_start * M_PI / 180.0;
        double range2_end_rad = range2_end * M_PI / 180.0;

        // Get LiDAR data details
        int num_ranges = msg->ranges.size();
        double angle_increment = msg->angle_increment; // Angle increment per ray (radians)
        double angle_min = msg->angle_min;             // Minimum angle of LiDAR scan (radians)

        // Calculate indices for the first range (135° to 180°)
        int range1_start_index = std::max(0, static_cast<int>((range1_start_rad - angle_min) / angle_increment));
        int range1_end_index = std::min(num_ranges - 1, static_cast<int>((range1_end_rad - angle_min) / angle_increment));

        // Calculate indices for the second range (-180° to -135°)
        int range2_start_index = std::max(0, static_cast<int>((range2_start_rad - angle_min) / angle_increment));
        int range2_end_index = std::min(num_ranges - 1, static_cast<int>((range2_end_rad - angle_min) / angle_increment));

        // Store detected objects
        std::vector<std::pair<double, double>> objects_range1;
        std::vector<std::pair<double, double>> objects_range2;

        // Process the first range (135° to 180°)
        for (int i = range1_start_index; i <= range1_end_index; ++i) {
            double distance = msg->ranges[i];
            if (distance >= msg->range_min && distance <= msg->range_max) {
                double angle = angle_min + i * angle_increment;
                objects_range1.push_back({angle, distance});
            }
        }

        // Process the second range (-180° to -135°)
        for (int i = range2_start_index; i <= range2_end_index; ++i) {
            double distance = msg->ranges[i];
            if (distance >= msg->range_min && distance <= msg->range_max) {
                double angle = angle_min + i * angle_increment;
                objects_range2.push_back({angle, distance});
            }
        }

        // Print detected objects
        if (!objects_range1.empty() || !objects_range2.empty()) {
            RCLCPP_INFO(this->get_logger(), "Objects detected in the defined angular ranges:");

            if (!objects_range1.empty()) {
                RCLCPP_INFO(this->get_logger(), "  Range 1 (135° to 180°):");
                for (const auto &obj : objects_range1) {
                    RCLCPP_INFO(this->get_logger(), "    - Angle: %.2f° | Distance: %.2f m",
                                obj.first * 180.0 / M_PI, obj.second);
                }
            }

            if (!objects_range2.empty()) {
                RCLCPP_INFO(this->get_logger(), "  Range 2 (-180° to -135°):");
                for (const auto &obj : objects_range2) {
                    RCLCPP_INFO(this->get_logger(), "    - Angle: %.2f° | Distance: %.2f m",
                                obj.first * 180.0 / M_PI, obj.second);
                }
            }
        } else {
            RCLCPP_INFO(this->get_logger(), "No objects detected in the defined angular ranges.");
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarObjectPrinter>());
    rclcpp::shutdown();
    return 0;
}
