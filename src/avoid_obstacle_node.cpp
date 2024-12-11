#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <cmath>

class ObstacleAvoidance : public rclcpp::Node {
public:
    ObstacleAvoidance() : Node("obstacle_avoidance_node") {
        // Declare and initialize parameters
        this->declare_parameter<double>("target_distance", 1.0);  // Default: 1 meter
        this->declare_parameter<double>("object_distance_threshold", 0.25);  // Default: 0.1 meters
        this->declare_parameter<std::string>("lidar_topic", "/scan");
        this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
        this->declare_parameter<std::string>("odometry_topic", "/odometry/filtered");

        target_distance_ = this->get_parameter("target_distance").as_double();
        object_distance_threshold_ = this->get_parameter("object_distance_threshold").as_double();
        lidar_topic_ = this->get_parameter("lidar_topic").as_string();
        cmd_vel_topic_ = this->get_parameter("cmd_vel_topic").as_string();
        odometry_topic_ = this->get_parameter("odometry_topic").as_string();

        // Initialize variables
        initial_position_x_ = 0.0;
        goal_reached_ = false;

        // Subscribers and publishers
        lidar_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            lidar_topic_, 10,
            std::bind(&ObstacleAvoidance::lidarCallback, this, std::placeholders::_1));

        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odometry_topic_, 10,
            std::bind(&ObstacleAvoidance::odomCallback, this, std::placeholders::_1));

        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);

        RCLCPP_INFO(this->get_logger(), "Obstacle Avoidance Node Initialized");
    }

private:
    double front_r_start_angle = 135.00;
    double front_r_end_angle = 180.00;
    double front_l_start_angle = -180.00;
    double front_l_end_angle = -135.00;

    double toRadians(double angle) {
        return angle * M_PI / 180.0;
    }
    void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        // Check for obstacles in the front range
        int num_ranges = msg->ranges.size();
        double angle_min = msg->angle_min;
        double angle_increment = msg->angle_increment;

        // Define front range ±10 degrees (in radians)
        double front_r_start_angle_rad = this->toRadians(front_r_start_angle);
        double front_r_end_angle_rad = this->toRadians(front_r_end_angle);
        double front_l_start_angle_rad = this->toRadians(front_l_start_angle);
        double front_rl_end_angle_rad = this->toRadians(front_l_end_angle);

        int start_r_index = std::max(0, static_cast<int>((front_r_start_angle_rad - angle_min) / angle_increment));
        int end_r_index = std::min(num_ranges - 1, static_cast<int>((front_r_end_angle_rad - angle_min) / angle_increment));
        int start_l_index = std::max(0, static_cast<int>((front_l_start_angle_rad - angle_min) / angle_increment));
        int end_l_index = std::min(num_ranges - 1, static_cast<int>((front_rl_end_angle_rad - angle_min) / angle_increment));
        // Reset flags before processing
        obstacle_detected_r_ = false;
        obstacle_detected_l_ = false;

        // Check for obstacles in the right range
        for (int i = start_r_index; i <= end_r_index; ++i) {
            if (msg->ranges[i] < object_distance_threshold_) {
                obstacle_detected_r_ = true;
                break; // Stop checking the right range as we already found an obstacle
            }
        }

        // Check for obstacles in the left range
        for (int i = start_l_index; i <= end_l_index; ++i) {
            if (msg->ranges[i] < object_distance_threshold_) {
                obstacle_detected_l_ = true;
                break; // Stop checking the left range as we already found an obstacle
            }
        }
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // Get the robot's current x position
        double current_position_x = msg->pose.pose.position.x;

        // Record the initial position if not already done
        if (initial_position_x_ == 0.0) {
            initial_position_x_ = current_position_x;
        }

        // Calculate distance traveled
        double distance_traveled = std::abs(current_position_x - initial_position_x_);

        // If the target distance is reached, stop the robot
        if (distance_traveled >= target_distance_) {
            stopRobot();
            goal_reached_ = true;
            RCLCPP_INFO(this->get_logger(), "Goal reached! Stopping the robot.");
        } else if (!goal_reached_) {
            moveRobot();
        }
    }

    void moveRobot() {
        geometry_msgs::msg::Twist cmd_msg;

        if (obstacle_detected_r_ && obstacle_detected_l_) {
            // Shift to the right to avoid obstacle
            cmd_msg.linear.y = -0.2;  // Shaft left
            cmd_msg.linear.x = 0.0;   // Stop forward motion
        } else if (obstacle_detected_r_) {
            // Shift to the right to avoid obstacle
            cmd_msg.linear.y = -0.2;  // Shaft left
            cmd_msg.linear.x = 0.0;   // Stop forward motion
        } else if (obstacle_detected_l_) {
            // Shift to the right to avoid obstacle
            cmd_msg.linear.y = 0.2;  // Shaft right
            cmd_msg.linear.x = 0.0;   // Stop forward motion
        } else {
            // Move forward
            cmd_msg.linear.y = 0.0;  // Stop shafting
            cmd_msg.linear.x = 0.2;   // Forward velocity
        }

        cmd_vel_publisher_->publish(cmd_msg);
    }

    void stopRobot() {
        geometry_msgs::msg::Twist cmd_msg;
        cmd_msg.linear.x = 0.0;
        cmd_msg.angular.z = 0.0;
        cmd_vel_publisher_->publish(cmd_msg);
    }

    // Parameters
    double target_distance_;
    double object_distance_threshold_;
    std::string lidar_topic_;
    std::string cmd_vel_topic_;
    std::string odometry_topic_;

    // State variables
    double initial_position_x_;
    bool obstacle_detected_l_ = false;
    bool obstacle_detected_r_ = false;
    bool goal_reached_ = false;

    // ROS 2 subscribers and publisher
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    // Prompt user for target distance
    double target_distance;
    std::cout << "Enter the target distance (in meters): ";
    std::cin >> target_distance;

    // Validate input
    if (target_distance <= 0) {
        std::cerr << "Error: Target distance must be greater than 0." << std::endl;
        return 1;
    }

    // Pass the target distance to your node
    auto node = std::make_shared<ObstacleAvoidance>("target_distance");
    
    rclcpp::spin(std::make_shared<ObstacleAvoidance>());
    rclcpp::shutdown();
    return 0;
}
