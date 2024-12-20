#include "obstacle_avoidance.h"
#include <cmath>
#include <iostream>

ObstacleAvoidance::ObstacleAvoidance(const rclcpp::NodeOptions &options)
    : Node("obstacle_avoidance_node", options), initial_position_x_(0.0), obstacle_detected_l_(false), obstacle_detected_r_(false), goal_reached_(false) {
    // Declare and initialize parameters
    this->declare_parameter<double>("target_distance", 1.0);
    this->declare_parameter<double>("object_distance_threshold", 0.25);
    this->declare_parameter<std::string>("lidar_topic", "/scan");
    this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    this->declare_parameter<std::string>("odometry_topic", "/odometry/filtered");

    // Retrieve parameters
    target_distance_ = this->get_parameter("target_distance").as_double();
    object_distance_threshold_ = this->get_parameter("object_distance_threshold").as_double();
    lidar_topic_ = this->get_parameter("lidar_topic").as_string();
    cmd_vel_topic_ = this->get_parameter("cmd_vel_topic").as_string();
    odometry_topic_ = this->get_parameter("odometry_topic").as_string();

    // Subscribers and publishers
    lidar_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        lidar_topic_, 10, std::bind(&ObstacleAvoidance::lidarCallback, this, std::placeholders::_1));

    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odometry_topic_, 10, std::bind(&ObstacleAvoidance::odomCallback, this, std::placeholders::_1));

    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);

    RCLCPP_INFO(this->get_logger(), "Obstacle Avoidance Node Initialized");
}

double ObstacleAvoidance::toRadians(double angle) {
    return angle * M_PI / 180.0;
}

void ObstacleAvoidance::lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    int num_ranges = msg->ranges.size();
    double angle_min = msg->angle_min;
    double angle_increment = msg->angle_increment;

    double front_r_start_angle = 135.0, front_r_end_angle = 180.0;
    double front_l_start_angle = -180.0, front_l_end_angle = -135.0;

    int start_r_index = std::max(0, static_cast<int>((toRadians(front_r_start_angle) - angle_min) / angle_increment));
    int end_r_index = std::min(num_ranges - 1, static_cast<int>((toRadians(front_r_end_angle) - angle_min) / angle_increment));
    int start_l_index = std::max(0, static_cast<int>((toRadians(front_l_start_angle) - angle_min) / angle_increment));
    int end_l_index = std::min(num_ranges - 1, static_cast<int>((toRadians(front_l_end_angle) - angle_min) / angle_increment));

    obstacle_detected_r_ = obstacle_detected_l_ = false;

    for (int i = start_r_index; i <= end_r_index; ++i) {
        if (msg->ranges[i] < object_distance_threshold_) {
            obstacle_detected_r_ = true;
            break;
        }
    }

    for (int i = start_l_index; i <= end_l_index; ++i) {
        if (msg->ranges[i] < object_distance_threshold_) {
            obstacle_detected_l_ = true;
            break;
        }
    }
}

void ObstacleAvoidance::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    double current_position_x = msg->pose.pose.position.x;

    if (initial_position_x_ == 0.0) {
        initial_position_x_ = current_position_x;
    }

    double distance_traveled = std::abs(current_position_x - initial_position_x_);

    if (distance_traveled >= target_distance_ && !goal_reached_) {
        stopRobot();
        goal_reached_ = true;
        RCLCPP_INFO(this->get_logger(), "Goal reached! Stopping the robot.");
    } else if (!goal_reached_) {
        moveRobot();
        RCLCPP_INFO(this->get_logger(), "Distance traveled: %.2f meters", distance_traveled);
    }
}

void ObstacleAvoidance::moveRobot() {
    geometry_msgs::msg::Twist cmd_msg;

    if (obstacle_detected_r_ && obstacle_detected_l_) {
        cmd_msg.linear.y = 0.2;
        cmd_msg.linear.x = 0.0;
        RCLCPP_INFO(this->get_logger(), "Obstacle detected in front and shafting left");
    } else if (obstacle_detected_r_) {
        cmd_msg.linear.y = 0.2;
        cmd_msg.linear.x = 0.0;
        RCLCPP_INFO(this->get_logger(), "Obstacle detected on the right and shafting left");
    } else if (obstacle_detected_l_) {
        cmd_msg.linear.y = -0.2;
        cmd_msg.linear.x = 0.0;
        RCLCPP_INFO(this->get_logger(), "Obstacle detected on the lfet and shafting right");
    } else {
        cmd_msg.linear.y = 0.0;
        cmd_msg.linear.x = 0.2;
    }

    cmd_vel_publisher_->publish(cmd_msg);
}

void ObstacleAvoidance::stopRobot() {
    geometry_msgs::msg::Twist cmd_msg;
    cmd_msg.linear.x = 0.0;
    cmd_msg.angular.z = 0.0;
    cmd_vel_publisher_->publish(cmd_msg);
}

bool ObstacleAvoidance::GoalReached() {
    return goal_reached_;
}