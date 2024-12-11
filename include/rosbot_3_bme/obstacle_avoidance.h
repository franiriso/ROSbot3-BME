#ifndef OBSTACLE_AVOIDANCE_HPP
#define OBSTACLE_AVOIDANCE_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <string>

class ObstacleAvoidance : public rclcpp::Node {
public:
    ObstacleAvoidance(const rclcpp::NodeOptions &options);
    bool GoalReached();

private:
    void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void moveRobot();
    void stopRobot();
    double toRadians(double angle);

    // Parameters
    double target_distance_;
    double object_distance_threshold_;
    std::string lidar_topic_;
    std::string cmd_vel_topic_;
    std::string odometry_topic_;

    // State variables
    double initial_position_x_;
    bool obstacle_detected_l_;
    bool obstacle_detected_r_;
    bool goal_reached_;

    // ROS 2 subscribers and publisher
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
};

#endif // OBSTACLE_AVOIDANCE_HPP