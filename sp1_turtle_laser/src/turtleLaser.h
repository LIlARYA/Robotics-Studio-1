#ifndef TURTLELASER_H
#define TURTLELASER_H

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/float64.hpp"

class TurtleLaser : public rclcpp::Node
{
public:
    TurtleLaser();
    void remapLaserReading(const sensor_msgs::msg::LaserScan::SharedPtr msg);   // parameter is like const nav

private:
    // rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_pub_;

    // rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_data_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_nth_;
};

#endif // TURTLELASER_H
