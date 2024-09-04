#include "moveTurtle.h"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#define DEBUG 1
#define ROS_INFO RCUTILS_LOG_INFO
#define ROS_DEBUG RCUTILS_LOG_DEBUG

using std::cout;
using std::endl;
using std::placeholders::_1;
using namespace std::chrono_literals;

moveTurtle::moveTurtle() : Node("move_turtle")
{
    laser_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&moveTurtle::remapLaserReading, this, std::placeholders::_1));

    scan_pub_nth_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan_remap_nth", 10);
}



void moveTurtle::remapLaserReading(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{   
    // Creating a new LaserScan message, copying the contents of the recieved scan message
    auto nth_scan = std::make_shared<sensor_msgs::msg::LaserScan>(*scan);
    // Clear the ranges vector of the nth_scan
    nth_scan->ranges.clear();

    // Loop to extract nth range reading
    for (size_t i = 0; i < scan->ranges.size(); i += 10)
    {   
        // only every second range reading is pushed back (i +=2)
        nth_scan->ranges.push_back(scan->ranges[i]);
    }

    // Adjusting the angle increment of the nth_scan, as readings as spaced twice as far
    nth_scan->angle_increment *= 10;

    // Publishing the modified nth_scan
    scan_pub_nth_->publish(*nth_scan);
}



int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<moveTurtle>());
    rclcpp::shutdown();
    return 0;
}

// ----------