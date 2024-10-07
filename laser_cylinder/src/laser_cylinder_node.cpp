#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>

class SingleCylinderDetector : public rclcpp::Node {
public:
    SingleCylinderDetector() : Node("single_cylinder_detector_node") {
        pub_marker_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "/single_cylinder_marker", 10
        );
        sub_scan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&SingleCylinderDetector::laserScanCallback, this, std::placeholders::_1)
        );
        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&SingleCylinderDetector::odometryCallback, this, std::placeholders::_1)
        );
    }

private:
    sensor_msgs::msg::LaserScan latest_laser_scan_;
    nav_msgs::msg::Odometry current_odometry_;

    void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        latest_laser_scan_ = *msg;
        geometry_msgs::msg::Point detected_cylinder;

        if (findCylinder(detected_cylinder)) {
            publishCylinderMarker(detected_cylinder);
            RCLCPP_INFO(this->get_logger(), "Detected cylinder at");
        }
    }

    bool findCylinder(geometry_msgs::msg::Point& cylinder) {
        cylinder = geometry_msgs::msg::Point(); // Reset the point

        const double proximity_threshold = 1.0;
        const double desired_diameter = 0.30; // 30 cm cylinder
        const double allowable_tolerance = 0.05; // Tolerance for diameter matching

        std::vector<geometry_msgs::msg::Point> current_segment;
        
        // Analyze laser scan points for cylinder detection
        for (size_t i = 0; i < latest_laser_scan_.ranges.size(); i++) {
            double range_value = latest_laser_scan_.ranges[i];

            if (std::isfinite(range_value) && range_value < latest_laser_scan_.range_max) {
                geometry_msgs::msg::Point laser_point = convertToCartesian(i);

                if (current_segment.empty()) {
                    current_segment.push_back(laser_point);
                } else {
                    geometry_msgs::msg::Point previous_point = current_segment.back();
                    double distance_between_points = std::hypot(laser_point.x - previous_point.x, laser_point.y - previous_point.y);

                    if (distance_between_points < proximity_threshold) {
                        current_segment.push_back(laser_point);
                    } else {
                        if (current_segment.size() > 1 && isValidCylinderSegment(current_segment, desired_diameter, allowable_tolerance)) {
                            cylinder = calculateCenterPoint(current_segment);
                            return true; // Found a cylinder
                        }
                        current_segment.clear();
                        current_segment.push_back(laser_point);
                    }
                }
            }
        }

        // Check the last segment
        if (current_segment.size() > 1 && isValidCylinderSegment(current_segment, desired_diameter, allowable_tolerance)) {
            cylinder = calculateCenterPoint(current_segment);
            return true; // Found a cylinder
        }

        return false; // No cylinder found
    }

    bool isValidCylinderSegment(const std::vector<geometry_msgs::msg::Point>& segment, double desired_diameter, double allowable_tolerance) {
        geometry_msgs::msg::Point start = segment.front();
        geometry_msgs::msg::Point end = segment.back();
        double segment_length = std::hypot(end.x - start.x, end.y - start.y);
        return std::abs(segment_length - desired_diameter) <= allowable_tolerance;
    }

    geometry_msgs::msg::Point calculateCenterPoint(const std::vector<geometry_msgs::msg::Point>& segment) {
        geometry_msgs::msg::Point center;
        center.x = (segment.front().x + segment.back().x) / 2.0;
        center.y = (segment.front().y + segment.back().y) / 2.0;
        return center;
    }

    geometry_msgs::msg::Point convertToCartesian(size_t index) {
        float angle = latest_laser_scan_.angle_min + latest_laser_scan_.angle_increment * index;
        float range = latest_laser_scan_.ranges[index];

        geometry_msgs::msg::Point cartesian_point;
        cartesian_point.x = static_cast<double>(range * cos(angle));
        cartesian_point.y = static_cast<double>(range * sin(angle));
        return cartesian_point;
    }

    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_odometry_ = *msg;
    }

    void publishCylinderMarker(const geometry_msgs::msg::Point& local_point) {
        geometry_msgs::msg::Point world_point;
        double robot_x = current_odometry_.pose.pose.position.x;
        double robot_y = current_odometry_.pose.pose.position.y;

        tf2::Quaternion robot_orientation(
            current_odometry_.pose.pose.orientation.x,
            current_odometry_.pose.pose.orientation.y,
            current_odometry_.pose.pose.orientation.z,
            current_odometry_.pose.pose.orientation.w
        );
        
        tf2::Matrix3x3 rotation_matrix(robot_orientation);
        double roll, pitch, yaw;
        rotation_matrix.getRPY(roll, pitch, yaw);

        world_point.x = robot_x + local_point.x * cos(yaw) - local_point.y * sin(yaw);
        world_point.y = robot_y + local_point.x * sin(yaw) + local_point.y * cos(yaw);
        world_point.z = 0.0; // Ground level

        visualization_msgs::msg::Marker marker;

        marker.header.frame_id = "map"; // Marker frame
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "cylinder"; 
        marker.id = 0; 

        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position = world_point; // Use the calculated world coordinates

        marker.pose.orientation.w = 1.0; // No rotation

        marker.scale.x = 0.3; // Diameter
        marker.scale.y = 0.3; // Diameter
        marker.scale.z = 0.5; // Height

        marker.color.r = 0.0;
        marker.color.g = 0.2;
        marker.color.b = 1.0; // Blue
        marker.color.a = 1.0; // Opaque

        pub_marker_->publish(marker);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_marker_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SingleCylinderDetector>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
