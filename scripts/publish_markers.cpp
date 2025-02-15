#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <Eigen/Dense>
#include <vector>
#include <mutex>

class SphereVisualizer : public rclcpp::Node {
public:
    SphereVisualizer()
        : Node("sphere_visualizer") {
        publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);
    }

    // Function to update points and publish immediately
    void updatePoints(const std::vector<Eigen::Vector3d> &new_points) {
        std::lock_guard<std::mutex> lock(mutex_);
        points_ = new_points;
        publishMarkers();
    }

private:
    void publishMarkers() {
        auto marker = visualization_msgs::msg::Marker();
        marker.header.frame_id = "map";  
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "spheres";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;  
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.scale.x = 0.1;  // Sphere diameter
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;

        marker.color.a = 1.0;  // Fully visible
        marker.color.r = 0.0;
        marker.color.g = 1.0;  // Green
        marker.color.b = 0.0;

        for (const auto &point : points_) {
            geometry_msgs::msg::Point p;
            p.x = point.x();
            p.y = point.y();
            p.z = point.z();
            marker.points.push_back(p);
        }

        publisher_->publish(marker);
        RCLCPP_INFO(this->get_logger(), "Published marker with %zu points", points_.size());
    }

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;
    std::vector<Eigen::Vector3d> points_;
    std::mutex mutex_;
};


