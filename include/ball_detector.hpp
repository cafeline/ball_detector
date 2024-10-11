#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <vector>
#include <cstring>
#include <visualization_msgs/msg/marker.hpp>


struct Point3D {
    float x;
    float y;
    float z;
};

class BallDetector : public rclcpp::Node
{
public:
  BallDetector();

private:
  void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  std::vector<Point3D> PC2_to_vector(const sensor_msgs::msg::PointCloud2& cloud_msg);
  sensor_msgs::msg::PointCloud2 vector_to_PC2(const std::vector<Point3D>& points);
  void filter_points(const std::vector<Point3D>& input, std::vector<Point3D>& output);
  Point3D calculate_centroid(const std::vector<Point3D>& points);
  visualization_msgs::msg::Marker create_ball_marker(const Point3D& centroid, const std_msgs::msg::Header& header);
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_cloud_publisher_;
};
