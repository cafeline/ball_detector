#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <vector>
#include <cstring>
#include <visualization_msgs/msg/marker.hpp>
#include <array>

struct Point3D {
    float x;
    float y;
    float z;
};

struct Region {
    double min_x, max_x;
    double min_y, max_y;
    double min_z, max_z;
    std::vector<Point3D> points;
    std::array<float, 4> color;
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
  visualization_msgs::msg::Marker create_bounding_box_marker(const std::vector<Point3D>& points, const std_msgs::msg::Header& header);
  visualization_msgs::msg::Marker create_bounding_box_marker(const std_msgs::msg::Header& header);
  void timer_callback();
  std::vector<Point3D> voxel_clustering(const std::vector<Point3D>& points, const Point3D& centroid);
  
  // 新しく追加する関数
  std::vector<Region> extract_regions(const std::vector<Point3D>& points, size_t max_regions);
  visualization_msgs::msg::Marker create_region_bounding_box_marker(const Region& region, const std_msgs::msg::Header& header, int id);
  sensor_msgs::msg::PointCloud2 region_to_PC2(const Region& region);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_cloud_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr bounding_box_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string frame_id_ = "map";

  // 新しく追加するパブリッシャー
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr region_cloud_publisher_;
};