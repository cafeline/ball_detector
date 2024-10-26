#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <vector>
#include <cstring>
#include <visualization_msgs/msg/marker.hpp>
#include <array>
#include <visualization_msgs/msg/marker_array.hpp>
#include "clustering.hpp"
#include "types.hpp"

// BallDetector クラス
class BallDetector : public rclcpp::Node
{
public:
  BallDetector();

private:
  void load_parameters();
  void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  std::vector<Point3D> PC2_to_vector(const sensor_msgs::msg::PointCloud2 &cloud_msg);
  sensor_msgs::msg::PointCloud2 vector_to_PC2(const std::vector<Point3D> &points);
  std::vector<Point3D> filter_points(const std::vector<Point3D> &input);
  Point3D calculate_centroid(const std::vector<Point3D> &points);
  Point3D calculate_cluster_centroid(const VoxelCluster &cluster);
  visualization_msgs::msg::Marker create_ball_marker(const Point3D &centroid, const std_msgs::msg::Header &header);
  visualization_msgs::msg::MarkerArray create_voxel_markers(const std::vector<Voxel> &voxels, const std_msgs::msg::Header &header);
  visualization_msgs::msg::MarkerArray create_voxel_cluster_markers(const std::vector<VoxelCluster> &clusters);
  visualization_msgs::msg::Marker create_detection_area_marker(const std_msgs::msg::Header &header);

  // 新たに追加された関数
  std::vector<Point3D> remove_clustered_points(const std::vector<Point3D> &original_points, const std::vector<VoxelCluster> &clusters);
  void collect_cluster_points(VoxelCluster &cluster, const std::vector<Point3D> &points);

  // メンバー変数
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr clustered_voxel_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr ball_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_cloud_publisher_;

  std::string frame_id_ = "map";

  Parameters params_;

  std::vector<Point3D> clustered_points_;

  // VoxelProcessor と Clustering のインスタンス
  std::unique_ptr<VoxelProcessor> voxel_processor_;
  std::unique_ptr<Clustering> clustering_;
};
