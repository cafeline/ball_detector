#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <vector>
#include <cstring>
#include <visualization_msgs/msg/marker.hpp>
#include <array>
#include <visualization_msgs/msg/marker_array.hpp>

// 3次元ポイント構造体
struct Point3D
{
  float x;
  float y;
  float z;
};

// ボクセル構造体
struct Voxel
{
  int x, y, z;                 // ボクセルのインデックス
  int point_count;             // ボクセル内の点群数
  std::vector<Point3D> points; // ボクセル内の点群データ
  std::array<float, 4> color;  // ボクセルの色 (RGBA)

  // デフォルトコンストラクタ
  Voxel() : x(0), y(0), z(0), point_count(0), color{1.0f, 1.0f, 1.0f, 1.0f} {}

  // パラメータ付きコンストラクタ
  Voxel(int vx, int vy, int vz) : x(vx), y(vy), z(vz), point_count(1), color{1.0f, 1.0f, 1.0f, 1.0f} {}

  // 点群数をインクリメント
  void increment()
  {
    point_count++;
  }

  // 点を追加
  void add_point(const Point3D &point)
  {
    points.push_back(point);
    increment();
  }
};

// ボクセルクラスタ構造体
struct VoxelCluster
{
  std::vector<Voxel> voxels;          // クラスタ内のボクセル
  int total_point_count;              // クラスタ内の総点群数
  std::vector<Point3D> points;        // クラスタ内の全点群データ
  std::array<float, 4> cluster_color; // クラスタの色 (RGBA)

  // デフォルトコンストラクタ
  VoxelCluster() : total_point_count(0), cluster_color{1.0f, 0.0f, 0.0f, 1.0f} {}
};

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
  std::vector<Voxel> create_voxel(const std::vector<Point3D> &points);
  std::vector<VoxelCluster> create_voxel_clustering(const std::vector<Point3D> &points);
  visualization_msgs::msg::MarkerArray create_voxel_markers(const std::vector<Voxel> &voxels, const std_msgs::msg::Header &header);
  visualization_msgs::msg::MarkerArray create_voxel_cluster_markers(const std::vector<VoxelCluster> &clusters);
  visualization_msgs::msg::Marker create_detection_area_marker(const std_msgs::msg::Header &header);
  std::vector<std::string> get_adjacent_voxels(const std::string &key) const;
  bool is_valid_cluster(const VoxelCluster &cluster, const std::vector<Point3D> &points) const;
  void calculate_cluster_size(const VoxelCluster &cluster, const std::vector<Point3D> &points, double &size_x, double &size_y, double &size_z) const;
  bool point_in_voxel(const Point3D &point, const Voxel &voxel) const;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr clustered_voxel_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr ball_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_cloud_publisher_;

  std::string frame_id_ = "map";

  struct Parameters
  {
    double min_x, max_x, min_y, max_y, min_z, max_z;
    double voxel_size_x, voxel_size_y, voxel_size_z;
  };
  Parameters params_;

  std::vector<Point3D> remove_clustered_points(const std::vector<Point3D> &original_points, const std::vector<VoxelCluster> &clusters);

  // クラスタリングされた点群を保持するメンバー変数
  std::vector<Point3D> clustered_points_;

  // 追加: collect_cluster_points 関数の宣言
  void collect_cluster_points(VoxelCluster &cluster, const std::vector<Point3D> &points);
};
