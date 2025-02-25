#pragma once

#include <vector>
#include <array>
#include <rclcpp/rclcpp.hpp>

// 3次元ポイント構造体
struct Point3D
{
  float x;
  float y;
  float z;
};

struct LaserPoint
{
  double x = 0.0;
  double y = 0.0;
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

// クラスタトラック情報
struct ClusterTrack
{
  int id;
  Point3D last_centroid;
  rclcpp::Time last_update_time;
  int missing_count;
};

// パラメータ構造体
struct Parameters
{
  double min_x, max_x, min_y, max_y, min_z, max_z;
  double voxel_size_x, voxel_size_y, voxel_size_z;

  double D_voxel_size_x, D_voxel_size_y, D_voxel_size_z;
  int voxel_search_range;
  double ball_radius;

  double landmark_width, landmark_height;
  double width_tolerance, height_tolerance;
  double laser_weight, odom_weight_liner, odom_weight_angler;
  int plane_iterations, line_iterations;
  double odom2laser_x, odom2laser_y;
  double max_distance_for_association;
  int missing_count_threshold;
  double ball_vel_min;
};
