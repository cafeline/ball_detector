#ifndef CLUSTERING_HPP
#define CLUSTERING_HPP

#include "ball_detector/types.hpp"
#include "ball_detector/cluster_tracking.hpp"
#include <vector>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include "visualization_msgs/msg/marker_array.hpp"

// ユーティリティ関数のプロトタイプ
std::string voxel_to_key(int x, int y, int z);
std::string point_to_voxel_key(const Point3D &point, const Parameters &params);

class Clustering
{
public:
  Clustering(const Parameters &params);

  // ポイントクラウドからのクラスタリング処理関連
  std::vector<ClusterInfo> create_voxel_clustering(const std::vector<Point3D> &points,
                                                   const std::vector<Voxel> &voxels);
  void collect_cluster_points(VoxelCluster &cluster, const std::vector<Point3D> &points);

  // 各クラスタ内部の点群からサイズ計算を行う（引数から points は不要）
  void calculate_cluster_size(const VoxelCluster &cluster,
                              double &size_x,
                              double &size_y,
                              double &size_z) const;

  // クラスタ処理
  void process_clusters(const std::vector<Point3D> &processed_points,
                        std::vector<ClusterInfo> &clusters,
                        rclcpp::Time current_time,
                        double dt);
  void refine_ball_clusters(std::vector<ClusterInfo> &clusters, const Point3D &ball_position);

  // 動的クラスタ識別
  std::vector<ClusterInfo> identify_dynamic_clusters(const std::vector<ClusterInfo> &clusters,
                                                     const rclcpp::Time &current_time,
                                                     double dt);

  // 各クラスタにボール／動的クラスタのフラグをセットする関数（points 引数は不要）
  void mark_ball_clusters(std::vector<ClusterInfo> &clusters);
  void mark_dynamic_clusters(std::vector<ClusterInfo> &clusters, const std::vector<ClusterInfo> &dynamic_clusters);
  void filter_dynamic_ball_clusters_near_boundaries(std::vector<ClusterInfo> &clusters);

  // ボクセル操作ユーティリティ
  bool point_in_voxel(const Point3D &point, const Voxel &voxel) const;

private:
  Parameters params_;
  ClusterTracking cluster_tracking_;

  // 位置・距離計算
  bool is_near_boundary(const Point3D &centroid) const;
  bool is_zero_position(const Point3D &position) const;
  double calculate_distance(const Point3D &a, const Point3D &b) const;

  // ボールクラスタ処理
  bool is_ball_size(double size_x, double size_y, double size_z) const;
  size_t find_closest_ball_cluster(const std::vector<ClusterInfo> &clusters,
                                   const Point3D &ball_position) const;
  void update_ball_cluster(std::vector<ClusterInfo> &clusters,
                           size_t best_idx,
                           const Point3D &ball_position);

  // 中心点計算
  Point3D calculate_cluster_centroid(const VoxelCluster &cluster) const;
  size_t find_closest_centroid(const std::vector<Point3D> &centroids,
                               const Point3D &target) const;
  bool are_centroids_close(const Point3D &a, const Point3D &b) const;

  // ボクセル隣接性
  std::vector<std::string> get_adjacent_voxels(const std::string &key) const;
};

#endif // CLUSTERING_HPP
