#ifndef BALL_DETECTOR_CLUSTERING_HPP
#define BALL_DETECTOR_CLUSTERING_HPP

#include <vector>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include "ball_detector/types.hpp"
#include "ball_detector/cluster_tracking.hpp"

class Clustering
{
public:
  explicit Clustering(const Parameters &params);

  // クラスタリング処理のメインメソッド
  std::vector<ClusterInfo> create_voxel_clustering(const std::vector<Point3D> &points,
                                                   const std::vector<Voxel> &voxels);

  // クラスタの処理と分類
  void process_clusters(const std::vector<Point3D> &processed_points,
                        std::vector<ClusterInfo> &clusters,
                        rclcpp::Time current_time,
                        double dt);

  // ボールクラスタの最適化
  void refine_ball_clusters(std::vector<ClusterInfo> &clusters, const Point3D &ball_position);

private:
  // クラスタの識別と分類
  void identify_ball_candidates(std::vector<ClusterInfo> &clusters);
  void identify_dynamic_clusters(std::vector<ClusterInfo> &clusters,
                                 rclcpp::Time current_time,
                                 double dt);
  void mark_dynamic_clusters(std::vector<ClusterInfo> &clusters,
                             const std::vector<VoxelCluster> &dynamic_clusters);
  void filter_clusters_near_boundaries(std::vector<ClusterInfo> &clusters);

  // クラスタ操作ヘルパー関数
  void collect_cluster_points(VoxelCluster &cluster, const std::vector<Point3D> &points);
  void calculate_cluster_size(const VoxelCluster &cluster,
                              double &size_x,
                              double &size_y,
                              double &size_z) const;
  Point3D calculate_cluster_centroid(const VoxelCluster &cluster) const;
  double calculate_distance(const Point3D &a, const Point3D &b) const;

  // ボールクラスタ処理ヘルパー
  bool is_zero_position(const Point3D &position) const;
  size_t find_closest_ball_cluster(const std::vector<ClusterInfo> &clusters,
                                   const Point3D &ball_position) const;
  void update_ball_cluster(std::vector<ClusterInfo> &clusters,
                           size_t best_idx,
                           const Point3D &ball_position);

  // クラスタマッチングとフィルタリング
  bool clusters_match(const VoxelCluster &a, const VoxelCluster &b) const;
  bool are_centroids_close(const Point3D &a, const Point3D &b) const;
  bool is_ball_size(double size_x, double size_y, double size_z) const;
  bool is_near_boundary(const Point3D &centroid) const;

  // ヴォクセル関連ヘルパー関数
  std::vector<std::string> get_adjacent_voxels(const std::string &key) const;
  bool point_in_voxel(const Point3D &point, const Voxel &voxel) const;

  // パラメータと追跡情報
  Parameters params_;
  ClusterTracking cluster_tracking_;
};

// ユーティリティ関数
std::string voxel_to_key(int x, int y, int z);
std::string point_to_voxel_key(const Point3D &point, const Parameters &params);

#endif // BALL_DETECTOR_CLUSTERING_HPP