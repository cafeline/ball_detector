#ifndef CLUSTERING_HPP
#define CLUSTERING_HPP

#include "ball_detector/types.hpp"
#include "ball_detector/cluster_tracking.hpp"
#include <vector>
#include <string>
#include <unordered_set>
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
  void calculate_cluster_size(const VoxelCluster &cluster,
                              const std::vector<Point3D> &points,
                              double &size_x,
                              double &size_y,
                              double &size_z) const;

  // クラスタ処理
  void process_clusters(const std::vector<Point3D> &processed_points,
                        const std::vector<ClusterInfo> &clusters,
                        rclcpp::Time current_time,
                        double dt);
  void refine_ball_clusters(std::vector<ClusterInfo> &clusters, const Point3D &ball_position);

  // 動的クラスタ識別
  std::vector<ClusterInfo> identify_dynamic_clusters(const std::vector<ClusterInfo> &clusters,
                                                     const rclcpp::Time &current_time,
                                                     double dt);

  // クラスタインデックス計算
  void calc_ball_clusters_indices(const std::vector<ClusterInfo> &clusters,
                                  const std::vector<Point3D> &points);
  void calc_dynamic_cluster_indices(const std::vector<ClusterInfo> &clusters,
                                    const std::vector<ClusterInfo> &dynamic_clusters);
  void calc_dynamic_ball_cluster_indices(const std::vector<ClusterInfo> &clusters);
  void filter_dynamic_ball_clusters_near_boundaries(const std::vector<ClusterInfo> &clusters);

  // クラスタ抽出
  std::vector<ClusterInfo> extract_ball_clusters(const std::vector<ClusterInfo> &clusters);
  std::vector<ClusterInfo> extract_dynamic_ball_clusters(const std::vector<ClusterInfo> &clusters);
  std::vector<ClusterInfo> extract_clusters_by_indices(const std::vector<ClusterInfo> &clusters,
                                                       const std::unordered_set<size_t> &indices);

  // ゲッターメソッド
  const std::unordered_set<size_t> &get_ball_size_cluster_indices() const { return ball_size_cluster_indices_; }
  const std::unordered_set<size_t> &get_dynamic_cluster_indices() const { return dynamic_cluster_indices_; }
  const std::unordered_set<size_t> &get_dynamic_ball_cluster_indices() const { return dynamic_ball_cluster_indices_; }

  // クラスタを取得するメソッド（ClusterInfoのvectorとして管理）
  const std::vector<ClusterInfo> &get_clusters() const { return current_clusters_; }

  // ボクセル操作ユーティリティ
  bool point_in_voxel(const Point3D &point, const Voxel &voxel) const;

private:
  Parameters params_;
  ClusterTracking cluster_tracking_;

  // クラスタインデックスセット（各クラスタに付与したIDを利用）
  std::unordered_set<size_t> ball_size_cluster_indices_;
  std::unordered_set<size_t> dynamic_cluster_indices_;
  std::unordered_set<size_t> dynamic_ball_cluster_indices_;

  // セット操作ユーティリティ
  std::unordered_set<size_t> set_intersection(
      const std::unordered_set<size_t> &set1,
      const std::unordered_set<size_t> &set2);

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

  // 現在のクラスタを ClusterInfo として保持
  std::vector<ClusterInfo> current_clusters_;
};

#endif // CLUSTERING_HPP
