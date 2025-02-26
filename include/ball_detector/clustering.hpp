#pragma once

#include <vector>
#include <string>
#include <unordered_set>
#include "ball_detector/pointcloud_processor.hpp"
#include "ball_detector/cluster_tracking.hpp"
#include <rclcpp/rclcpp.hpp>

class Clustering
{
public:
  Clustering(const Parameters &params);

  void process_clusters(const std::vector<Point3D> &processed_points,
                        const std::vector<VoxelCluster> &clusters,
                        rclcpp::Time current_time,
                        double dt);
  std::vector<VoxelCluster> create_voxel_clustering(const std::vector<Point3D> &points,
                                                    const std::vector<Voxel> &voxels);

  // 各種クラスタ処理
  void collect_cluster_points(VoxelCluster &cluster, const std::vector<Point3D> &points);
  void calculate_cluster_size(const VoxelCluster &cluster,
                              const std::vector<Point3D> &points,
                              double &size_x,
                              double &size_y,
                              double &size_z) const;
  std::vector<VoxelCluster> identify_dynamic_clusters(const std::vector<VoxelCluster> &clusters,
                                                      const rclcpp::Time &current_time,
                                                      double dt);

  // ボールクラスタに関する処理
  void calc_ball_clusters_indices(const std::vector<VoxelCluster> &clusters,
                                  const std::vector<Point3D> &points);
  std::vector<VoxelCluster> extract_ball_clusters(const std::vector<VoxelCluster> &clusters);
  std::vector<VoxelCluster> extract_dynamic_ball_clusters(const std::vector<VoxelCluster> &clusters);
  void calc_dynamic_ball_cluster_indices(const std::vector<VoxelCluster> &clusters);
  void calc_dynamic_cluster_indices(const std::vector<VoxelCluster> &clusters,
                                    const std::vector<VoxelCluster> &dynamic_clusters);

  bool is_ball_size(double size_x, double size_y, double size_z) const;
  Point3D calculate_cluster_centroid(const VoxelCluster &cluster);
  bool are_centroids_close(const Point3D &a, const Point3D &b) const;
  void filter_dynamic_ball_clusters_near_boundaries(const std::vector<VoxelCluster> &clusters);
  void refine_ball_clusters(std::vector<VoxelCluster> &clusters, const Point3D &ball_position);

  const std::unordered_set<size_t> &get_ball_size_cluster_indices() const { return ball_size_cluster_indices_; }
  const std::unordered_set<size_t> &get_dynamic_cluster_indices() const { return dynamic_cluster_indices_; }
  const std::unordered_set<size_t> &get_dynamic_ball_cluster_indices() const { return dynamic_ball_cluster_indices_; }

private:
  Parameters params_;
  ClusterTracking cluster_tracking_;
  // 内部ヘルパー関数
  std::vector<std::string> get_adjacent_voxels(const std::string &key) const;
  bool point_in_voxel(const Point3D &point, const Voxel &voxel) const;

  std::unordered_set<size_t> ball_size_cluster_indices_;
  std::unordered_set<size_t> dynamic_cluster_indices_;
  std::unordered_set<size_t> dynamic_ball_cluster_indices_;
};