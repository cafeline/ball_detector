#pragma once

#include <vector>
#include <string>
#include <queue>
#include <map>
#include <unordered_set>
#include "pointcloud_processor/types.hpp"

class VoxelProcessor
{
public:
  VoxelProcessor(const Parameters &params);

  // ボクセルを作成する関数
  std::vector<Voxel> create_voxel(const std::vector<Point3D> &points);

private:
  Parameters params_;
};

class Clustering
{
public:
  Clustering(const Parameters &params);

  std::vector<VoxelCluster> create_voxel_clustering(const std::vector<Point3D> &points, const std::vector<Voxel> &voxels);

  void collect_cluster_points(VoxelCluster &cluster, const std::vector<Point3D> &points);
  void calculate_cluster_size(const VoxelCluster &cluster, const std::vector<Point3D> &points, double &size_x, double &size_y, double &size_z) const;

  std::vector<VoxelCluster> extract_ball_clusters(const std::vector<VoxelCluster> &clusters, const std::vector<Point3D> &points);
  void identify_dynamic_clusters(const std::vector<VoxelCluster> &clusters, const std::vector<VoxelCluster> &dynamic_clusters);

  std::vector<int> associate_clusters(const std::vector<VoxelCluster> &current_clusters,
                                      std::map<int, ClusterTrack> &tracks,
                                      double max_distance_for_association,
                                      rclcpp::Time current_time,
                                      double dt);

  std::vector<VoxelCluster> filter_by_speed(const std::vector<VoxelCluster> &current_clusters,
                                            const std::vector<int> &assignments,
                                            std::map<int, ClusterTrack> &tracks,
                                            double dt,
                                            double speed_threshold);

  bool is_ball_size(double size_x, double size_y, double size_z) const;
  Point3D calculate_cluster_centroid(const VoxelCluster &cluster);
  bool are_centroids_close(const Point3D &a, const Point3D &b) const;

  const std::unordered_set<size_t> &get_ball_size_cluster_indices() const { return ball_size_cluster_indices_; }
  const std::unordered_set<size_t> &get_dynamic_cluster_indices() const { return dynamic_cluster_indices_; }
  size_t get_dynamic_ball_cluster_original_index(size_t j) const { return dynamic_ball_cluster_indices_[j]; }

private:
  Parameters params_;

  std::vector<std::string> get_adjacent_voxels(const std::string &key) const;
  bool is_valid_cluster(const VoxelCluster &cluster, const std::vector<Point3D> &points) const;
  bool point_in_voxel(const Point3D &point, const Voxel &voxel) const;

  std::unordered_set<size_t> ball_size_cluster_indices_;
  std::unordered_set<size_t> dynamic_cluster_indices_;
  std::vector<size_t> dynamic_ball_cluster_indices_;
};
