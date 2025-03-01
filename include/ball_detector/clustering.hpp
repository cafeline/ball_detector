#ifndef BALL_DETECTOR_CLUSTERING_HPP
#define BALL_DETECTOR_CLUSTERING_HPP

#include <vector>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include "ball_detector/types.hpp"
#include "ball_detector/tracking.hpp"

class ClusterCreator
{
public:
  explicit ClusterCreator(const Parameters &params);
  std::vector<ClusterInfo> create_voxel_clustering(const std::vector<Point3D> &points);
  std::vector<Voxel> create_voxel(const std::vector<Point3D> &points);

private:
  void collect_cluster_points(VoxelCluster &cluster, const std::vector<Point3D> &points);
  std::vector<std::string> get_adjacent_voxels(const std::string &key) const;
  bool point_in_voxel(const Point3D &point, const Voxel &voxel) const;
  Parameters params_;
};

class ClusterClassifier
{
public:
  explicit ClusterClassifier(const Parameters &params);
  void identify_ball_candidates(std::vector<ClusterInfo> &clusters);
  void filter_clusters_near_boundaries(std::vector<ClusterInfo> &clusters);
  void calculate_cluster_size(const VoxelCluster &cluster,
                              double &size_x, double &size_y, double &size_z) const;
  Point3D calculate_cluster_centroid(const VoxelCluster &cluster) const;

private:
  bool is_ball_size(double size_x, double size_y, double size_z) const;
  bool is_near_boundary(const Point3D &centroid) const;
  bool are_centroids_close(const Point3D &a, const Point3D &b) const;
  Parameters params_;
};

class ClusterManager
{
public:
  explicit ClusterManager(const Parameters &params);
  void process_clusters(const std::vector<Point3D> &processed_points,
                        std::vector<ClusterInfo> &clusters,
                        rclcpp::Time current_time,
                        double dt);
  void refine_ball_clusters(std::vector<ClusterInfo> &clusters,
                            const Point3D &ball_position);
  ClusterCreator cluster_creator_;

private:
  Parameters params_;
  ClusterClassifier cluster_classifier_;
  std::unique_ptr<TrackingManager> tracking_manager_;
};

// ユーティリティ関数
std::string voxel_to_key(int x, int y, int z);
std::string point_to_voxel_key(const Point3D &point, const Parameters &params);

class Clustering
{
public:
  explicit Clustering(const Parameters &params);
  std::unique_ptr<TrackingManager> tracking_manager_;
  ClusterManager cluster_manager_;

private:
  Parameters params_;
};

#endif // BALL_DETECTOR_CLUSTERING_HPP