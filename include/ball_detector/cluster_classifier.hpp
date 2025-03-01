#ifndef BALL_DETECTOR_CLUSTER_CLASSIFIER_HPP
#define BALL_DETECTOR_CLUSTER_CLASSIFIER_HPP

#include <vector>
#include <rclcpp/rclcpp.hpp>
#include "ball_detector/geometry_types.hpp"
#include "ball_detector/clustering_types.hpp"
#include "ball_detector/parameters.hpp"

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
  Parameters params_;
};

#endif // BALL_DETECTOR_CLUSTER_CLASSIFIER_HPP