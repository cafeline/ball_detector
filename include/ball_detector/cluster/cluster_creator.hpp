#ifndef BALL_DETECTOR_CLUSTER_CREATOR_HPP
#define BALL_DETECTOR_CLUSTER_CREATOR_HPP

#include <vector>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include "ball_detector/types/geometry_types.hpp"
#include "ball_detector/types/clustering_types.hpp"
#include "ball_detector/types/parameters.hpp"

class ClusterCreator
{
public:
  explicit ClusterCreator(const Parameters &params);
  std::vector<ClusterInfo> create_voxel_clustering(const std::vector<Point3D> &points);
  std::vector<Voxel> create_voxel(const std::vector<Point3D> &points);

  // 静的メソッド
  static std::string voxel_to_key(int x, int y, int z);
  static std::string point_to_voxel_key(const Point3D &point, const Parameters &params);

private:
  void collect_cluster_points(VoxelCluster &cluster, const std::vector<Point3D> &points);
  std::vector<std::string> get_adjacent_voxels(const std::string &key) const;
  bool point_in_voxel(const Point3D &point, const Voxel &voxel) const;
  Parameters params_;
};

#endif // BALL_DETECTOR_CLUSTER_CREATOR_HPP