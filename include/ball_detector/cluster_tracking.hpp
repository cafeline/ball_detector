#pragma once

#include "ball_detector/types.hpp"
#include <vector>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <limits>
#include <cmath>

class ClusterTracking
{
public:
  ClusterTracking() = default;

  std::vector<int> associateClusters(const std::vector<VoxelCluster> &current_clusters,
                                     double max_distance_for_association,
                                     rclcpp::Time current_time,
                                     double dt);

  void removeMissingTracks();

  std::vector<VoxelCluster> filterBySpeed(const std::vector<VoxelCluster> &current_clusters,
                                          const std::vector<int> &assignments,
                                          double dt,
                                          double speed_threshold);

private:
  // トラック情報を管理するマップ
  std::map<int, ClusterTrack> tracks_;

  Point3D calculateClusterCentroid(const VoxelCluster &cluster);
};
