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

  std::vector<int> associate_clusters(const std::vector<VoxelCluster> &current_clusters,
                                      double max_distance_for_association,
                                      rclcpp::Time current_time,
                                      double dt);

  void remove_missing_tracks();

  std::vector<VoxelCluster> filter_by_speed(const std::vector<VoxelCluster> &current_clusters,
                                            const std::vector<int> &assignments,
                                            double dt,
                                            double speed_threshold);

private:
  // トラック情報を管理するマップ
  std::map<int, ClusterTrack> tracks_;

  Point3D calculate_cluster_centroid(const VoxelCluster &cluster);
  double calculate_distance(const Point3D &p1, const Point3D &p2);
  int create_new_track(const VoxelCluster &cluster, rclcpp::Time current_time);
  void update_track(int track_id, const Point3D &centroid, rclcpp::Time current_time);
};
