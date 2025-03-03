#include "ball_detector/cluster/cluster_classifier.hpp"
#include <algorithm>
#include <limits>
#include <random>
#include <string>
#include <cmath>
#include <rclcpp/rclcpp.hpp>

ClusterClassifier::ClusterClassifier(const Parameters &params)
    : params_(params)
{
}

void ClusterClassifier::identify_ball_candidates(std::vector<ClusterInfo> &clusters)
{
  for (auto &ci : clusters)
  {
    double size_x, size_y, size_z;
    calculate_cluster_size(ci.cluster, size_x, size_y, size_z);
    if (is_ball_size(size_x, size_y, size_z))
    {
      ci.type = ClusterType::BALL_CANDIDATE;
    }
    else
    {
      ci.type = ClusterType::LARGE;
    }
  }
}

void ClusterClassifier::calculate_cluster_size(const VoxelCluster &cluster,
                                               double &size_x,
                                               double &size_y,
                                               double &size_z) const
{
  double min_x = std::numeric_limits<double>::max();
  double min_y = std::numeric_limits<double>::max();
  double min_z = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();
  double max_y = std::numeric_limits<double>::lowest();
  double max_z = std::numeric_limits<double>::lowest();

  for (const auto &point : cluster.points)
  {
    min_x = std::min(min_x, static_cast<double>(point.x));
    max_x = std::max(max_x, static_cast<double>(point.x));
    min_y = std::min(min_y, static_cast<double>(point.y));
    max_y = std::max(max_y, static_cast<double>(point.y));
    min_z = std::min(min_z, static_cast<double>(point.z));
    max_z = std::max(max_z, static_cast<double>(point.z));
  }
  size_x = max_x - min_x;
  size_y = max_y - min_y;
  size_z = max_z - min_z;
}

bool ClusterClassifier::is_ball_size(double size_x, double size_y, double size_z) const
{
  double max_diameter = 2 * params_.ball_radius;
  return (size_x <= max_diameter &&
          size_y <= max_diameter &&
          size_z <= max_diameter);
}

void ClusterClassifier::filter_clusters_near_boundaries(std::vector<ClusterInfo> &clusters)
{
  for (auto &ci : clusters)
  {
    if (ci.type == ClusterType::DYNAMIC_BALL)
    {
      Point3D centroid = calculate_cluster_centroid(ci.cluster);
      if (is_near_boundary(centroid))
      {
        // 境界付近のクラスタはボール候補から大きいクラスタに戻す
        ci.type = ClusterType::LARGE;
      }
    }
  }
}

bool ClusterClassifier::is_near_boundary(const Point3D &centroid) const
{
  double buffer = 2.0;
  bool near_boundary = ((centroid.x - params_.min_x) < buffer * params_.voxel_size_x) ||
                       ((params_.max_x - centroid.x) < buffer * params_.voxel_size_x) ||
                       ((centroid.y - params_.min_y) < buffer * params_.voxel_size_y) ||
                       ((params_.max_y - centroid.y) < buffer * params_.voxel_size_y) ||
                       ((centroid.z - params_.min_z) < buffer * params_.voxel_size_z) ||
                       ((params_.max_z - centroid.z) < buffer * params_.voxel_size_z);
  return near_boundary;
}

Point3D ClusterClassifier::calculate_cluster_centroid(const VoxelCluster &cluster) const
{
  if (cluster.points.empty())
  {
    return Point3D{0.0, 0.0, 0.0};
  }

  float sum_x = 0.0;
  float sum_y = 0.0;
  float sum_z = 0.0;

  for (const auto &point : cluster.points)
  {
    sum_x += point.x;
    sum_y += point.y;
    sum_z += point.z;
  }

  float num_points = static_cast<float>(cluster.points.size());
  return Point3D{sum_x / num_points, sum_y / num_points, sum_z / num_points};
}