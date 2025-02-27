#include "ball_detector/cluster_analyzer.hpp"
#include <limits>
#include <cmath>
#include <algorithm>

namespace ball_detector
{

  void ClusterAnalyzer::calculate_cluster_size(const VoxelCluster &cluster, const std::vector<Point3D> &points,
                                               double &size_x, double &size_y, double &size_z) const
  {
    double min_x = std::numeric_limits<double>::max(), max_x = std::numeric_limits<double>::lowest();
    double min_y = std::numeric_limits<double>::max(), max_y = std::numeric_limits<double>::lowest();
    double min_z = std::numeric_limits<double>::max(), max_z = std::numeric_limits<double>::lowest();

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

  Point3D ClusterAnalyzer::compute_centroid(const VoxelCluster &cluster) const
  {
    Point3D centroid{0.0, 0.0, 0.0};
    if (cluster.points.empty())
      return centroid;
    float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;
    for (const auto &point : cluster.points)
    {
      sum_x += point.x;
      sum_y += point.y;
      sum_z += point.z;
    }
    float n = static_cast<float>(cluster.points.size());
    centroid.x = sum_x / n;
    centroid.y = sum_y / n;
    centroid.z = sum_z / n;
    return centroid;
  }

  void ClusterAnalyzer::refine_ball_clusters(std::vector<VoxelCluster> &clusters, const Point3D &ball_position) const
  {
    // 無効な位置なら処理を行わない
    const double epsilon = 1e-9;
    if (std::abs(ball_position.x) < epsilon && std::abs(ball_position.y) < epsilon && std::abs(ball_position.z) < epsilon)
      return;

    size_t best_idx = SIZE_MAX;
    double min_dist = std::numeric_limits<double>::max();
    for (size_t i = 0; i < clusters.size(); ++i)
    {
      Point3D centroid = compute_centroid(clusters[i]);
      double dx = centroid.x - ball_position.x;
      double dy = centroid.y - ball_position.y;
      double dz = centroid.z - ball_position.z;
      double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
      if (dist < min_dist)
      {
        min_dist = dist;
        best_idx = i;
      }
    }
    if (best_idx != SIZE_MAX)
    {
      clusters[best_idx].points.clear();
      clusters[best_idx].points.push_back(ball_position);
    }
  }

} // namespace ball_detector
