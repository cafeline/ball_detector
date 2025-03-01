#include "ball_detector/tracking_utils.hpp"

namespace tracking_utils
{
  Point3D calculateClusterCentroid(const VoxelCluster &cluster)
  {
    Point3D centroid{0.0f, 0.0f, 0.0f};
    if (cluster.points.empty())
    {
      return centroid;
    }

    double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
    for (const auto &p : cluster.points)
    {
      sum_x += p.x;
      sum_y += p.y;
      sum_z += p.z;
    }
    double n = static_cast<double>(cluster.points.size());
    centroid.x = static_cast<float>(sum_x / n);
    centroid.y = static_cast<float>(sum_y / n);
    centroid.z = static_cast<float>(sum_z / n);
    return centroid;
  }

  double calculateDistance(const Point3D &p1, const Point3D &p2)
  {
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    double dz = p1.z - p2.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
  }

  bool is_zero_position(const Point3D &position, double epsilon)
  {
    return std::abs(position.x) < epsilon &&
           std::abs(position.y) < epsilon &&
           std::abs(position.z) < epsilon;
  }

  bool clusters_match(const VoxelCluster &a, const VoxelCluster &b)
  {
    if (a.voxels.size() != b.voxels.size())
      return false;

    // 単純化のためにヴォクセル数で比較
    for (size_t i = 0; i < a.voxels.size(); ++i)
    {
      if (a.voxels[i].x != b.voxels[i].x ||
          a.voxels[i].y != b.voxels[i].y ||
          a.voxels[i].z != b.voxels[i].z)
      {
        return false;
      }
    }
    return true;
  }
}