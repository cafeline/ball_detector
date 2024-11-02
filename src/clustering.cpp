#include "clustering.hpp"
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <limits>
#include <random>
#include <string>

VoxelProcessor::VoxelProcessor(const Parameters &params) : params_(params)
{
}

std::vector<Voxel> VoxelProcessor::create_voxel(const std::vector<Point3D> &points)
{
  std::unordered_map<std::string, Voxel> occupied_voxels;
  for (const auto &point : points)
  {
    int vx = static_cast<int>((point.x - params_.min_x) / params_.voxel_size_x);
    int vy = static_cast<int>((point.y - params_.min_y) / params_.voxel_size_y);
    int vz = static_cast<int>((point.z - params_.min_z) / params_.voxel_size_z);
    std::string key = std::to_string(vx) + "," + std::to_string(vy) + "," + std::to_string(vz);
    if (occupied_voxels.find(key) == occupied_voxels.end())
    {
      occupied_voxels[key] = Voxel(vx, vy, vz);
    }
    else
    {
      occupied_voxels[key].increment();
    }
  }

  std::vector<Voxel> result;
  result.reserve(occupied_voxels.size());
  for (const auto &pair : occupied_voxels)
  {
    result.push_back(pair.second);
  }

  return result;
}

Clustering::Clustering(const Parameters &params)
    : params_(params)
{
}

std::vector<VoxelCluster> Clustering::create_voxel_clustering(const std::vector<Point3D> &points, const std::vector<Voxel> &voxels)
{
  std::unordered_map<std::string, Voxel> occupied_voxels;
  for (const auto &voxel : voxels)
  {
    std::string key = std::to_string(voxel.x) + "," + std::to_string(voxel.y) + "," + std::to_string(voxel.z);
    occupied_voxels[key] = voxel;
  }

  std::vector<VoxelCluster> clusters;
  std::unordered_map<std::string, bool> visited;
  std::queue<std::string> q;

  for (const auto &pair : occupied_voxels)
  {
    const std::string &key = pair.first;
    if (visited.find(key) != visited.end())
      continue;

    VoxelCluster cluster;
    q.push(key);
    visited[key] = true;

    while (!q.empty())
    {
      std::string current_key = q.front();
      q.pop();
      cluster.voxels.push_back(occupied_voxels[current_key]);

      auto neighbors = get_adjacent_voxels(current_key);
      for (const auto &neighbor_key : neighbors)
      {
        if (occupied_voxels.find(neighbor_key) != occupied_voxels.end() && visited.find(neighbor_key) == visited.end())
        {
          q.push(neighbor_key);
          visited[neighbor_key] = true;
        }
      }
    }

    if (is_valid_cluster(cluster, points))
    {
      collect_cluster_points(cluster, points);
      clusters.push_back(cluster);
    }
  }

  return clusters;
}

std::vector<std::string> Clustering::get_adjacent_voxels(const std::string &key) const
{
  int cx, cy, cz;
  sscanf(key.c_str(), "%d,%d,%d", &cx, &cy, &cz);
  std::vector<std::string> neighbors;
  for (int dx = -params_.voxel_search_range; dx <= params_.voxel_search_range; ++dx)
  {
    for (int dy = -params_.voxel_search_range; dy <= params_.voxel_search_range; ++dy)
    {
      for (int dz = -params_.voxel_search_range; dz <= params_.voxel_search_range; ++dz)
      {
        if (dx == 0 && dy == 0 && dz == 0)
          continue;
        neighbors.emplace_back(std::to_string(cx + dx) + "," + std::to_string(cy + dy) + "," + std::to_string(cz + dz));
      }
    }
  }
  return neighbors;
}

bool Clustering::is_valid_cluster(const VoxelCluster &cluster, const std::vector<Point3D> &points) const
{
  double size_x, size_y, size_z;
  calculate_cluster_size(cluster, points, size_x, size_y, size_z);
  return size_x <= 2 * params_.ball_radius && size_y <= 2 * params_.ball_radius && size_z <= 2 * params_.ball_radius && points.size() >= 3;
}

void Clustering::calculate_cluster_size(const VoxelCluster &cluster, const std::vector<Point3D> &points,
                                        double &size_x, double &size_y, double &size_z) const
{
  double min_x = std::numeric_limits<double>::max(), max_x = std::numeric_limits<double>::lowest();
  double min_y = std::numeric_limits<double>::max(), max_y = std::numeric_limits<double>::lowest();
  double min_z = std::numeric_limits<double>::max(), max_z = std::numeric_limits<double>::lowest();
  for (const auto &point : points)
  {
    for (const auto &voxel : cluster.voxels)
    {
      if (point_in_voxel(point, voxel))
      {
        min_x = std::min(min_x, static_cast<double>(point.x));
        max_x = std::max(max_x, static_cast<double>(point.x));
        min_y = std::min(min_y, static_cast<double>(point.y));
        max_y = std::max(max_y, static_cast<double>(point.y));
        min_z = std::min(min_z, static_cast<double>(point.z));
        max_z = std::max(max_z, static_cast<double>(point.z));
        break;
      }
    }
  }
  size_x = max_x - min_x;
  size_y = max_y - min_y;
  size_z = max_z - min_z;
}

void Clustering::collect_cluster_points(VoxelCluster &cluster, const std::vector<Point3D> &points)
{
  for (const auto &voxel : cluster.voxels)
  {
    for (const auto &point : points)
    {
      if (point_in_voxel(point, voxel))
      {
        cluster.points.push_back(point);
      }
    }
  }
}

bool Clustering::point_in_voxel(const Point3D &point, const Voxel &voxel) const
{
  return point.x >= params_.min_x + voxel.x * params_.voxel_size_x &&
         point.x < params_.min_x + (voxel.x + 1) * params_.voxel_size_x &&
         point.y >= params_.min_y + voxel.y * params_.voxel_size_y &&
         point.y < params_.min_y + (voxel.y + 1) * params_.voxel_size_y &&
         point.z >= params_.min_z + voxel.z * params_.voxel_size_z &&
         point.z < params_.min_z + (voxel.z + 1) * params_.voxel_size_z;
}