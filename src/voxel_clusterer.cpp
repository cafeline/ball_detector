#include "ball_detector/voxel_clusterer.hpp"
#include <cstdio>

namespace ball_detector
{

  inline std::string voxel_key(int x, int y, int z)
  {
    return std::to_string(x) + "," + std::to_string(y) + "," + std::to_string(z);
  }

  std::vector<std::string> VoxelClusterer::get_adjacent_keys(const std::string &key) const
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
          neighbors.push_back(voxel_key(cx + dx, cy + dy, cz + dz));
        }
      }
    }
    return neighbors;
  }

  std::vector<VoxelCluster> VoxelClusterer::create_clusters(const std::vector<Point3D> &points, const std::vector<Voxel> &voxels)
  {
    // occupied_voxels: キー（文字列）→Voxel
    std::unordered_map<std::string, Voxel> occupied;
    for (const auto &v : voxels)
    {
      std::string key = voxel_key(v.x, v.y, v.z);
      occupied[key] = v;
    }

    std::unordered_map<std::string, bool> visited;
    std::vector<VoxelCluster> clusters;
    std::queue<std::string> q;

    for (const auto &pair : occupied)
    {
      if (visited.find(pair.first) != visited.end())
        continue;
      VoxelCluster cluster;
      q.push(pair.first);
      visited[pair.first] = true;
      while (!q.empty())
      {
        std::string cur = q.front();
        q.pop();
        cluster.voxels.push_back(occupied[cur]);
        for (const auto &neighbor : get_adjacent_keys(cur))
        {
          if (occupied.find(neighbor) != occupied.end() && visited.find(neighbor) == visited.end())
          {
            q.push(neighbor);
            visited[neighbor] = true;
          }
        }
      }
      // ※ 必要に応じて、cluster.points の収集は ClusterAnalyzer 側で行うことも可能です。
      clusters.push_back(cluster);
    }
    return clusters;
  }

} // namespace ball_detector
