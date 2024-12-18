#include "ball_detector/clustering.hpp"
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <limits>
#include <random>
#include <string>
#include <cmath>
#include <rclcpp/rclcpp.hpp>

VoxelProcessor::VoxelProcessor(const Parameters &params) : params_(params) {}

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

Clustering::Clustering(const Parameters &params) : params_(params) {}

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

    collect_cluster_points(cluster, points);
    clusters.push_back(cluster);
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

void Clustering::process_clusters(const std::vector<Point3D> &processed_points, const std::vector<VoxelCluster> &clusters, rclcpp::Time current_time, double dt)
{
  calc_ball_clusters_indices(clusters, processed_points);
  std::vector<VoxelCluster> ball_clusters = extract_ball_clusters(clusters);
  // clustering側でアソシエーション
  std::vector<int> assignments = associate_clusters(clusters, tracks_, params_.max_distance_for_association, current_time, dt);
  remove_missing_tracks();
  std::vector<VoxelCluster> dynamic_clusters = filter_by_speed(clusters, assignments, tracks_, dt, params_.ball_vel_min);
  identify_dynamic_clusters(clusters, dynamic_clusters);
}

void Clustering::remove_missing_tracks()
{
  for (auto it = tracks_.begin(); it != tracks_.end();)
  {
    if (it->second.missing_count > 15)
      it = tracks_.erase(it);
    else
      ++it;
  }
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

bool Clustering::is_ball_size(double size_x, double size_y, double size_z) const
{
  return (size_x <= 2 * params_.ball_radius &&
          size_y <= 2 * params_.ball_radius &&
          size_z <= 2 * params_.ball_radius);
}

bool Clustering::are_centroids_close(const Point3D &a, const Point3D &b) const
{
  double dx = a.x - b.x;
  double dy = a.y - b.y;
  double dz = a.z - b.z;
  double tol = 1e-6;
  return (dx * dx + dy * dy + dz * dz) < (tol * tol);
}

Point3D Clustering::calculate_cluster_centroid(const VoxelCluster &cluster)
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

void Clustering::identify_dynamic_clusters(const std::vector<VoxelCluster> &clusters, const std::vector<VoxelCluster> &dynamic_clusters)
{
  dynamic_cluster_indices_.clear();
  for (const auto &dyn_cluster : dynamic_clusters)
  {
    Point3D dyn_center = calculate_cluster_centroid(dyn_cluster);
    for (size_t i = 0; i < clusters.size(); ++i)
    {
      Point3D orig_center = calculate_cluster_centroid(clusters[i]);
      if (are_centroids_close(orig_center, dyn_center))
      {
        dynamic_cluster_indices_.insert(i);
        break;
      }
    }
  }
}

std::vector<VoxelCluster> Clustering::extract_ball_clusters(const std::vector<VoxelCluster> &clusters)
{
  std::vector<VoxelCluster> ball_clusters;
  const auto &ball_indices = get_ball_size_cluster_indices();

  for (size_t i = 0; i < clusters.size(); ++i)
  {
    bool is_ball_cluster = (ball_indices.find(i) != ball_indices.end());
    if (is_ball_cluster)
    {
      ball_clusters.push_back(clusters[i]);
    }
  }
  return ball_clusters;
}

std::vector<VoxelCluster> Clustering::extract_dynamic_ball_clusters(const std::vector<VoxelCluster> &clusters)
{
  std::vector<VoxelCluster> dynamic_ball_clusters;
  const auto &dynamic_ball_indices = get_dynamic_ball_cluster_indices();

  for (size_t i = 0; i < clusters.size(); ++i)
  {
    bool is_dynamic_ball_cluster = (dynamic_ball_indices.find(i) != dynamic_ball_indices.end());
    if (is_dynamic_ball_cluster)
    {
      dynamic_ball_clusters.push_back(clusters[i]);
    }
  }
  return dynamic_ball_clusters;
}

void Clustering::calc_ball_clusters_indices(const std::vector<VoxelCluster> &clusters, const std::vector<Point3D> &points)
{
  ball_size_cluster_indices_.clear();

  for (size_t i = 0; i < clusters.size(); ++i)
  {
    double size_x, size_y, size_z;
    calculate_cluster_size(clusters[i], points, size_x, size_y, size_z);
    if (is_ball_size(size_x, size_y, size_z))
    {
      ball_size_cluster_indices_.insert(i);
    }
  }
}

void Clustering::calc_dynamic_ball_cluster_indices(const std::vector<VoxelCluster> &clusters)
{
  const auto &ball_indices = get_ball_size_cluster_indices();
  const auto &dynamic_indices = get_dynamic_cluster_indices();

  dynamic_ball_cluster_indices_.clear();

  for (size_t i = 0; i < clusters.size(); ++i)
  {
    bool is_ball_cluster = (ball_indices.find(i) != ball_indices.end());
    bool is_dynamic = (dynamic_indices.find(i) != dynamic_indices.end());

    if (is_ball_cluster && is_dynamic)
    {
      dynamic_ball_cluster_indices_.insert(i);
    }
  }
}

std::vector<int> Clustering::associate_clusters(const std::vector<VoxelCluster> &current_clusters,
                                                std::map<int, ClusterTrack> &tracks,
                                                double max_distance_for_association,
                                                rclcpp::Time current_time,
                                                double dt)
{
  std::vector<int> assignments(current_clusters.size(), -1);
  std::vector<bool> track_used(tracks.size(), false);

  std::vector<std::map<int, ClusterTrack>::iterator> track_list;
  track_list.reserve(tracks.size());
  for (auto it = tracks.begin(); it != tracks.end(); ++it)
  {
    track_list.push_back(it);
  }

  for (size_t i = 0; i < current_clusters.size(); i++)
  {
    Point3D c = calculate_cluster_centroid(current_clusters[i]);
    double best_dist = std::numeric_limits<double>::max();
    int best_idx = -1;
    for (size_t j = 0; j < track_list.size(); j++)
    {
      if (track_used[j])
        continue;
      Point3D t = track_list[j]->second.last_centroid;
      double dx = c.x - t.x;
      double dy = c.y - t.y;
      double dz = c.z - t.z;
      double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
      if (dist < best_dist && dist < max_distance_for_association)
      {
        best_dist = dist;
        best_idx = (int)j;
      }
    }
    if (best_idx >= 0)
    {
      assignments[i] = track_list[best_idx]->first;
      track_used[best_idx] = true;
      track_list[best_idx]->second.last_update_time = current_time;
      track_list[best_idx]->second.missing_count = 0;
    }
  }

  for (size_t i = 0; i < assignments.size(); i++)
  {
    if (assignments[i] < 0)
    {
      int new_id = tracks.empty() ? 0 : tracks.rbegin()->first + 1;
      Point3D c = calculate_cluster_centroid(current_clusters[i]);
      ClusterTrack new_track{new_id, c, current_time, 0};
      tracks[new_id] = new_track;
      assignments[i] = new_id;
    }
  }

  for (size_t j = 0; j < track_list.size(); j++)
  {
    if (!track_used[j])
    {
      track_list[j]->second.missing_count += 1;
    }
  }

  return assignments;
}

std::vector<VoxelCluster> Clustering::filter_by_speed(const std::vector<VoxelCluster> &current_clusters,
                                                      const std::vector<int> &assignments,
                                                      std::map<int, ClusterTrack> &tracks,
                                                      double dt,
                                                      double speed_threshold)
{
  std::vector<VoxelCluster> result;
  for (size_t i = 0; i < current_clusters.size(); i++)
  {
    int id = assignments[i];
    if (id < 0)
    {
      result.push_back(current_clusters[i]);
      continue;
    }
    auto it = tracks.find(id);
    if (it == tracks.end())
    {
      result.push_back(current_clusters[i]);
      continue;
    }

    Point3D last_c = it->second.last_centroid;
    Point3D cur_c = calculate_cluster_centroid(current_clusters[i]);
    double dx = cur_c.x - last_c.x;
    double dy = cur_c.y - last_c.y;
    double dz = cur_c.z - last_c.z;
    double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
    double speed = dist / dt;

    if (speed >= speed_threshold)
    {
      result.push_back(current_clusters[i]);
    }
    it->second.last_centroid = cur_c;
  }

  return result;
}
