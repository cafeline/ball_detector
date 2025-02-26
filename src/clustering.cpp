#include "ball_detector/clustering.hpp"
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <limits>
#include <random>
#include <string>
#include <cmath>
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <sstream>

Clustering::Clustering(const Parameters &params)
    : params_(params), cluster_tracking_()
{
}

void Clustering::process_clusters(const std::vector<Point3D> &processed_points,
                                  const std::vector<VoxelCluster> &clusters,
                                  rclcpp::Time current_time,
                                  double dt)
{
  // ボールクラスタのインデックス計算
  calc_ball_clusters_indices(clusters, processed_points);

  // ClusterTracking を使って動的クラスタを識別
  std::vector<VoxelCluster> dynamic_clusters = identify_dynamic_clusters(clusters, current_time, dt);

  // 動的クラスタのインデックス計算
  calc_dynamic_cluster_indices(clusters, dynamic_clusters);
  calc_dynamic_ball_cluster_indices(clusters);

  filter_dynamic_ball_clusters_near_boundaries(clusters);
}

std::vector<VoxelCluster> Clustering::create_voxel_clustering(const std::vector<Point3D> &points,
                                                              const std::vector<Voxel> &voxels)
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
        if (occupied_voxels.find(neighbor_key) != occupied_voxels.end() &&
            visited.find(neighbor_key) == visited.end())
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

void Clustering::calculate_cluster_size(const VoxelCluster &cluster,
                                        const std::vector<Point3D> &points,
                                        double &size_x,
                                        double &size_y,
                                        double &size_z) const
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

std::vector<VoxelCluster> Clustering::identify_dynamic_clusters(const std::vector<VoxelCluster> &clusters,
                                                                const rclcpp::Time &current_time,
                                                                double dt)
{
  std::vector<int> assignments = cluster_tracking_.associateClusters(clusters,
                                                                     params_.max_distance_for_association,
                                                                     current_time,
                                                                     dt);
  std::vector<VoxelCluster> dynamic_clusters = cluster_tracking_.filterBySpeed(clusters,
                                                                               assignments,
                                                                               dt,
                                                                               params_.ball_vel_min);
  cluster_tracking_.removeMissingTracks();
  return dynamic_clusters;
}

void Clustering::calc_ball_clusters_indices(const std::vector<VoxelCluster> &clusters,
                                            const std::vector<Point3D> &points)
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

std::vector<VoxelCluster> Clustering::extract_ball_clusters(const std::vector<VoxelCluster> &clusters)
{
  std::vector<VoxelCluster> ball_clusters;
  for (size_t i = 0; i < clusters.size(); ++i)
  {
    if (ball_size_cluster_indices_.find(i) != ball_size_cluster_indices_.end())
    {
      ball_clusters.push_back(clusters[i]);
    }
  }
  return ball_clusters;
}

std::vector<VoxelCluster> Clustering::extract_dynamic_ball_clusters(const std::vector<VoxelCluster> &clusters)
{
  std::vector<VoxelCluster> dynamic_ball_clusters;
  for (size_t i = 0; i < clusters.size(); ++i)
  {
    if (dynamic_ball_cluster_indices_.find(i) != dynamic_ball_cluster_indices_.end())
    {
      dynamic_ball_clusters.push_back(clusters[i]);
    }
  }
  return dynamic_ball_clusters;
}

void Clustering::calc_dynamic_ball_cluster_indices(const std::vector<VoxelCluster> &clusters)
{
  const auto &ball_indices = get_ball_size_cluster_indices();
  const auto &dynamic_indices = get_dynamic_cluster_indices();

  dynamic_ball_cluster_indices_.clear();

  for (size_t i = 0; i < clusters.size(); ++i)
  {
    if (ball_indices.find(i) != ball_indices.end() && dynamic_indices.find(i) != dynamic_indices.end())
    {
      dynamic_ball_cluster_indices_.insert(i);
    }
  }
}

void Clustering::calc_dynamic_cluster_indices(const std::vector<VoxelCluster> &clusters,
                                              const std::vector<VoxelCluster> &dynamic_clusters)
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

bool Clustering::is_ball_size(double size_x, double size_y, double size_z) const
{
  return (size_x <= 2 * params_.ball_radius &&
          size_y <= 2 * params_.ball_radius &&
          size_z <= 2 * params_.ball_radius);
}

Point3D Clustering::calculate_cluster_centroid(const VoxelCluster &cluster)
{
  Point3D centroid{0.0f, 0.0f, 0.0f};
  if (cluster.points.empty())
    return centroid;

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

bool Clustering::are_centroids_close(const Point3D &a, const Point3D &b) const
{
  double dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
  double tol = 1e-6;
  return (dx * dx + dy * dy + dz * dz) < (tol * tol);
}

void Clustering::filter_dynamic_ball_clusters_near_boundaries(const std::vector<VoxelCluster> &clusters)
{
  std::unordered_set<size_t> filtered_dynamic_ball;
  for (auto i : dynamic_ball_cluster_indices_)
  {
    Point3D centroid = calculate_cluster_centroid(clusters[i]);
    bool near_boundary =
        ((centroid.x - params_.min_x) < 2 * params_.voxel_size_x) ||
        ((params_.max_x - centroid.x) < 2 * params_.voxel_size_x) ||
        ((centroid.y - params_.min_y) < 2 * params_.voxel_size_y) ||
        ((params_.max_y - centroid.y) < 2 * params_.voxel_size_y) ||
        ((centroid.z - params_.min_z) < 2 * params_.voxel_size_z) ||
        ((params_.max_z - centroid.z) < 2 * params_.voxel_size_z);

    if (near_boundary)
    {
      ball_size_cluster_indices_.erase(i);
      dynamic_cluster_indices_.erase(i);
    }
    else
    {
      filtered_dynamic_ball.insert(i);
    }
  }
  dynamic_ball_cluster_indices_ = filtered_dynamic_ball;
}

void Clustering::refine_ball_clusters(std::vector<VoxelCluster> &clusters, const Point3D &ball_position)
{
  if (ball_position.x == 0.0 && ball_position.y == 0.0 && ball_position.z == 0.0)
    return;

  double min_dist = std::numeric_limits<double>::max();
  size_t best_cluster_idx = std::numeric_limits<size_t>::max();

  for (auto idx : dynamic_ball_cluster_indices_)
  {
    Point3D centroid = calculate_cluster_centroid(clusters[idx]);
    double dx = centroid.x - ball_position.x;
    double dy = centroid.y - ball_position.y;
    double dz = centroid.z - ball_position.z;
    double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (dist < min_dist)
    {
      min_dist = dist;
      best_cluster_idx = idx;
    }
  }

  if (best_cluster_idx == std::numeric_limits<size_t>::max())
    return;

  clusters[best_cluster_idx].points.clear();
  clusters[best_cluster_idx].points.push_back(ball_position);

  std::unordered_set<size_t> refined_dynamic_ball;
  refined_dynamic_ball.insert(best_cluster_idx);
  dynamic_ball_cluster_indices_ = refined_dynamic_ball;

  std::unordered_set<size_t> refined_ball_size;
  if (ball_size_cluster_indices_.find(best_cluster_idx) != ball_size_cluster_indices_.end())
    refined_ball_size.insert(best_cluster_idx);
  ball_size_cluster_indices_ = refined_ball_size;

  std::unordered_set<size_t> refined_dynamic;
  if (dynamic_cluster_indices_.find(best_cluster_idx) != dynamic_cluster_indices_.end())
    refined_dynamic.insert(best_cluster_idx);
  dynamic_cluster_indices_ = refined_dynamic;
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
        neighbors.emplace_back(std::to_string(cx + dx) + "," +
                               std::to_string(cy + dy) + "," +
                               std::to_string(cz + dz));
      }
    }
  }
  return neighbors;
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
