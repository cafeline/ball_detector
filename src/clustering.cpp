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

// ClusterCreator実装
ClusterCreator::ClusterCreator(const Parameters &params)
    : params_(params)
{
}

std::vector<ClusterInfo> ClusterCreator::create_voxel_clustering(const std::vector<Point3D> &points,
                                                                 const std::vector<Voxel> &voxels)
{
  std::unordered_map<std::string, Voxel> occupied_voxels;
  for (const auto &voxel : voxels)
  {
    std::string key = voxel_to_key(voxel.x, voxel.y, voxel.z);
    occupied_voxels[key] = voxel;
  }

  std::vector<VoxelCluster> temp_clusters;
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
    temp_clusters.push_back(cluster);
  }

  // 各クラスタに一意のIDを付与して ClusterInfo としてまとめる
  std::vector<ClusterInfo> cluster_infos;
  cluster_infos.reserve(temp_clusters.size());
  for (size_t i = 0; i < temp_clusters.size(); ++i)
  {
    ClusterInfo info;
    info.index = i;
    info.cluster = temp_clusters[i];
    cluster_infos.push_back(info);
  }

  return cluster_infos;
}

void ClusterCreator::collect_cluster_points(VoxelCluster &cluster, const std::vector<Point3D> &points)
{
  std::unordered_map<std::string, std::vector<size_t>> voxel_to_points;

  for (size_t i = 0; i < points.size(); ++i)
  {
    std::string key = point_to_voxel_key(points[i], params_);
    voxel_to_points[key].push_back(i);
  }

  for (const auto &voxel : cluster.voxels)
  {
    for (const auto &point_idx : voxel_to_points[voxel_to_key(voxel.x, voxel.y, voxel.z)])
    {
      Point3D point = points[point_idx];
      if (point_in_voxel(point, voxel))
      {
        cluster.points.push_back(point);
      }
    }
  }
}

std::vector<std::string> ClusterCreator::get_adjacent_voxels(const std::string &key) const
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

bool ClusterCreator::point_in_voxel(const Point3D &point, const Voxel &voxel) const
{
  return point.x >= params_.min_x + voxel.x * params_.voxel_size_x &&
         point.x < params_.min_x + (voxel.x + 1) * params_.voxel_size_x &&
         point.y >= params_.min_y + voxel.y * params_.voxel_size_y &&
         point.y < params_.min_y + (voxel.y + 1) * params_.voxel_size_y &&
         point.z >= params_.min_z + voxel.z * params_.voxel_size_z &&
         point.z < params_.min_z + (voxel.z + 1) * params_.voxel_size_z;
}

// 新しく追加する関数
std::vector<Voxel> ClusterCreator::create_voxel(const std::vector<Point3D> &points)
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

// ClusterClassifier実装
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

  // 既に抽出されたクラスタ内の点群を使う
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
        // 境界付近のクラスタはボール候補から静的クラスタに戻す
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

bool ClusterClassifier::are_centroids_close(const Point3D &a, const Point3D &b) const
{
  double dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
  double tol = 1e-6;
  return (dx * dx + dy * dy + dz * dz) < (tol * tol);
}

// ClusterManager実装
ClusterManager::ClusterManager(const Parameters &params)
    : params_(params),
      cluster_creator_(params),
      cluster_classifier_(params)
{
  tracking_manager_ = std::make_unique<TrackingManager>();
}

std::vector<ClusterInfo> ClusterManager::create_voxel_clustering(const std::vector<Point3D> &points,
                                                                 const std::vector<Voxel> &voxels)
{
  return cluster_creator_.create_voxel_clustering(points, voxels);
}

void ClusterManager::process_clusters(const std::vector<Point3D> &processed_points,
                                      std::vector<ClusterInfo> &clusters,
                                      rclcpp::Time current_time,
                                      double dt)
{
  // ボールサイズの識別
  cluster_classifier_.identify_ball_candidates(clusters);

  // 動的クラスタの識別
  tracking_manager_->identify_dynamic_clusters(clusters, current_time, dt, params_);

  // 境界付近のクラスタをフィルタリング
  cluster_classifier_.filter_clusters_near_boundaries(clusters);
}

void ClusterManager::refine_ball_clusters(std::vector<ClusterInfo> &clusters,
                                          const Point3D &ball_position)
{
  tracking_manager_->refine_ball_clusters(clusters, ball_position, params_);
}

// Clustering（ラッパークラス）実装
Clustering::Clustering(const Parameters &params)
    : params_(params), cluster_manager_(params)
{
  tracking_manager_ = std::make_unique<TrackingManager>();
}

std::vector<ClusterInfo> Clustering::create_voxel_clustering(const std::vector<Point3D> &points,
                                                             const std::vector<Voxel> &voxels)
{
  return cluster_manager_.create_voxel_clustering(points, voxels);
}

void Clustering::process_clusters(const std::vector<Point3D> &processed_points,
                                  std::vector<ClusterInfo> &clusters,
                                  rclcpp::Time current_time,
                                  double dt)
{
  cluster_manager_.process_clusters(processed_points, clusters, current_time, dt);
}

// 再利用可能なユーティリティ関数
std::string voxel_to_key(int x, int y, int z)
{
  return std::to_string(x) + "," + std::to_string(y) + "," + std::to_string(z);
}

std::string point_to_voxel_key(const Point3D &point, const Parameters &params)
{
  int vx = static_cast<int>((point.x - params.min_x) / params.voxel_size_x);
  int vy = static_cast<int>((point.y - params.min_y) / params.voxel_size_y);
  int vz = static_cast<int>((point.z - params.min_z) / params.voxel_size_z);
  return voxel_to_key(vx, vy, vz);
}
