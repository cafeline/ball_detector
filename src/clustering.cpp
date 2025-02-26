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
  // 現在のクラスタを保存
  current_clusters_ = clusters;

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
    std::string key = voxel_to_key(voxel.x, voxel.y, voxel.z);
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

std::vector<VoxelCluster> Clustering::extract_clusters_by_indices(
    const std::vector<VoxelCluster> &clusters,
    const std::unordered_set<size_t> &indices)
{
  std::vector<VoxelCluster> result;
  result.reserve(indices.size()); // 事前に必要なサイズを確保

  for (size_t i = 0; i < clusters.size(); ++i)
  {
    if (indices.find(i) != indices.end())
    {
      result.push_back(clusters[i]);
    }
  }

  return result;
}

std::vector<VoxelCluster> Clustering::extract_ball_clusters(const std::vector<VoxelCluster> &clusters)
{
  return extract_clusters_by_indices(clusters, ball_size_cluster_indices_);
}

std::vector<VoxelCluster> Clustering::extract_dynamic_ball_clusters(const std::vector<VoxelCluster> &clusters)
{
  return extract_clusters_by_indices(clusters, dynamic_ball_cluster_indices_);
}

void Clustering::calc_dynamic_ball_cluster_indices(const std::vector<VoxelCluster> &clusters)
{
  dynamic_ball_cluster_indices_ = set_intersection(ball_size_cluster_indices_, dynamic_cluster_indices_);
}

std::unordered_set<size_t> Clustering::set_intersection(
    const std::unordered_set<size_t> &set1,
    const std::unordered_set<size_t> &set2)
{
  std::unordered_set<size_t> result;

  // 小さい方のセットをループすることで効率化
  const auto &smaller = (set1.size() < set2.size()) ? set1 : set2;
  const auto &larger = (set1.size() < set2.size()) ? set2 : set1;

  for (auto item : smaller)
  {
    if (larger.find(item) != larger.end())
    {
      result.insert(item);
    }
  }

  return result;
}

void Clustering::calc_dynamic_cluster_indices(const std::vector<VoxelCluster> &clusters,
                                              const std::vector<VoxelCluster> &dynamic_clusters)
{
  dynamic_cluster_indices_.clear();

  // 元クラスタの中心点をあらかじめ計算して保存
  std::vector<Point3D> cluster_centroids;
  cluster_centroids.reserve(clusters.size());

  for (const auto &cluster : clusters)
  {
    cluster_centroids.push_back(calculate_cluster_centroid(cluster));
  }

  // 動的クラスタとの対応付け
  for (const auto &dyn_cluster : dynamic_clusters)
  {
    Point3D dyn_center = calculate_cluster_centroid(dyn_cluster);
    size_t closest_idx = find_closest_centroid(cluster_centroids, dyn_center);

    if (closest_idx != SIZE_MAX)
    {
      dynamic_cluster_indices_.insert(closest_idx);
    }
  }
}

size_t Clustering::find_closest_centroid(
    const std::vector<Point3D> &centroids,
    const Point3D &target) const
{
  const double tolerance_squared = 1e-6 * 1e-6;
  double min_dist_squared = std::numeric_limits<double>::max();
  size_t closest_idx = SIZE_MAX;

  for (size_t i = 0; i < centroids.size(); ++i)
  {
    double dx = centroids[i].x - target.x;
    double dy = centroids[i].y - target.y;
    double dz = centroids[i].z - target.z;
    double dist_squared = dx * dx + dy * dy + dz * dz;

    if (dist_squared < min_dist_squared)
    {
      min_dist_squared = dist_squared;
      closest_idx = i;
    }
  }

  // 距離が許容範囲内の場合のみ結果を返す
  return (min_dist_squared <= tolerance_squared) ? closest_idx : SIZE_MAX;
}

bool Clustering::is_ball_size(double size_x, double size_y, double size_z) const
{
  double max_diameter = 2 * params_.ball_radius;
  return (size_x <= max_diameter &&
          size_y <= max_diameter &&
          size_z <= max_diameter);
}

Point3D Clustering::calculate_cluster_centroid(const VoxelCluster &cluster) const
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
    if (!is_near_boundary(calculate_cluster_centroid(clusters[i])))
    {
      filtered_dynamic_ball.insert(i);
    }
    else
    {
      // 境界付近のクラスタを各セットから削除
      ball_size_cluster_indices_.erase(i);
      dynamic_cluster_indices_.erase(i);
    }
  }

  dynamic_ball_cluster_indices_ = filtered_dynamic_ball;
}

bool Clustering::is_near_boundary(const Point3D &centroid) const
{
  double buffer = 2.0;
  return ((centroid.x - params_.min_x) < buffer * params_.voxel_size_x) ||
         ((params_.max_x - centroid.x) < buffer * params_.voxel_size_x) ||
         ((centroid.y - params_.min_y) < buffer * params_.voxel_size_y) ||
         ((params_.max_y - centroid.y) < buffer * params_.voxel_size_y) ||
         ((centroid.z - params_.min_z) < buffer * params_.voxel_size_z) ||
         ((params_.max_z - centroid.z) < buffer * params_.voxel_size_z);
}

void Clustering::refine_ball_clusters(std::vector<VoxelCluster> &clusters, const Point3D &ball_position)
{
  // 無効な位置の場合は処理をスキップ
  if (is_zero_position(ball_position))
  {
    return;
  }

  size_t best_cluster_idx = find_closest_ball_cluster(clusters, ball_position);

  if (best_cluster_idx == SIZE_MAX)
  {
    return;
  }

  // 選択されたクラスタのみを残し、他のクラスタを除外
  update_ball_cluster(clusters, best_cluster_idx, ball_position);
}

bool Clustering::is_zero_position(const Point3D &position) const
{
  const double epsilon = 1e-9;
  return std::abs(position.x) < epsilon &&
         std::abs(position.y) < epsilon &&
         std::abs(position.z) < epsilon;
}

size_t Clustering::find_closest_ball_cluster(
    const std::vector<VoxelCluster> &clusters,
    const Point3D &ball_position) const
{
  double min_dist = std::numeric_limits<double>::max();
  size_t best_idx = SIZE_MAX;

  for (auto idx : dynamic_ball_cluster_indices_)
  {
    Point3D centroid = calculate_cluster_centroid(clusters[idx]);
    double dist = calculate_distance(centroid, ball_position);

    if (dist < min_dist)
    {
      min_dist = dist;
      best_idx = idx;
    }
  }

  return best_idx;
}

double Clustering::calculate_distance(const Point3D &a, const Point3D &b) const
{
  double dx = a.x - b.x;
  double dy = a.y - b.y;
  double dz = a.z - b.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

void Clustering::update_ball_cluster(
    std::vector<VoxelCluster> &clusters,
    size_t best_idx,
    const Point3D &ball_position)
{
  // 選択したクラスタの点を置き換え
  clusters[best_idx].points.clear();
  clusters[best_idx].points.push_back(ball_position);

  // インデックスセットを更新
  std::unordered_set<size_t> refined_set;
  refined_set.insert(best_idx);

  dynamic_ball_cluster_indices_ = refined_set;

  if (ball_size_cluster_indices_.find(best_idx) != ball_size_cluster_indices_.end())
  {
    ball_size_cluster_indices_ = refined_set;
  }
  else
  {
    ball_size_cluster_indices_.clear();
  }

  if (dynamic_cluster_indices_.find(best_idx) != dynamic_cluster_indices_.end())
  {
    dynamic_cluster_indices_ = refined_set;
  }
  else
  {
    dynamic_cluster_indices_.clear();
  }
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

// 再利用可能な関数としてヘッダに追加
inline std::string voxel_to_key(int x, int y, int z)
{
  return std::to_string(x) + "," + std::to_string(y) + "," + std::to_string(z);
}

inline std::string point_to_voxel_key(const Point3D &point, const Parameters &params)
{
  int vx = static_cast<int>((point.x - params.min_x) / params.voxel_size_x);
  int vy = static_cast<int>((point.y - params.min_y) / params.voxel_size_y);
  int vz = static_cast<int>((point.z - params.min_z) / params.voxel_size_z);
  return voxel_to_key(vx, vy, vz);
}
