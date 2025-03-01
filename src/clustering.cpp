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

std::vector<ClusterInfo> Clustering::create_voxel_clustering(const std::vector<Point3D> &points,
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

  // クラスタリング処理（各クラスタはVoxelClusterとして作成）
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

void Clustering::process_clusters(const std::vector<Point3D> &processed_points,
                                  std::vector<ClusterInfo> &clusters,
                                  rclcpp::Time current_time,
                                  double dt)
{
  // mark_ball_clusters の呼び出し時に、外部の processed_points は渡さない
  mark_ball_clusters(clusters);

  // 動的クラスタの識別
  std::vector<VoxelCluster> raw_clusters;
  for (const auto &ci : clusters)
  {
    raw_clusters.push_back(ci.cluster);
  }
  std::vector<int> assignments = cluster_tracking_.associateClusters(raw_clusters,
                                                                     params_.max_distance_for_association,
                                                                     current_time,
                                                                     dt);
  std::vector<VoxelCluster> dyn_raw = cluster_tracking_.filterBySpeed(raw_clusters,
                                                                      assignments,
                                                                      dt,
                                                                      params_.ball_vel_min);
  cluster_tracking_.removeMissingTracks();

  std::vector<ClusterInfo> dynamic_clusters;
  for (const auto &ci : clusters)
  {
    for (const auto &dc : dyn_raw)
    {
      if (ci.cluster.voxels.size() == dc.voxels.size())
      {
        bool match = true;
        for (size_t i = 0; i < ci.cluster.voxels.size(); ++i)
        {
          if (ci.cluster.voxels[i].x != dc.voxels[i].x ||
              ci.cluster.voxels[i].y != dc.voxels[i].y ||
              ci.cluster.voxels[i].z != dc.voxels[i].z)
          {
            match = false;
            break;
          }
        }
        if (match)
        {
          dynamic_clusters.push_back(ci);
          break;
        }
      }
    }
  }

  // 動的クラスタのマーク
  mark_dynamic_clusters(clusters, dynamic_clusters);

  // 動的かつボールクラスタ（両方の条件を満たすもの）のみとするため、フィルタを実施
  filter_dynamic_ball_clusters_near_boundaries(clusters);

  // 以降、clusters の各 ClusterInfo が is_ball_cluster, is_dynamic_ball のフラグで管理されるようになる
}

void Clustering::refine_ball_clusters(std::vector<ClusterInfo> &clusters, const Point3D &ball_position)
{
  if (is_zero_position(ball_position))
  {
    return;
  }

  size_t best_cluster_idx = find_closest_ball_cluster(clusters, ball_position);
  if (best_cluster_idx == SIZE_MAX)
  {
    return;
  }

  update_ball_cluster(clusters, best_cluster_idx, ball_position);
}

bool Clustering::is_zero_position(const Point3D &position) const
{
  const double epsilon = 1e-9;
  return std::abs(position.x) < epsilon &&
         std::abs(position.y) < epsilon &&
         std::abs(position.z) < epsilon;
}

size_t Clustering::find_closest_ball_cluster(const std::vector<ClusterInfo> &clusters,
                                             const Point3D &ball_position) const
{
  double min_dist = std::numeric_limits<double>::max();
  size_t best_id = SIZE_MAX;

  for (const auto &ci : clusters)
  {
    if (ci.is_dynamic_ball)
    {
      Point3D centroid = calculate_cluster_centroid(ci.cluster);
      double dist = calculate_distance(centroid, ball_position);
      if (dist < min_dist)
      {
        min_dist = dist;
        best_id = ci.index;
      }
    }
  }
  return best_id;
}

double Clustering::calculate_distance(const Point3D &a, const Point3D &b) const
{
  double dx = a.x - b.x;
  double dy = a.y - b.y;
  double dz = a.z - b.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

void Clustering::update_ball_cluster(std::vector<ClusterInfo> &clusters,
                                     size_t best_idx,
                                     const Point3D &ball_position)
{
  if (best_idx < clusters.size())
  {
    clusters[best_idx].cluster.points.clear();
    clusters[best_idx].cluster.points.push_back(ball_position);

    // フラグを明示的に true に設定する
    clusters[best_idx].is_ball_cluster = true;
    clusters[best_idx].is_dynamic_ball = true;
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

// 再利用可能な関数
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

void Clustering::mark_ball_clusters(std::vector<ClusterInfo> &clusters)
{
  for (auto &ci : clusters)
  {
    double size_x, size_y, size_z;
    calculate_cluster_size(ci.cluster, size_x, size_y, size_z);
    // 各クラスタ内部の点群からサイズを計算し、ボールサイズかどうかを判定
    ci.is_ball_cluster = is_ball_size(size_x, size_y, size_z);
  }
}

void Clustering::mark_dynamic_clusters(std::vector<ClusterInfo> &clusters, const std::vector<ClusterInfo> &dynamic_clusters)
{
  // まず全クラスタの動的フラグをクリア
  for (auto &ci : clusters)
  {
    ci.is_dynamic_ball = false;
  }
  // 動的クラスタとして識別されたクラスタにフラグを立てる
  for (const auto &dyn_ci : dynamic_clusters)
  {
    for (auto &ci : clusters)
    {
      if (ci.cluster.voxels.size() == dyn_ci.cluster.voxels.size())
      {
        bool match = true;
        for (size_t i = 0; i < ci.cluster.voxels.size(); ++i)
        {
          if (ci.cluster.voxels[i].x != dyn_ci.cluster.voxels[i].x ||
              ci.cluster.voxels[i].y != dyn_ci.cluster.voxels[i].y ||
              ci.cluster.voxels[i].z != dyn_ci.cluster.voxels[i].z)
          {
            match = false;
            break;
          }
        }
        if (match)
        {
          // ここでボールサイズチェックを追加
          double size_x, size_y, size_z;
          calculate_cluster_size(ci.cluster, size_x, size_y, size_z);
          if (is_ball_size(size_x, size_y, size_z))
          {
            ci.is_dynamic_ball = true;
          }
          break;
        }
      }
    }
  }
}

void Clustering::filter_dynamic_ball_clusters_near_boundaries(std::vector<ClusterInfo> &clusters)
{
  for (auto &ci : clusters)
  {
    if (ci.is_dynamic_ball)
    {
      Point3D centroid = calculate_cluster_centroid(ci.cluster);
      // 境界近傍の場合はフラグをクリアする
      if (is_near_boundary(centroid))
      {
        ci.is_dynamic_ball = false;
        ci.is_ball_cluster = false; // 必要に応じて、ボールクラスタ判定もクリア
      }
    }
  }
}

bool Clustering::is_near_boundary(const Point3D &centroid) const
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

bool Clustering::is_ball_size(double size_x, double size_y, double size_z) const
{
  double max_diameter = 2 * params_.ball_radius;
  return (size_x <= max_diameter &&
          size_y <= max_diameter &&
          size_z <= max_diameter);
}
