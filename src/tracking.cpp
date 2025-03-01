#include "ball_detector/tracking.hpp"

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

//---------------------------
// TrackingEngine実装
//---------------------------

std::vector<int> TrackingEngine::associateClusters(
    const std::vector<VoxelCluster> &current_clusters,
    double max_distance_for_association,
    rclcpp::Time current_time,
    double dt)
{
  std::vector<int> assignments(current_clusters.size(), -1);
  std::vector<bool> track_used(tracks_.size(), false);

  // トラックのイテレータをリスト化
  std::vector<std::map<int, ClusterTrack>::iterator> track_list;
  track_list.reserve(tracks_.size());
  for (auto it = tracks_.begin(); it != tracks_.end(); ++it)
  {
    track_list.push_back(it);
  }

  // 各クラスタと既存トラックとの関連付け
  for (size_t i = 0; i < current_clusters.size(); ++i)
  {
    Point3D cluster_centroid = tracking_utils::calculateClusterCentroid(current_clusters[i]);
    double best_distance = std::numeric_limits<double>::max();
    int best_track_idx = -1;

    // 最も近いトラックを探す
    for (size_t j = 0; j < track_list.size(); ++j)
    {
      if (track_used[j])
        continue;

      Point3D track_centroid = track_list[j]->second.last_centroid;
      double distance = tracking_utils::calculateDistance(cluster_centroid, track_centroid);

      if (distance < best_distance && distance < max_distance_for_association)
      {
        best_distance = distance;
        best_track_idx = static_cast<int>(j);
      }
    }

    // 最も近いトラックが見つかった場合、関連付け
    if (best_track_idx >= 0)
    {
      assignments[i] = track_list[best_track_idx]->first;
      track_used[best_track_idx] = true;
      updateTrack(track_list[best_track_idx]->first, cluster_centroid, current_time);
    }
  }

  // 未対応のクラスタに対して新たなトラックを生成
  for (size_t i = 0; i < assignments.size(); ++i)
  {
    if (assignments[i] < 0)
    {
      assignments[i] = createNewTrack(current_clusters[i], current_time);
    }
  }

  // 未使用トラックの missing_count 更新
  for (size_t j = 0; j < track_list.size(); ++j)
  {
    if (!track_used[j])
    {
      track_list[j]->second.missing_count += 1;
    }
  }

  return assignments;
}

void TrackingEngine::removeMissingTracks(int max_missing_count)
{
  for (auto it = tracks_.begin(); it != tracks_.end();)
  {
    if (it->second.missing_count > max_missing_count)
      it = tracks_.erase(it);
    else
      ++it;
  }
}

std::vector<VoxelCluster> TrackingEngine::filterBySpeed(
    const std::vector<VoxelCluster> &current_clusters,
    const std::vector<int> &assignments,
    double dt,
    double speed_threshold)
{
  std::vector<VoxelCluster> filtered_clusters;
  for (size_t i = 0; i < current_clusters.size(); ++i)
  {
    int track_id = assignments[i];
    if (track_id < 0)
    {
      filtered_clusters.push_back(current_clusters[i]);
      continue;
    }

    auto track_it = tracks_.find(track_id);
    if (track_it == tracks_.end())
    {
      filtered_clusters.push_back(current_clusters[i]);
      continue;
    }

    Point3D current_centroid = tracking_utils::calculateClusterCentroid(current_clusters[i]);
    Point3D last_centroid = track_it->second.last_centroid;

    double distance = tracking_utils::calculateDistance(current_centroid, last_centroid);
    double speed = distance / dt;

    if (speed >= speed_threshold)
    {
      filtered_clusters.push_back(current_clusters[i]);
    }

    // トラックの重心を更新
    track_it->second.last_centroid = current_centroid;
  }
  return filtered_clusters;
}

int TrackingEngine::createNewTrack(const VoxelCluster &cluster, rclcpp::Time current_time)
{
  int new_id = tracks_.empty() ? 0 : tracks_.rbegin()->first + 1;
  Point3D centroid = tracking_utils::calculateClusterCentroid(cluster);
  ClusterTrack new_track{new_id, centroid, current_time, 0};
  tracks_[new_id] = new_track;
  return new_id;
}

void TrackingEngine::updateTrack(int track_id, const Point3D &centroid, rclcpp::Time current_time)
{
  auto it = tracks_.find(track_id);
  if (it != tracks_.end())
  {
    it->second.last_update_time = current_time;
    it->second.missing_count = 0;
  }
}

//---------------------------
// TrackingManager実装
//---------------------------

TrackingManager::TrackingManager()
    : tracking_engine_()
{
}

void TrackingManager::identify_dynamic_clusters(
    std::vector<ClusterInfo> &clusters,
    rclcpp::Time current_time,
    double dt,
    const Parameters &params)
{
  // 動的クラスタの識別処理
  std::vector<VoxelCluster> raw_clusters;
  for (const auto &ci : clusters)
  {
    raw_clusters.push_back(ci.cluster);
  }

  // クラスタの関連付け
  std::vector<int> assignments = tracking_engine_.associateClusters(
      raw_clusters,
      params.max_distance_for_association,
      current_time,
      dt);

  // 速度ベースでフィルタリング
  std::vector<VoxelCluster> dynamic_clusters = tracking_engine_.filterBySpeed(
      raw_clusters,
      assignments,
      dt,
      params.ball_vel_min);

  // トラックの更新
  tracking_engine_.removeMissingTracks();

  // 動的クラスタのマーキング
  mark_dynamic_clusters(clusters, dynamic_clusters);
}

void TrackingManager::mark_dynamic_clusters(
    std::vector<ClusterInfo> &clusters,
    const std::vector<VoxelCluster> &dynamic_clusters)
{
  // すべてのクラスタの動的フラグを初期化（既存のtypeを保持）
  for (const auto &dyn_cluster : dynamic_clusters)
  {
    for (auto &ci : clusters)
    {
      if (tracking_utils::clusters_match(ci.cluster, dyn_cluster) &&
          ci.type == ClusterType::BALL_CANDIDATE)
      {
        ci.type = ClusterType::DYNAMIC_BALL;
        break;
      }
    }
  }
}

void TrackingManager::refine_ball_clusters(
    std::vector<ClusterInfo> &clusters,
    const Point3D &ball_position,
    const Parameters &params)
{
  if (tracking_utils::is_zero_position(ball_position))
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

size_t TrackingManager::find_closest_ball_cluster(
    const std::vector<ClusterInfo> &clusters,
    const Point3D &ball_position) const
{
  double min_dist = std::numeric_limits<double>::max();
  size_t best_id = SIZE_MAX;

  for (const auto &ci : clusters)
  {
    if (ci.type == ClusterType::DYNAMIC_BALL)
    {
      Point3D centroid = tracking_utils::calculateClusterCentroid(ci.cluster);
      double dist = tracking_utils::calculateDistance(centroid, ball_position);
      if (dist < min_dist)
      {
        min_dist = dist;
        best_id = ci.index;
      }
    }
  }
  return best_id;
}

void TrackingManager::update_ball_cluster(
    std::vector<ClusterInfo> &clusters,
    size_t best_idx,
    const Point3D &ball_position)
{
  if (best_idx < clusters.size())
  {
    clusters[best_idx].cluster.points.clear();
    clusters[best_idx].cluster.points.push_back(ball_position);

    // タイプを明示的に設定
    clusters[best_idx].type = ClusterType::DYNAMIC_BALL;
  }
}