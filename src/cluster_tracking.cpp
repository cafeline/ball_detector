#include "ball_detector/cluster_tracking.hpp"

std::vector<int> ClusterTracking::associateClusters(const std::vector<VoxelCluster> &current_clusters,
                                                    double max_distance_for_association,
                                                    rclcpp::Time current_time,
                                                    double dt)
{
  std::vector<int> assignments(current_clusters.size(), -1);
  std::vector<bool> track_used(tracks_.size(), false);

  // tracks_ のイテレータをリスト化
  std::vector<std::map<int, ClusterTrack>::iterator> track_list;
  track_list.reserve(tracks_.size());
  for (auto it = tracks_.begin(); it != tracks_.end(); ++it)
  {
    track_list.push_back(it);
  }

  // 各クラスタと既存トラックとの距離を計算し、最も近いトラックと対応付ける
  for (size_t i = 0; i < current_clusters.size(); ++i)
  {
    Point3D c = calculateClusterCentroid(current_clusters[i]);
    double best_dist = std::numeric_limits<double>::max();
    int best_idx = -1;
    for (size_t j = 0; j < track_list.size(); ++j)
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
        best_idx = static_cast<int>(j);
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

  // 未対応のクラスタに対しては新たなトラックを生成
  for (size_t i = 0; i < assignments.size(); ++i)
  {
    if (assignments[i] < 0)
    {
      int new_id = tracks_.empty() ? 0 : tracks_.rbegin()->first + 1;
      Point3D c = calculateClusterCentroid(current_clusters[i]);
      ClusterTrack new_track{new_id, c, current_time, 0};
      tracks_[new_id] = new_track;
      assignments[i] = new_id;
    }
  }

  // 対応付けに使われなかったトラックは missing_count をインクリメント
  for (size_t j = 0; j < track_list.size(); ++j)
  {
    if (!track_used[j])
    {
      track_list[j]->second.missing_count += 1;
    }
  }

  return assignments;
}

void ClusterTracking::removeMissingTracks()
{
  for (auto it = tracks_.begin(); it != tracks_.end();)
  {
    // ここでは missing_count が 50 を超えたものを削除
    if (it->second.missing_count > 50)
      it = tracks_.erase(it);
    else
      ++it;
  }
}

std::vector<VoxelCluster> ClusterTracking::filterBySpeed(const std::vector<VoxelCluster> &current_clusters,
                                                         const std::vector<int> &assignments,
                                                         double dt,
                                                         double speed_threshold)
{
  std::vector<VoxelCluster> result;
  for (size_t i = 0; i < current_clusters.size(); ++i)
  {
    int id = assignments[i];
    if (id < 0)
    {
      result.push_back(current_clusters[i]);
      continue;
    }
    auto it = tracks_.find(id);
    if (it == tracks_.end())
    {
      result.push_back(current_clusters[i]);
      continue;
    }

    Point3D last_c = it->second.last_centroid;
    Point3D cur_c = calculateClusterCentroid(current_clusters[i]);
    double dx = cur_c.x - last_c.x;
    double dy = cur_c.y - last_c.y;
    double dz = cur_c.z - last_c.z;
    double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
    double speed = dist / dt;

    if (speed >= speed_threshold)
    {
      result.push_back(current_clusters[i]);
    }
    // トラックの重心を更新
    it->second.last_centroid = cur_c;
  }
  return result;
}

Point3D ClusterTracking::calculateClusterCentroid(const VoxelCluster &cluster)
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
