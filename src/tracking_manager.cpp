#include "ball_detector/tracking_manager.hpp"
#include "ball_detector/tracking_utils.hpp"

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