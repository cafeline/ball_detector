#ifndef BALL_DETECTOR_TRACKING_MANAGER_HPP
#define BALL_DETECTOR_TRACKING_MANAGER_HPP

#include "ball_detector/types.hpp"
#include "ball_detector/tracking_engine.hpp"
#include <vector>
#include <limits>

// トラッキング機能全体を管理するクラス
class TrackingManager
{
public:
  TrackingManager();

  // 動的クラスタの識別
  void identify_dynamic_clusters(
      std::vector<ClusterInfo> &clusters,
      rclcpp::Time current_time,
      double dt,
      const Parameters &params);

  // ボールクラスタの精緻化
  void refine_ball_clusters(
      std::vector<ClusterInfo> &clusters,
      const Point3D &ball_position,
      const Parameters &params);

private:
  TrackingEngine tracking_engine_;

  // 内部ヘルパーメソッド
  void mark_dynamic_clusters(std::vector<ClusterInfo> &clusters,
                             const std::vector<VoxelCluster> &dynamic_clusters);
  size_t find_closest_ball_cluster(const std::vector<ClusterInfo> &clusters,
                                   const Point3D &ball_position) const;
  void update_ball_cluster(std::vector<ClusterInfo> &clusters,
                           size_t best_idx,
                           const Point3D &ball_position);
};

#endif // BALL_DETECTOR_TRACKING_MANAGER_HPP