#ifndef BALL_DETECTOR_TRACKING_ENGINE_HPP
#define BALL_DETECTOR_TRACKING_ENGINE_HPP

#include "ball_detector/types.hpp"
#include <vector>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <limits>

// トラッキングの基本エンジン
class TrackingEngine
{
public:
  TrackingEngine() = default;

  // 新しいフレームのクラスタを既存トラックと関連付ける
  std::vector<int> associateClusters(
      const std::vector<VoxelCluster> &current_clusters,
      double max_distance_for_association,
      rclcpp::Time current_time,
      double dt);

  // 一定期間見えなくなったトラックを除去
  void removeMissingTracks(int max_missing_count = 50);

  // 速度で動的クラスタをフィルタリング
  std::vector<VoxelCluster> filterBySpeed(
      const std::vector<VoxelCluster> &current_clusters,
      const std::vector<int> &assignments,
      double dt,
      double speed_threshold);

  // トラックへのアクセサ
  const std::map<int, ClusterTrack> &getTracks() const { return tracks_; }

private:
  // トラック情報を管理するマップ
  std::map<int, ClusterTrack> tracks_;

  // 内部ヘルパーメソッド
  int createNewTrack(const VoxelCluster &cluster, rclcpp::Time current_time);
  void updateTrack(int track_id, const Point3D &centroid, rclcpp::Time current_time);
};

#endif // BALL_DETECTOR_TRACKING_ENGINE_HPP