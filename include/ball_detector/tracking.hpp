#ifndef BALL_DETECTOR_TRACKING_HPP
#define BALL_DETECTOR_TRACKING_HPP

#include "ball_detector/types.hpp"
#include <vector>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <limits>
#include <cmath>

// 座標計算などの共通ユーティリティ
namespace tracking_utils
{
  Point3D calculateClusterCentroid(const VoxelCluster &cluster);
  double calculateDistance(const Point3D &p1, const Point3D &p2);
  bool is_zero_position(const Point3D &position, double epsilon = 1e-9);
  bool clusters_match(const VoxelCluster &a, const VoxelCluster &b);
}

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

#endif // BALL_DETECTOR_TRACKING_HPP