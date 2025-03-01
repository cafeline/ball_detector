#ifndef BALL_DETECTOR_TRACKING_UTILS_HPP
#define BALL_DETECTOR_TRACKING_UTILS_HPP

#include "ball_detector/geometry_types.hpp"
#include "ball_detector/clustering_types.hpp"
#include <cmath>

// 座標計算などの共通ユーティリティ
namespace tracking_utils
{
  // クラスタの重心を計算
  Point3D calculateClusterCentroid(const VoxelCluster &cluster);

  // 2点間の距離を計算
  double calculateDistance(const Point3D &p1, const Point3D &p2);

  // 位置が原点に近いかチェック
  bool is_zero_position(const Point3D &position, double epsilon = 1e-9);

  // 2つのクラスタが一致するかチェック
  bool clusters_match(const VoxelCluster &a, const VoxelCluster &b);
}

#endif // BALL_DETECTOR_TRACKING_UTILS_HPP