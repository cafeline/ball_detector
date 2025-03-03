#pragma once

#include <vector>
#include <cstddef>
#include <array>
#include "ball_detector/types/geometry_types.hpp"

struct Voxel
{
  int x, y, z;
  std::vector<Point3D> points; // ボクセル内の点群データ
  int point_count;            // ボクセル内の点群数

  Voxel() : x(0), y(0), z(0), point_count(0) {}
  Voxel(int vx, int vy, int vz) : x(vx), y(vy), z(vz), point_count(0) {}

  void increment()
  {
    point_count++;
  }
};

struct VoxelCluster
{
  std::vector<Voxel> voxels;   // クラスタ内のボクセル
  std::vector<Point3D> points; // クラスタ内の全点群データ
};

// クラスタの種類を表す列挙型
enum class ClusterType
{
  UNKNOWN,        // 不明
  LARGE,          // ボールより大きい
  BALL_CANDIDATE, // ボール候補
  DYNAMIC_BALL    // 動的ボール
};

// クラスタの情報を保持する構造体
struct ClusterInfo
{
  size_t index;
  VoxelCluster cluster;
  ClusterType type = ClusterType::UNKNOWN;
};