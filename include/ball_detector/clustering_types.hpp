#pragma once

#include <vector>
#include <cstddef>
#include <array>
#include "ball_detector/geometry_types.hpp"

// ボクセル構造体
struct Voxel
{
  int x, y, z;                 // ボクセルのインデックス
  int point_count;             // ボクセル内の点群数
  std::vector<Point3D> points; // ボクセル内の点群データ
  std::array<float, 4> color;  // ボクセルの色 (RGBA)

  // デフォルトコンストラクタ
  Voxel() : x(0), y(0), z(0), point_count(0), color{1.0f, 1.0f, 1.0f, 1.0f} {}

  // パラメータ付きコンストラクタ
  Voxel(int vx, int vy, int vz) : x(vx), y(vy), z(vz), point_count(1), color{1.0f, 1.0f, 1.0f, 1.0f} {}

  // 点群数をインクリメント
  void increment()
  {
    point_count++;
  }

  // 点を追加
  void add_point(const Point3D &point)
  {
    points.push_back(point);
    increment();
  }
};

// ボクセルクラスタ構造体
struct VoxelCluster
{
  std::vector<Voxel> voxels;   // クラスタ内のボクセル
  int total_point_count;       // クラスタ内の総点群数
  std::vector<Point3D> points; // クラスタ内の全点群データ
};

// クラスタの種類を表す列挙型
enum class ClusterType
{
  UNKNOWN,        // 不明なクラスタ
  LARGE,          // ボールより大きい
  BALL_CANDIDATE, // ボール候補クラスタ
  DYNAMIC_BALL    // 動的ボールクラスタ
};

// クラスタの情報（クラスタ本体とその一意なインデックス）を保持する構造体
struct ClusterInfo
{
  size_t index;
  VoxelCluster cluster;
  ClusterType type = ClusterType::UNKNOWN;
};