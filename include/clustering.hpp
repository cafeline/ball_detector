#pragma once

#include <vector>
#include <string>
#include <queue>
#include "types.hpp"

// クラスタリング処理に関連する関数と構造体を定義するヘッダーファイル
class VoxelProcessor
{
public:
  VoxelProcessor(const Parameters &params);

  // ボクセルを作成する関数
  std::vector<Voxel> create_voxel(const std::vector<Point3D> &points);

private:
  Parameters params_;
};

class Clustering
{
public:
  Clustering(const Parameters &params);

  // ボクセルクラスタリングを作成する関数
  std::vector<VoxelCluster> create_voxel_clustering(const std::vector<Point3D> &points, const std::vector<Voxel> &voxels);

  // 新規追加: collect_cluster_points を public に変更
  void collect_cluster_points(VoxelCluster &cluster, const std::vector<Point3D> &points);

private:
  Parameters params_;

  // 隣接ボクセルのキーを取得するヘルパー関数
  std::vector<std::string> get_adjacent_voxels(const std::string &key) const;

  // クラスタの有効性を評価するヘルパー関数
  bool is_valid_cluster(const VoxelCluster &cluster, const std::vector<Point3D> &points) const;

  // クラスタのサイズを計算するヘルパー関数
  void calculate_cluster_size(const VoxelCluster &cluster, const std::vector<Point3D> &points, double &size_x, double &size_y, double &size_z) const;

  // 点がボクセル内に属するかをチェックするヘルパー関数
  bool point_in_voxel(const Point3D &point, const Voxel &voxel) const;
};
