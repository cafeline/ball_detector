#pragma once

#include <vector>
#include <string>
#include <queue>
#include "pointcloud_processor/types.hpp"

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

  // ボクセルをクラスタリング
  std::vector<VoxelCluster> create_voxel_clustering(const std::vector<Point3D> &points, const std::vector<Voxel> &voxels);

  void collect_cluster_points(VoxelCluster &cluster, const std::vector<Point3D> &points);

private:
  Parameters params_;

  // 隣接ボクセルのキーを取得
  std::vector<std::string> get_adjacent_voxels(const std::string &key) const;

  // クラスタの有効性を評価
  bool is_valid_cluster(const VoxelCluster &cluster, const std::vector<Point3D> &points) const;

  // クラスタのサイズを計算
  void calculate_cluster_size(const VoxelCluster &cluster, const std::vector<Point3D> &points, double &size_x, double &size_y, double &size_z) const;

  // 点がボクセル内に属するかをチェック
  bool point_in_voxel(const Point3D &point, const Voxel &voxel) const;
};
