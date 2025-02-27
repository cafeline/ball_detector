#ifndef BALL_DETECTOR_CLUSTER_ANALYZER_HPP_
#define BALL_DETECTOR_CLUSTER_ANALYZER_HPP_

#include "ball_detector/types.hpp"
#include <vector>

namespace ball_detector
{

  class ClusterAnalyzer
  {
  public:
    explicit ClusterAnalyzer(const Parameters &params) : params_(params) {}

    // クラスタ内の点群から各次元のサイズを計算する
    void calculate_cluster_size(const VoxelCluster &cluster, const std::vector<Point3D> &points,
                                double &size_x, double &size_y, double &size_z) const;
    // クラスタの重心を計算する
    Point3D compute_centroid(const VoxelCluster &cluster) const;
    // ボールクラスタの精緻化処理（例：最も近いクラスタのみに更新）
    void refine_ball_clusters(std::vector<VoxelCluster> &clusters, const Point3D &ball_position) const;

  private:
    Parameters params_;
  };

} // namespace ball_detector

#endif // BALL_DETECTOR_CLUSTER_ANALYZER_HPP_
