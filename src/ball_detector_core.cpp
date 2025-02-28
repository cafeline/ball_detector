#include "ball_detector/ball_detector_core.hpp"
#include <chrono>
#include <unordered_set>
#include <random>
#include <cmath>

namespace ball_detector
{
  BallDetectorCore::BallDetectorCore()
  {
    voxel_processor_ = std::make_unique<VoxelProcessor>(params_);
    clustering_ = std::make_unique<Clustering>(params_);
    visualizer_ = std::make_unique<Visualizer>(params_);
  }

  // パラメータを後から設定するためのセッター
  void BallDetectorCore::set_params(const Parameters &params)
  {
    params_ = params;
    voxel_processor_ = std::make_unique<VoxelProcessor>(params_);
    clustering_ = std::make_unique<Clustering>(params_);
    visualizer_ = std::make_unique<Visualizer>(params_);
  }

  DetectionResult BallDetectorCore::detect_ball(const std::vector<Point3D> &processed_points, const rclcpp::Time &current_time, double dt)
  {
    // ボクセル化
    std::vector<Voxel> voxels = voxel_processor_->create_voxel(processed_points);

    // クラスタリング: 戻り値は ClusterInfo の vector となる
    std::vector<ClusterInfo> clusters = clustering_->create_voxel_clustering(processed_points, voxels);

    // クラスタの処理
    clustering_->process_clusters(processed_points, clusters, current_time, dt);

    // ボール位置の計算
    Point3D ball_position = calculate_ball_position(clusters);

    // ボールクラスタの精緻化
    clustering_->refine_ball_clusters(clusters, ball_position);

    visualizer_->clustering_ = std::make_unique<Clustering>(*clustering_);

    // 検出結果とビジュアライゼーションデータを返す
    return DetectionResult{ball_position, clusters, processed_points};
  }

  Point3D BallDetectorCore::calculate_ball_position(const std::vector<ClusterInfo> &clusters)
  {
    const auto &dynamic_ball_indices = clustering_->get_dynamic_ball_cluster_indices();
    std::vector<Point3D> candidate_points;

    // 動的かつボール以下のサイズのクラスタから全ての点を収集
    for (const auto &index : dynamic_ball_indices)
    {
      // ClusterInfo の中の cluster が VoxelCluster となっているので、点集合は cluster.points としてアクセス
      const ClusterInfo &cluster = clusters[index];
      candidate_points.insert(candidate_points.end(), cluster.cluster.points.begin(), cluster.cluster.points.end());
    }

    if (candidate_points.size() < 2)
    {
      // RCLCPP_WARN(this->get_logger(), "候補点が2点未満のため、ボールを検出できません。");
      return Point3D{0.0, 0.0, 0.0}; // 無効な位置を示す
    }

    // x座標でソート
    std::sort(candidate_points.begin(), candidate_points.end(),
              [](const Point3D &a, const Point3D &b)
              { return a.x < b.x; });

    // 最小のxを持つ2点を選択し平均値を計算
    const Point3D &point1 = candidate_points[0];
    const Point3D &point2 = candidate_points[1];

    return Point3D{(point1.x + point2.x) / 2.0, (point1.y + point2.y) / 2.0, (point1.z + point2.z) / 2.0};
  }

}
