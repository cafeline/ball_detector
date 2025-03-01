#include "ball_detector/ball_detector_core.hpp"
#include <chrono>
#include <unordered_set>
#include <random>
#include <cmath>

namespace ball_detector
{
  BallDetectorCore::BallDetectorCore()
  {
    // ClusterManagerの代わりに個別にCreatorとClassifierを初期化
    cluster_creator_ = std::make_unique<ClusterCreator>(params_);
    cluster_classifier_ = std::make_unique<ClusterClassifier>(params_);
    tracking_manager_ = std::make_unique<TrackingManager>();
    visualizer_ = std::make_unique<Visualizer>(params_);
  }

  void BallDetectorCore::set_params(const Parameters &params)
  {
    params_ = params;
    // ClusterManagerの代わりに個別にCreatorとClassifierを初期化
    cluster_creator_ = std::make_unique<ClusterCreator>(params_);
    cluster_classifier_ = std::make_unique<ClusterClassifier>(params_);
    tracking_manager_ = std::make_unique<TrackingManager>();
    visualizer_ = std::make_unique<Visualizer>(params_);
    pointcloud_processor = std::make_unique<PointCloudProcessor>(params_);
  }

  DetectionResult BallDetectorCore::detect_ball(const std::vector<Point3D> &processed_points, const rclcpp::Time &current_time, double dt)
  {
    // クラスタリング（ClusterCreatorを直接使用）
    std::vector<ClusterInfo> clusters = cluster_creator_->create_voxel_clustering(processed_points);

    // クラスタの処理（クラス分類）（ClusterClassifierを直接使用）
    cluster_classifier_->identify_ball_candidates(clusters);

    // 動的クラスタの識別（トラッキング処理）
    tracking_manager_->identify_dynamic_clusters(clusters, current_time, dt, params_);

    // 境界付近のクラスタをフィルタリング（ClusterClassifierを直接使用）
    cluster_classifier_->filter_clusters_near_boundaries(clusters);

    // ボール位置の計算
    Point3D ball_position = calculate_ball_position(clusters);

    // ボールクラスタの精緻化
    tracking_manager_->refine_ball_clusters(clusters, ball_position, params_);

    return DetectionResult{ball_position, clusters};
  }

  Point3D BallDetectorCore::calculate_ball_position(const std::vector<ClusterInfo> &clusters)
  {
    std::vector<Point3D> candidate_points;

    // 動的ボールクラスタから点を収集
    for (const auto &cluster_info : clusters)
    {
      if (cluster_info.type == ClusterType::DYNAMIC_BALL)
      {
        candidate_points.insert(candidate_points.end(),
                                cluster_info.cluster.points.begin(),
                                cluster_info.cluster.points.end());
      }
    }

    if (candidate_points.size() < 2)
    {
      // 候補点が2点未満の場合は無効な位置を返す
      return Point3D{0.0f, 0.0f, 0.0f};
    }

    // x座標でソート
    std::sort(candidate_points.begin(), candidate_points.end(),
              [](const Point3D &a, const Point3D &b)
              { return a.x < b.x; });

    const Point3D &point1 = candidate_points[0];
    const Point3D &point2 = candidate_points[1];

    return Point3D{
        static_cast<float>((point1.x + point2.x) / 2.0),
        static_cast<float>((point1.y + point2.y) / 2.0),
        static_cast<float>((point1.z + point2.z) / 2.0)};
  }

}
