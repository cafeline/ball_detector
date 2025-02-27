#include "ball_detector/ball_detector_core.hpp"
#include <algorithm>
#include <cmath>

namespace ball_detector
{

  BallDetectorCore::BallDetectorCore() : params_()
  {
    voxel_processor_ = std::make_unique<VoxelProcessor>(params_);
    clusterer_ = std::make_unique<VoxelClusterer>(params_);
    analyzer_ = std::make_unique<ClusterAnalyzer>(params_);
    marker_factory_ = std::make_unique<MarkerFactory>(params_);
    trajectory_manager_ = std::make_unique<TrajectoryManager>();
    cluster_tracking_ = std::make_unique<ClusterTracking>();
  }

  void BallDetectorCore::set_params(const Parameters &params)
  {
    params_ = params;
    voxel_processor_ = std::make_unique<VoxelProcessor>(params_);
    clusterer_ = std::make_unique<VoxelClusterer>(params_);
    analyzer_ = std::make_unique<ClusterAnalyzer>(params_);
    marker_factory_ = std::make_unique<MarkerFactory>(params_);
    // trajectory_manager_ は内部状態を保持するため再生成は不要な場合もある
  }

  DetectionResult BallDetectorCore::detect_ball(const std::vector<Point3D> &processed_points,
                                                const rclcpp::Time &current_time,
                                                double dt)
  {
    // 1. 点群からボクセルを作成
    std::vector<Voxel> voxels = voxel_processor_->create_voxel(processed_points);

    // 2. VoxelClusterer によりクラスタを生成
    std::vector<VoxelCluster> clusters = clusterer_->create_clusters(processed_points, voxels);

    // 3. ボール位置の計算（例として各クラスタの重心を利用）
    Point3D ball_position = calculate_ball_position(clusters);

    // 4. ClusterAnalyzer によるクラスタの精緻化
    analyzer_->refine_ball_clusters(clusters, ball_position);

    // 5. 軌跡更新
    trajectory_manager_->update_trajectory(ball_position);

    // 以下を追加
    std::vector<int> assignments = cluster_tracking_->associate_clusters(
        clusters, params_.max_distance_for_association, current_time, dt);

    cluster_tracking_->remove_missing_tracks();

    std::vector<VoxelCluster> filtered_clusters =
        cluster_tracking_->filter_by_speed(clusters, assignments, dt, params_.ball_vel_min);

    std::unordered_set<size_t> ball_indices;
    std::unordered_set<size_t> dynamic_indices;
    std::unordered_set<size_t> dynamic_ball_indices;

    // インデックスを計算
    for (size_t j = 0; j < filtered_clusters.size(); ++j)
    {
      int i = assignments[j];
      if (i < 0)
        continue;
      dynamic_ball_indices.insert(i);
    }

    for (size_t i = 0; i < clusters.size(); ++i)
    {
      if (dynamic_ball_indices.find(i) == dynamic_ball_indices.end())
      {
        dynamic_indices.insert(i);
      }
    }

    return DetectionResult{ball_position, clusters, processed_points, ball_indices, dynamic_indices, dynamic_ball_indices};
  }

  Point3D BallDetectorCore::calculate_ball_position(const std::vector<VoxelCluster> &clusters)
  {
    std::vector<Point3D> candidatePoints;
    for (const auto &cluster : clusters)
    {
      candidatePoints.push_back(analyzer_->compute_centroid(cluster));
    }
    if (candidatePoints.size() < 2)
      return Point3D{0.0, 0.0, 0.0};

    std::sort(candidatePoints.begin(), candidatePoints.end(),
              [](const Point3D &a, const Point3D &b)
              { return a.x < b.x; });
    const Point3D &p1 = candidatePoints[0];
    const Point3D &p2 = candidatePoints[1];
    return Point3D{(p1.x + p2.x) / 2.0, (p1.y + p2.y) / 2.0, (p1.z + p2.z) / 2.0};
  }

} // namespace ball_detector
