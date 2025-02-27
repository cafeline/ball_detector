#ifndef BALL_DETECTOR_BALL_DETECTOR_CORE_HPP_
#define BALL_DETECTOR_BALL_DETECTOR_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "ball_detector/types.hpp"
#include "ball_detector/pointcloud_processor.hpp"
#include "ball_detector/voxel_clusterer.hpp"
#include "ball_detector/cluster_analyzer.hpp"
#include "ball_detector/marker_factory.hpp"
#include "ball_detector/trajectory_manager.hpp"
#include <memory>
#include <vector>

namespace ball_detector
{

  struct DetectionResult
  {
    Point3D ball_position;
    std::vector<VoxelCluster> clusters;
    std::vector<Point3D> processed_points;
  };

  class BallDetectorCore
  {
  public:
    BallDetectorCore();
    ~BallDetectorCore() = default;

    void set_params(const Parameters &params);
    DetectionResult detect_ball(const std::vector<Point3D> &processed_points, const rclcpp::Time &current_time, double dt);
    std::unique_ptr<MarkerFactory> marker_factory_;
    std::unique_ptr<VoxelProcessor> voxel_processor_;
    std::unique_ptr<VoxelClusterer> clusterer_;
    std::unique_ptr<ClusterAnalyzer> analyzer_;
    std::unique_ptr<TrajectoryManager> trajectory_manager_;

  private:
    Point3D calculate_ball_position(const std::vector<VoxelCluster> &clusters);

    Parameters params_;

  };

} // namespace ball_detector

#endif // BALL_DETECTOR_BALL_DETECTOR_CORE_HPP_
