#ifndef BALL_DETECTOR_BALL_DETECTOR_HPP_
#define BALL_DETECTOR_BALL_DETECTOR_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "ball_detector/pointcloud_processor.hpp"
#include "ball_detector/clustering.hpp"
#include "ball_detector/visualizer.hpp"

namespace ball_detector
{
  // 検出結果とビジュアライゼーションデータを格納する構造体
  struct DetectionResult
  {
    Point3D ball_position;
    std::vector<ClusterInfo> clusters;
    std::vector<Point3D> processed_points;
  };

  class BallDetectorCore
  {
  public:
    BallDetectorCore();
    ~BallDetectorCore() = default;

    void set_params(const Parameters &params);
    DetectionResult detect_ball(const std::vector<Point3D> &processed_points, const rclcpp::Time &current_time, double dt);

    Point3D calculate_ball_position(const std::vector<ClusterInfo> &clusters);

    // 新規追加: ClusterInfoのvectorからVoxelClusterを抽出し、Visualizerの対応する関数を呼び出す
    visualization_msgs::msg::MarkerArray create_voxel_cluster_markers(const std::vector<ClusterInfo> &clusters);

    std::unique_ptr<Clustering> clustering_;
    std::unique_ptr<Visualizer> visualizer_;

  private:
    Parameters params_;
  };

} // namespace ball_detector

#endif // BALL_DETECTOR_BALL_DETECTOR_HPP_