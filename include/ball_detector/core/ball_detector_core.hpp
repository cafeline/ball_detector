#ifndef BALL_DETECTOR_BALL_DETECTOR_HPP_
#define BALL_DETECTOR_BALL_DETECTOR_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "ball_detector/types/geometry_types.hpp"
#include "ball_detector/types/clustering_types.hpp"
#include "ball_detector/types/parameters.hpp"
#include "ball_detector/processing/pointcloud_processor.hpp"
#include "ball_detector/cluster/cluster_creator.hpp"
#include "ball_detector/cluster/cluster_classifier.hpp"
#include "ball_detector/visualizer/visualizer.hpp"
#include "ball_detector/tracking/tracking_manager.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace ball_detector
{
  struct DetectionResult
  {
    Point3D ball_position;
    std::vector<ClusterInfo> clusters;
  };

  struct VisualizationData
  {
    visualization_msgs::msg::MarkerArray voxel_markers;
    sensor_msgs::msg::PointCloud2 filtered_cloud;
    visualization_msgs::msg::Marker ball_marker;
    visualization_msgs::msg::Marker trajectory_marker;
    visualization_msgs::msg::Marker past_points_marker;
  };

  class BallDetectorCore
  {
  public:
    BallDetectorCore();
    ~BallDetectorCore() = default;

    void set_params(const Parameters &params);
    DetectionResult detect_ball(const std::vector<Point3D> &processed_points, const rclcpp::Time &current_time, double dt);

    Point3D calculate_ball_position(const std::vector<ClusterInfo> &clusters);

    // 視覚化用データを準備する関数
    VisualizationData prepare_visualization(const DetectionResult &result, const std::vector<Point3D> &processed_points);

    std::unique_ptr<ClusterCreator> cluster_creator_;
    std::unique_ptr<ClusterClassifier> cluster_classifier_;
    std::unique_ptr<TrackingManager> tracking_manager_;
    std::unique_ptr<Visualizer> visualizer_;
    std::unique_ptr<PointCloudProcessor> pointcloud_processor_;

  private:
    Parameters params_;
  };

} // namespace ball_detector

#endif // BALL_DETECTOR_BALL_DETECTOR_HPP_