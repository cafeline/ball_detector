#ifndef BALL_DETECTOR_VISUALIZER_HPP_
#define BALL_DETECTOR_VISUALIZER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "ball_detector/geometry_types.hpp"
#include "ball_detector/clustering_types.hpp"
#include "ball_detector/parameters.hpp"
#include <deque>
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace ball_detector
{
  class Visualizer
  {
  public:
    Visualizer(const Parameters &params);
    ~Visualizer() = default;

    // 引数は std::vector<ClusterInfo> のみ
    visualization_msgs::msg::MarkerArray create_voxel_cluster_markers(const std::vector<ClusterInfo> &cluster_infos);
    visualization_msgs::msg::Marker create_ball_marker(const Point3D &centroid, const std_msgs::msg::Header &header);
    void update_trajectory(const Point3D &centroid, const sensor_msgs::msg::PointCloud2 &remaining_cloud);
    visualization_msgs::msg::Marker create_trajectory_marker(const std::deque<Point3D> &trajectory, const std_msgs::msg::Header &header);
    visualization_msgs::msg::Marker create_past_points_marker(const std::deque<Point3D> &past_points, const std_msgs::msg::Header &header);

    std::deque<Point3D> ball_trajectory_points_;
    std::deque<Point3D> past_points_;

  private:
    Parameters params_;
  };
} // namespace ball_detector

#endif // BALL_DETECTOR_VISUALIZER_HPP_