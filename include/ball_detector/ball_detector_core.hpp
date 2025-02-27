#ifndef BALL_DETECTOR_BALL_DETECTOR_HPP_
#define BALL_DETECTOR_BALL_DETECTOR_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "ball_detector/pointcloud_processor.hpp"
#include "ball_detector/clustering.hpp"
#include <deque>

namespace ball_detector
{
  class BallDetector
  {
  public:
    BallDetector();
    ~BallDetector() = default;

    void set_params(const Parameters &params);
    Point3D detect_ball(const std::vector<Point3D> &processed_points, const rclcpp::Time &current_time, double dt);

    Point3D calculate_ball_position(const std::vector<VoxelCluster> &clusters);
    visualization_msgs::msg::MarkerArray create_voxel_cluster_markers(const std::vector<VoxelCluster> &clusters);
    visualization_msgs::msg::Marker create_ball_marker(const Point3D &centroid, const std_msgs::msg::Header &header);
    void update_trajectory(const Point3D &centroid, const sensor_msgs::msg::PointCloud2 &remaining_cloud);
    visualization_msgs::msg::Marker create_trajectory_marker(const std::deque<Point3D> &trajectory, const std_msgs::msg::Header &header);
    visualization_msgs::msg::Marker create_past_points_marker(const std::deque<Point3D> &past_points, const std_msgs::msg::Header &header);

    std::unique_ptr<Clustering> clustering_;
    std::unique_ptr<VoxelProcessor> voxel_processor_;
    std::deque<Point3D> ball_trajectory_points_;
    std::deque<Point3D> past_points_;

  private:
    Parameters params_;
  };

} // namespace ball_detector

#endif // BALL_DETECTOR_BALL_DETECTOR_HPP_