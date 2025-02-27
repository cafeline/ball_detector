#ifndef BALL_DETECTOR_BALL_DETECTOR_HPP_
#define BALL_DETECTOR_BALL_DETECTOR_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "ball_detector/pointcloud_processor.hpp"
#include "ball_detector/clustering.hpp"

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

    std::unique_ptr<Clustering> clustering_;
    std::unique_ptr<VoxelProcessor> voxel_processor_;

  private:
    Parameters params_;
  };

} // namespace ball_detector

#endif // BALL_DETECTOR_BALL_DETECTOR_HPP_