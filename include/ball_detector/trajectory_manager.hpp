#ifndef BALL_DETECTOR_TRAJECTORY_MANAGER_HPP_
#define BALL_DETECTOR_TRAJECTORY_MANAGER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "ball_detector/types.hpp"
#include <deque>
#include <std_msgs/msg/header.hpp>

namespace ball_detector
{

  class TrajectoryManager
  {
  public:
    explicit TrajectoryManager(size_t maxPoints = 10) : maxPoints_(maxPoints) {}

    void update_trajectory(const Point3D &newPoint);
    const std::deque<Point3D> &get_trajectory() const;
    visualization_msgs::msg::Marker create_trajectory_marker(const std_msgs::msg::Header &header) const;

  private:
    size_t maxPoints_;
    std::deque<Point3D> trajectoryPoints_;
  };

} // namespace ball_detector

#endif // BALL_DETECTOR_TRAJECTORY_MANAGER_HPP_
