#include "ball_detector/trajectory_manager.hpp"
#include "geometry_msgs/msg/point.hpp"

namespace ball_detector
{

  void TrajectoryManager::update_trajectory(const Point3D &newPoint)
  {
    if (trajectoryPoints_.size() >= maxPoints_)
    {
      trajectoryPoints_.pop_front();
    }
    trajectoryPoints_.push_back(newPoint);
  }

  const std::deque<Point3D> &TrajectoryManager::get_trajectory() const
  {
    return trajectoryPoints_;
  }

  visualization_msgs::msg::Marker TrajectoryManager::create_trajectory_marker(const std_msgs::msg::Header &header) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header = header;
    marker.ns = "ball_trajectory";
    marker.id = 1;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.05; // 線の太さ
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 188.0 / 255.0;
    marker.color.a = 0.3;
    marker.lifetime = rclcpp::Duration(0, 0);
    for (const auto &pt : trajectoryPoints_)
    {
      geometry_msgs::msg::Point geom_point;
      geom_point.x = pt.x;
      geom_point.y = pt.y;
      geom_point.z = pt.z;
      marker.points.push_back(geom_point);
    }
    return marker;
  }

} // namespace ball_detector
