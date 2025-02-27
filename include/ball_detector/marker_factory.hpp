#ifndef BALL_DETECTOR_MARKER_FACTORY_HPP_
#define BALL_DETECTOR_MARKER_FACTORY_HPP_

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "ball_detector/types.hpp"
#include "ball_detector/voxel_clusterer.hpp"
#include <unordered_set>
#include <vector>

namespace ball_detector
{

  class MarkerFactory
  {
  public:
    explicit MarkerFactory(const Parameters &params) : params_(params) {}
    visualization_msgs::msg::Marker create_ball_marker(const Point3D &centroid, const std_msgs::msg::Header &header);
    visualization_msgs::msg::Marker create_past_points_marker(const std::deque<Point3D> &past_points, const std_msgs::msg::Header &header);

    visualization_msgs::msg::MarkerArray create_voxel_cluster_markers(
        const std::vector<VoxelCluster> &clusters,
        const std::unordered_set<size_t> &ballIndices,
        const std::unordered_set<size_t> &dynamicIndices,
        const std::unordered_set<size_t> &dynamicBallIndices);

  private:
    Parameters params_;
  };

} // namespace ball_detector

#endif // BALL_DETECTOR_MARKER_FACTORY_HPP_
