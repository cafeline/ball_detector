#include "ball_detector/marker_factory.hpp"

namespace ball_detector
{

  visualization_msgs::msg::Marker MarkerFactory::create_ball_marker(const Point3D &centroid, const std_msgs::msg::Header &header)
  {
    visualization_msgs::msg::Marker marker;
    marker.header = header;
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = centroid.x;
    marker.pose.position.y = centroid.y;
    marker.pose.position.z = centroid.z;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    marker.lifetime = rclcpp::Duration(0, 1e8);
    return marker;
  }

  visualization_msgs::msg::Marker MarkerFactory::create_past_points_marker(const std::deque<Point3D> &past_points, const std_msgs::msg::Header &header)
  {
    visualization_msgs::msg::Marker marker;
    marker.header = header;
    marker.ns = "past_ball_points";
    marker.id = 2;
    marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // 各点のスケール
    marker.scale.x = 0.05; // ボールの点の半径
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;

    // ボールの点の色
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0; // 緑色
    marker.color.a = 0.3; // 完全不透明

    // ライフタイムを0に設定して永続的に表示
    marker.lifetime = rclcpp::Duration(0, 0);

    // 過去の検出点を追加（最大10点）
    for (const auto &point : past_points)
    {
      geometry_msgs::msg::Point geom_point;
      geom_point.x = point.x;
      geom_point.y = point.y;
      geom_point.z = point.z;
      marker.points.push_back(geom_point);
    }

    return marker;
  }

  visualization_msgs::msg::MarkerArray MarkerFactory::create_voxel_cluster_markers(
      const std::vector<VoxelCluster> &clusters,
      const std::unordered_set<size_t> &ballIndices,
      const std::unordered_set<size_t> &dynamicIndices,
      const std::unordered_set<size_t> &dynamicBallIndices)
  {

    visualization_msgs::msg::MarkerArray markerArray;
    for (size_t i = 0; i < clusters.size(); ++i)
    {
      bool isBallCluster = (ballIndices.find(i) != ballIndices.end());
      bool isDynamic = (dynamicIndices.find(i) != dynamicIndices.end());
      bool isDynamicBall = (dynamicBallIndices.find(i) != dynamicBallIndices.end());
      float r, g, b;
      if (isDynamicBall)
      {
        r = 0.0f;
        g = 1.0f;
        b = 0.0f;
      }
      else if (isBallCluster)
      {
        r = 1.0f;
        g = 0.0f;
        b = 0.0f;
      }
      else
      {
        r = 0.0f;
        g = 0.0f;
        b = 1.0f;
      }

      for (size_t v = 0; v < clusters[i].voxels.size(); ++v)
      {
        const auto &voxel = clusters[i].voxels[v];
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.ns = "voxel_cluster_markers";
        marker.id = static_cast<int>(i * 10000 + v);
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        double offset_x = params_.voxel_size_x / 2.0;
        double offset_y = params_.voxel_size_y / 2.0;
        double offset_z = params_.voxel_size_z / 2.0;
        marker.pose.position.x = params_.min_x + (voxel.x * params_.voxel_size_x) + offset_x;
        marker.pose.position.y = params_.min_y + (voxel.y * params_.voxel_size_y) + offset_y;
        marker.pose.position.z = params_.min_z + (voxel.z * params_.voxel_size_z) + offset_z;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = params_.voxel_size_x;
        marker.scale.y = params_.voxel_size_y;
        marker.scale.z = params_.voxel_size_z;
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = 0.3;
        marker.lifetime = rclcpp::Duration(0, 1e8);
        markerArray.markers.push_back(marker);
      }
    }
    return markerArray;
  }

} // namespace ball_detector
