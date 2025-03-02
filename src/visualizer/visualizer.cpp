#include "ball_detector/visualizer/visualizer.hpp"

namespace ball_detector
{
  Visualizer::Visualizer(const Parameters &params) : params_(params) {}

  visualization_msgs::msg::MarkerArray Visualizer::create_voxel_cluster_markers(const std::vector<ClusterInfo> &cluster_infos)
  {
    visualization_msgs::msg::MarkerArray marker_array;

    for (const auto &cluster_info : cluster_infos)
    {
      size_t cluster_id = cluster_info.index;

      float r, g, b;
      switch (cluster_info.type)
      {
      case ClusterType::DYNAMIC_BALL:
        r = 0.0f;
        g = 1.0f;
        b = 0.0f; // 動的ボールクラスターは緑
        break;
      case ClusterType::BALL_CANDIDATE:
        r = 1.0f;
        g = 0.0f;
        b = 0.0f; // ボールクラスターは赤
        break;
      case ClusterType::LARGE:
      case ClusterType::UNKNOWN:
      default:
        r = 0.0f;
        g = 0.0f;
        b = 1.0f; // その他は青
        break;
      }

      // ClusterInfo 内の VoxelCluster を利用してマーカーを生成
      const VoxelCluster &cluster = cluster_info.cluster;
      for (size_t v = 0; v < cluster.voxels.size(); ++v)
      {
        const auto &voxel = cluster.voxels[v];
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = params_.frame_id;
        marker.header.stamp = rclcpp::Clock().now();
        marker.ns = "voxel_cluster_markers";
        marker.id = static_cast<int>(cluster_id * 10000 + v);
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

        marker_array.markers.push_back(marker);
      }
    }
    return marker_array;
  }

  visualization_msgs::msg::Marker Visualizer::create_ball_marker(const Point3D &centroid)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = params_.frame_id;
    marker.header.stamp = rclcpp::Clock().now();
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

  void Visualizer::update_trajectory(const Point3D &centroid, const sensor_msgs::msg::PointCloud2 &remaining_cloud)
  {
    // ボールが検出されない場合、軌跡をリセット
    if (centroid.x == 0.0 || centroid.y == 0.0 || centroid.z == 0.0)
    {
      ball_trajectory_points_.clear();
      past_points_.clear();
      return;
    }

    // 設定可能な履歴サイズ（params_から取得するか、動的に調整可能）
    const size_t trajectory_history_size = 10;

    if (ball_trajectory_points_.size() >= trajectory_history_size)
    {
      ball_trajectory_points_.pop_front();
    }
    ball_trajectory_points_.push_back(centroid);

    // ball_trajectory_points_と完全に同期させる
    past_points_ = ball_trajectory_points_;

    visualization_msgs::msg::Marker trajectory_marker = create_trajectory_marker(ball_trajectory_points_);
    visualization_msgs::msg::Marker past_points_marker = create_past_points_marker(past_points_);
  }

  visualization_msgs::msg::Marker Visualizer::create_trajectory_marker(const std::deque<Point3D> &trajectory)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = params_.frame_id;
    marker.header.stamp = rclcpp::Clock().now();
    marker.ns = "ball_trajectory";
    marker.id = 1;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.scale.x = 0.05;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 188.0 / 255.0;
    marker.color.a = 0.3;
    marker.lifetime = rclcpp::Duration(0, 0);

    for (const auto &point : trajectory)
    {
      geometry_msgs::msg::Point geom_point;
      geom_point.x = point.x;
      geom_point.y = point.y;
      geom_point.z = point.z;
      marker.points.push_back(geom_point);
    }

    return marker;
  }

  visualization_msgs::msg::Marker Visualizer::create_past_points_marker(const std::deque<Point3D> &past_points)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = params_.frame_id;
    marker.header.stamp = rclcpp::Clock().now();
    marker.ns = "past_ball_points";
    marker.id = 2;
    marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 0.3;
    marker.lifetime = rclcpp::Duration(0, 0);

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
} // namespace ball_detector