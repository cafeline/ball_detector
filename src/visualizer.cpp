#include "ball_detector/visualizer.hpp"

namespace ball_detector
{
  Visualizer::Visualizer(const Parameters &params) : params_(params) {}

  visualization_msgs::msg::MarkerArray Visualizer::create_voxel_cluster_markers(const std::vector<ClusterInfo> &cluster_infos)
  {
    visualization_msgs::msg::MarkerArray marker_array;

    // 各 ClusterInfo を直接処理：Clustering から索引集合を取得せず、ClusterInfo の要素から情報を得る
    for (const auto &cluster_info : cluster_infos)
    {
      // ClusterInfo の index メンバーを利用
      size_t cluster_id = cluster_info.index;
      // ClusterInfo 内にフラグを持たせ、そこから各種判定を行う
      bool is_ball_cluster = cluster_info.is_ball_cluster;
      bool is_dynamic_ball = cluster_info.is_dynamic_ball;
      float r, g, b;
      if (is_dynamic_ball)
      {
        r = 0.0f;
        g = 1.0f;
        b = 0.0f; // 動的ボールクラスターは緑
      }
      else if (is_ball_cluster)
      {
        r = 1.0f;
        g = 0.0f;
        b = 0.0f; // ボールクラスターは赤
      }
      else
      {
        r = 0.0f;
        g = 0.0f;
        b = 1.0f; // その他は青
      }

      // ClusterInfo 内の VoxelCluster を利用してマーカーを生成
      const VoxelCluster &cluster = cluster_info.cluster;
      for (size_t v = 0; v < cluster.voxels.size(); ++v)
      {
        const auto &voxel = cluster.voxels[v];
        visualization_msgs::msg::Marker marker;
        // ヘッダーは "map" としていますが、必要に応じて修正してください
        marker.header.frame_id = "map";
        marker.ns = "voxel_cluster_markers";
        // 一意のIDとして、cluster_id と voxel のインデックスを組み合わせる
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

  visualization_msgs::msg::Marker Visualizer::create_ball_marker(const Point3D &centroid, const std_msgs::msg::Header &header)
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

  void Visualizer::update_trajectory(const Point3D &centroid, const sensor_msgs::msg::PointCloud2 &remaining_cloud)
  {
    if (centroid.x == 0.0 || centroid.y == 0.0 || centroid.z == 0.0)
    {
      return;
    }

    const size_t MAX_PAST_POINTS = 10;

    if (ball_trajectory_points_.size() >= MAX_PAST_POINTS)
    {
      ball_trajectory_points_.pop_front();
    }
    ball_trajectory_points_.push_back(centroid);

    if (past_points_.size() >= MAX_PAST_POINTS)
    {
      past_points_.pop_front();
    }
    past_points_.push_back(centroid);

    visualization_msgs::msg::Marker trajectory_marker = create_trajectory_marker(ball_trajectory_points_, remaining_cloud.header);
    visualization_msgs::msg::Marker past_points_marker = create_past_points_marker(past_points_, remaining_cloud.header);
    // 必要に応じて、各 Marker をパブリッシャーで発行してください
  }

  visualization_msgs::msg::Marker Visualizer::create_trajectory_marker(const std::deque<Point3D> &trajectory, const std_msgs::msg::Header &header)
  {
    visualization_msgs::msg::Marker marker;
    marker.header = header;
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

  visualization_msgs::msg::Marker Visualizer::create_past_points_marker(const std::deque<Point3D> &past_points, const std_msgs::msg::Header &header)
  {
    visualization_msgs::msg::Marker marker;
    marker.header = header;
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