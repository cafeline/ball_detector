#include "ball_detector/ball_detector.hpp"
#include <chrono>
#include <unordered_set>
#include <random>
#include <cmath>

namespace ball_detector
{
  BallDetector::BallDetector(const rclcpp::NodeOptions &options) : BallDetector("", options) {}

  BallDetector::BallDetector(const std::string &name_space, const rclcpp::NodeOptions &options)
      : rclcpp::Node("ball_detector", name_space, options)
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/livox/lidar", 10, std::bind(&BallDetector::pointcloud_callback, this, std::placeholders::_1));
    pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/self_pose", 10, std::bind(&BallDetector::pose_callback, this, std::placeholders::_1));
    autonomous_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
        "autonomous", 10, std::bind(&BallDetector::autonomous_callback, this, std::placeholders::_1));

    ball_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("tennis_ball", 10);
    filtered_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("ball_detector_pointcloud", 10);
    trajectory_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("ball_trajectory", 10);
    past_points_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("past_ball_points", 10);
    clustered_voxel_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("clustered_voxel", 10);

    load_parameters();

    voxel_processor_ = std::make_unique<VoxelProcessor>(params_);
    clustering_ = std::make_unique<Clustering>(params_);
  }

  void BallDetector::load_parameters()
  {
    params_.min_x = this->get_parameter("min_x").as_double();
    params_.max_x = this->get_parameter("max_x").as_double();
    params_.min_y = this->get_parameter("min_y").as_double();
    params_.max_y = this->get_parameter("max_y").as_double();
    params_.min_z = this->get_parameter("min_z").as_double();
    params_.max_z = this->get_parameter("max_z").as_double();
    params_.livox_pitch = this->get_parameter("livox_pitch").as_double();
    params_.voxel_size_x = this->get_parameter("voxel_size_x").as_double();
    params_.voxel_size_y = this->get_parameter("voxel_size_y").as_double();
    params_.voxel_size_z = this->get_parameter("voxel_size_z").as_double();
    params_.voxel_search_range = this->get_parameter("voxel_search_range").as_int();
    params_.ball_radius = this->get_parameter("ball_radius").as_double();
    params_.ball_vel_min = this->get_parameter("ball_vel_min").as_double();
    params_.max_distance_for_association = this->get_parameter("max_distance_for_association").as_double();
  }

  void BallDetector::pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // if (!is_autonomous)
    //   return;
    auto start = std::chrono::high_resolution_clock::now();
    rclcpp::Time current_time = this->now();
    double dt = (previous_time_.nanoseconds() == 0) ? 0.0 : (current_time - previous_time_).seconds();
    previous_time_ = current_time;

    PointCloudProcessor processor(params_);
    std::vector<Point3D> processed_points = processor.process(msg, self_pose_.x, self_pose_.y, self_pose_.z);
    // ボクセル化 & クラスタリング
    std::vector<Voxel> voxels = voxel_processor_->create_voxel(processed_points);
    std::vector<VoxelCluster> clusters = clustering_->create_voxel_clustering(processed_points, voxels);
    clustering_->process_clusters(processed_points, clusters, current_time, dt);
    Point3D ball_position = calculate_ball_position(clusters);
    clustering_->refine_ball_clusters(clusters, ball_position);

    // 残りの点群をPointCloud2形式に変換してパブリッシュ
    sensor_msgs::msg::PointCloud2 remaining_cloud = processor.vector_to_PC2(processed_points);
    filtered_cloud_publisher_->publish(remaining_cloud);

    // マーカーのパブリッシュ
    publish_markers(ball_position, clusters, remaining_cloud);
    // RCLCPP_INFO(this->get_logger(), "exec time: %ld ms", std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start).count());
  }

  void BallDetector::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    double w = msg->pose.orientation.w;
    double x = msg->pose.orientation.x;
    double y = msg->pose.orientation.y;
    double z = msg->pose.orientation.z;
    // ヨー角の算出
    double siny_cosp = 2.0 * (w * z + x * y);
    double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    double yaw = std::atan2(siny_cosp, cosy_cosp);
    self_pose_.x = msg->pose.position.x;
    self_pose_.y = msg->pose.position.y;
    self_pose_.z = yaw;
  }

  void BallDetector::autonomous_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    is_autonomous = msg->data;
  }

  void BallDetector::publish_markers(Point3D &ball_position, const std::vector<VoxelCluster> &clusters, const sensor_msgs::msg::PointCloud2 &remaining_cloud)
  {
    visualization_msgs::msg::MarkerArray voxel_marker_array = create_voxel_cluster_markers(clusters);
    clustered_voxel_publisher_->publish(voxel_marker_array);

    if (ball_position.x == 0.0 && ball_position.y == 0.0 && ball_position.z == 0.0)
    {
      return;
    }

    visualization_msgs::msg::Marker marker = create_ball_marker(ball_position, remaining_cloud.header);
    ball_publisher_->publish(marker);

    update_trajectory(ball_position, remaining_cloud);
  }

  Point3D BallDetector::calculate_ball_position(const std::vector<VoxelCluster> &clusters)
  {
    const auto &dynamic_ball_indices = clustering_->get_dynamic_ball_cluster_indices();
    std::vector<Point3D> candidate_points;

    // 動的かつボール以下のサイズのクラスタから全ての点を収集
    for (const auto &index : dynamic_ball_indices)
    {
      const VoxelCluster &cluster = clusters[index];
      candidate_points.insert(candidate_points.end(),
                              cluster.points.begin(),
                              cluster.points.end());
    }

    if (candidate_points.size() < 2)
    {
      // RCLCPP_WARN(this->get_logger(), "候補点が2点未満のため、ボールを検出できません。");
      return Point3D{0.0, 0.0, 0.0}; // 無効な位置を示す
    }

    // x座標でソート
    std::sort(candidate_points.begin(), candidate_points.end(),
              [](const Point3D &a, const Point3D &b)
              { return a.x < b.x; });

    // 最小のxを持つ2点を選択し平均値を計算
    const Point3D &point1 = candidate_points[0];
    const Point3D &point2 = candidate_points[1];

    return Point3D{(point1.x + point2.x) / 2.0, (point1.y + point2.y) / 2.0, (point1.z + point2.z) / 2.0};
  }

  visualization_msgs::msg::MarkerArray BallDetector::create_voxel_cluster_markers(const std::vector<VoxelCluster> &clusters)
  {
    visualization_msgs::msg::MarkerArray marker_array;

    const auto &ball_indices = clustering_->get_ball_size_cluster_indices();
    const auto &dynamic_indices = clustering_->get_dynamic_cluster_indices();
    const auto &dynamic_ball_indices = clustering_->get_dynamic_ball_cluster_indices();

    for (size_t i = 0; i < clusters.size(); ++i)
    {
      bool is_ball_cluster = (ball_indices.find(i) != ball_indices.end());
      bool is_dynamic = (dynamic_indices.find(i) != dynamic_indices.end());
      bool is_dynamic_ball = (dynamic_ball_indices.find(i) != dynamic_ball_indices.end());
      float r, g, b;
      if (is_dynamic_ball)
      {
        r = 0.0f;
        g = 1.0f;
        b = 0.0f; // 緑
      }
      else if (is_ball_cluster)
      {
        r = 1.0f;
        g = 0.0f;
        b = 0.0f; // 赤
      }
      else
      {
        r = 0.0f;
        g = 0.0f;
        b = 1.0f; // 青
      }

      for (size_t v = 0; v < clusters[i].voxels.size(); ++v)
      {
        const auto &voxel = clusters[i].voxels[v];
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = frame_id_;
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

        marker_array.markers.push_back(marker);
      }
    }

    return marker_array;
  }

  visualization_msgs::msg::Marker BallDetector::create_ball_marker(const Point3D &centroid, const std_msgs::msg::Header &header)
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

  void BallDetector::update_trajectory(const Point3D &centroid, const sensor_msgs::msg::PointCloud2 &remaining_cloud)
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
    trajectory_publisher_->publish(trajectory_marker);
    visualization_msgs::msg::Marker past_points_marker = create_past_points_marker(past_points_, remaining_cloud.header);
    past_points_publisher_->publish(past_points_marker);
  }

  visualization_msgs::msg::Marker BallDetector::create_trajectory_marker(const std::deque<Point3D> &trajectory, const std_msgs::msg::Header &header)
  {
    visualization_msgs::msg::Marker marker;
    marker.header = header;
    marker.ns = "ball_trajectory";
    marker.id = 1;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // 線の幅
    marker.scale.x = 0.05; // 線の太さ

    // 線の色
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 188.0 / 255.0; // 青色
    marker.color.a = 0.3;

    // ライフタイムを0に設定して永続的に表示
    marker.lifetime = rclcpp::Duration(0, 0);

    // ポイントを追加
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

  visualization_msgs::msg::Marker BallDetector::create_past_points_marker(const std::deque<Point3D> &past_points, const std_msgs::msg::Header &header)
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
}
