#include "ball_detector/ball_detector_node.hpp"

namespace ball_detector
{
  BallDetectorNode::BallDetectorNode(const rclcpp::NodeOptions &options)
      : Node("ball_detector", options)
  {
    ball_detector_core_ = std::make_unique<BallDetectorCore>();

    load_parameters();
    setup_publishers_and_subscribers();
  }

  void BallDetectorNode::load_parameters()
  {
    // パラメータの読み込み処理（BallDetectorクラスから移動）
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

    ball_detector_core_->set_params(params_);
    pointcloud_processor = std::make_unique<PointCloudProcessor>(params_);
  }

  void BallDetectorNode::setup_publishers_and_subscribers()
  {
    // サブスクリプションの設定
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/livox/lidar", 10, std::bind(&BallDetectorNode::pointcloud_callback, this, std::placeholders::_1));
    pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/self_pose", 10, std::bind(&BallDetectorNode::pose_callback, this, std::placeholders::_1));
    autonomous_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
        "autonomous", 10, std::bind(&BallDetectorNode::autonomous_callback, this, std::placeholders::_1));

    // パブリッシャーの設定
    ball_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("tennis_ball", 10);
    filtered_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("ball_detector_pointcloud", 10);
    trajectory_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("ball_trajectory", 10);
    past_points_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("past_ball_points", 10);
    clustered_voxel_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("clustered_voxel", 10);
  }

  void BallDetectorNode::pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // if (!is_autonomous)
    //   return;

    auto start = std::chrono::high_resolution_clock::now();
    rclcpp::Time current_time = this->now();
    double dt = (previous_time_.nanoseconds() == 0) ? 0.0 : (current_time - previous_time_).seconds();
    previous_time_ = current_time;

    // ポイントクラウドの処理
    std::vector<Point3D> processed_points = pointcloud_processor->process(msg, self_pose_.x, self_pose_.y, self_pose_.z);

    // ボール検出
    DetectionResult result = ball_detector_core_->detect_ball(processed_points, current_time, dt);

    // 視覚化
    publish_visualization(result, msg->header);

    // RCLCPP_INFO(this->get_logger(), "exec time: %ld ms", std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start).count());
  }

  void BallDetectorNode::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    double w = msg->pose.orientation.w;
    double x = msg->pose.orientation.x;
    double y = msg->pose.orientation.y;
    double z = msg->pose.orientation.z;

    double siny_cosp = 2.0 * (w * z + x * y);
    double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    double yaw = std::atan2(siny_cosp, cosy_cosp);

    self_pose_.x = msg->pose.position.x;
    self_pose_.y = msg->pose.position.y;
    self_pose_.z = yaw;
  }

  void BallDetectorNode::autonomous_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    is_autonomous = msg->data;
  }

  void BallDetectorNode::publish_visualization(const DetectionResult &result, const std_msgs::msg::Header &header)
  {
    visualization_msgs::msg::MarkerArray voxel_marker_array = ball_detector_core_->marker_factory_->create_voxel_cluster_markers(
        result.clusters, result.ball_indices, result.dynamic_indices, result.dynamic_ball_indices);
    clustered_voxel_publisher_->publish(voxel_marker_array);

    sensor_msgs::msg::PointCloud2 remaining_cloud = pointcloud_processor->vector_to_PC2(result.processed_points);
    filtered_cloud_publisher_->publish(remaining_cloud);

    if (result.ball_position.x == 0.0 && result.ball_position.y == 0.0 && result.ball_position.z == 0.0)
    {
      return;
    }
    visualization_msgs::msg::Marker marker = ball_detector_core_->marker_factory_->create_ball_marker(result.ball_position, header);
    ball_publisher_->publish(marker);

    ball_detector_core_->trajectory_manager_->update_trajectory(result.ball_position);
    visualization_msgs::msg::Marker trajectory_marker = ball_detector_core_->trajectory_manager_->create_trajectory_marker(header);
    trajectory_publisher_->publish(trajectory_marker);
    // visualization_msgs::msg::Marker past_points_marker = ball_detector_core_->marker_factory_->create_past_points_marker(ball_detector_core_->marker_factory_->past_points_, header);
    // past_points_publisher_->publish(past_points_marker);
  }

} // namespace ball_detector
