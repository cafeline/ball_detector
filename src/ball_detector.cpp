#include "ball_detector.hpp"
#include "pointcloud_processor/pointcloud_processor.hpp"
#include <chrono>
#include <unordered_set>
#include <random>

BallDetector::BallDetector() : Node("ball_detector")
{
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/camera/camera/depth/color/points", 10, std::bind(&BallDetector::pointcloud_callback, this, std::placeholders::_1));
  ball_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("tennis_ball", 10);
  filtered_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("filtered_pointcloud", 10);

  trajectory_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("ball_trajectory", 10);

  past_points_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("past_ball_points", 10);
  clustered_voxel_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("clustered_voxel", 10);
  human_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("human_marker", 10);

  load_parameters();

  voxel_processor_ = std::make_unique<VoxelProcessor>(params_);
  clustering_ = std::make_unique<Clustering>(params_);
}

void BallDetector::load_parameters()
{
  this->declare_parameter("min_x", -10.0);
  this->declare_parameter("max_x", 10.0);
  this->declare_parameter("min_y", -10.0);
  this->declare_parameter("max_y", 10.0);
  this->declare_parameter("min_z", -2.0);
  this->declare_parameter("max_z", 5.0);
  this->declare_parameter("voxel_size_x", 0.1);
  this->declare_parameter("voxel_size_y", 0.1);
  this->declare_parameter("voxel_size_z", 0.1);
  this->declare_parameter("D_voxel_size_x", 0.05);
  this->declare_parameter("D_voxel_size_y", 0.05);
  this->declare_parameter("D_voxel_size_z", 0.05);
  this->declare_parameter("voxel_search_range", 3);
  this->declare_parameter("ball_radius", 0.1);

  params_.min_x = this->get_parameter("min_x").as_double();
  params_.max_x = this->get_parameter("max_x").as_double();
  params_.min_y = this->get_parameter("min_y").as_double();
  params_.max_y = this->get_parameter("max_y").as_double();
  params_.min_z = this->get_parameter("min_z").as_double();
  params_.max_z = this->get_parameter("max_z").as_double();
  params_.voxel_size_x = this->get_parameter("voxel_size_x").as_double();
  params_.voxel_size_y = this->get_parameter("voxel_size_y").as_double();
  params_.voxel_size_z = this->get_parameter("voxel_size_z").as_double();
  params_.D_voxel_size_x = this->get_parameter("D_voxel_size_x").as_double();
  params_.D_voxel_size_y = this->get_parameter("D_voxel_size_y").as_double();
  params_.D_voxel_size_z = this->get_parameter("D_voxel_size_z").as_double();
  params_.voxel_search_range = this->get_parameter("voxel_search_range").as_int();
  params_.ball_radius = this->get_parameter("ball_radius").as_double();
}

void BallDetector::pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "***********************************************");

  auto start_time = std::chrono::high_resolution_clock::now();

  // 外部ライブラリを使用して点群処理を実行
  PointCloudProcessor processor(params_);
  std::vector<Point3D> processed_points = processor.process_pointcloud(*msg);
  std::vector<Point3D> downsampled_points = processor.get_downsampled_points();

  detect_human(downsampled_points);

  RCLCPP_INFO(this->get_logger(), "Time taken for voxelization and clustering: %ld ms",
             std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::high_resolution_clock::now() - start_time).count());

  // 点群処理の分割
  std::vector<VoxelCluster> clusters = process_pointcloud(processed_points, downsampled_points);

  // 残りの点群をPointCloud2形式に変換してパブリッシュ
  sensor_msgs::msg::PointCloud2 remaining_cloud = processor.vector_to_PC2(processed_points);
  filtered_cloud_publisher_->publish(remaining_cloud);

  // マーカーのパブリッシュ
  publish_markers(clusters, remaining_cloud);

  // 軌道と過去の検出点の更新
  update_trajectory(clusters, remaining_cloud);
}

// 追加: detect_human メソッドの実装
void BallDetector::detect_human(const std::vector<Point3D> &points)
{
  RCLCPP_INFO(this->get_logger(), "0");
  // ボクセルサイズと探索範囲の設定
  double voxel_size_x = 0.5;
  double min_z = 0.5;
  double max_z = 1.0;
  double min_y = params_.min_y;
  double max_y = params_.max_y;
  std::vector<Point3D> human_points;
  for (const auto &point : points)
  {
    if (point.z >= min_z && point.z <= max_z)
    {
      human_points.push_back(point);
      RCLCPP_INFO(this->get_logger(), "#########human#########");
    }
  }
  // human_pointsでボクセル化　クラスタリング　コートの中心に一番近いクラスタを人として認識
  std::vector<Voxel> human_voxels = voxel_processor_->create_voxel(human_points);
  RCLCPP_INFO(this->get_logger(), "human_voxels.size(): %ld", human_voxels.size());
}

std::vector<Point3D> BallDetector::axis_image2robot(const std::vector<Point3D> &input)
{
  std::vector<Point3D> swapped;
  swapped.reserve(input.size());

  for (const auto &point : input)
  {
    Point3D swapped_point;
    swapped_point.x = point.z;
    swapped_point.y = point.x;
    swapped_point.z = -point.y;
    swapped.push_back(swapped_point);
  }

  return swapped;
}

std::vector<VoxelCluster> BallDetector::process_pointcloud(const std::vector<Point3D> &filtered_points, const std::vector<Point3D> &downsampled_points)
{
  // ボクセル化の実行
  std::vector<Voxel> voxels = voxel_processor_->create_voxel(downsampled_points);

  // クラスタリングの実行
  std::vector<VoxelCluster> clusters = clustering_->create_voxel_clustering(downsampled_points, voxels);

  // クラスタ化された点群の除去
  std::vector<Point3D> remaining_points = remove_clustered_points(downsampled_points, clusters);

  // 残りの点群を更新
  clustered_points_ = std::move(remaining_points);

  return clusters;
}

void BallDetector::publish_markers(const std::vector<VoxelCluster> &clusters, const sensor_msgs::msg::PointCloud2 &remaining_cloud)
{
  // クラスタのマーカーを作成してパブリッシュ
  if (!clusters.empty())
  {
    visualization_msgs::msg::MarkerArray voxel_marker_array = create_voxel_cluster_markers(clusters);
    clustered_voxel_publisher_->publish(voxel_marker_array);
  }

  // 各クラスタ内で x 座標が最も小さい点を見つける
  for (const auto &cluster : clusters)
  {
    if (cluster.points.empty())
    {
      continue;
    }

    // 最小 x 座標を持つ点を検索
    const Point3D *min_x_point = &cluster.points[0];
    for (const auto &point : cluster.points)
    {
      if (point.x < min_x_point->x)
      {
        min_x_point = &point;
      }
    }

    // マーカーを作成してパブリッシュ
    visualization_msgs::msg::Marker marker = create_custom_marker(*min_x_point, remaining_cloud.header);
    ball_publisher_->publish(marker);
  }
}

visualization_msgs::msg::Marker BallDetector::create_custom_marker(const Point3D &point, const std_msgs::msg::Header &header)
{
  visualization_msgs::msg::Marker marker;
  marker.header = header;
  marker.ns = "min_x_points";
  marker.id = static_cast<int>(point.x * 1000); // 一意のIDを設定（例: x座標を基に）
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;

  // 点の位置を設定
  marker.pose.position.x = point.x;
  marker.pose.position.y = point.y;
  marker.pose.position.z = point.z;

  // マーカーのスケールを設定
  marker.scale.x = 0.05;
  marker.scale.y = 0.05;
  marker.scale.z = 0.05;

  // マーカーの色を設定 (例: 青色)
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;
  marker.color.a = 1.0;

  // ライフタイムを0に設定して永続的に表示
  marker.lifetime = rclcpp::Duration(0, 0);

  return marker;
}
void BallDetector::update_trajectory(const std::vector<VoxelCluster> &clusters, const sensor_msgs::msg::PointCloud2 &remaining_cloud)
{
  VoxelCluster ball_cluster;
  for (const auto &cluster : clusters)
  {
    for (const auto &point : cluster.points)
    {
      ball_cluster.points.push_back(point);
    }
  }

  if (!ball_cluster.points.empty())
  {
    Point3D centroid = calculate_cluster_centroid(ball_cluster);

    // 軌道コンテナに追加
    const size_t MAX_TRAJECTORY_POINTS = 200; // 最大保持ポイント数
    if (ball_trajectory_points_.size() >= MAX_TRAJECTORY_POINTS)
    {
      ball_trajectory_points_.pop_front(); // 古いポイントを削除
    }
    ball_trajectory_points_.push_back(centroid);

    // 過去の検出点コンテナに追加
    const size_t MAX_PAST_POINTS = 200; // 最大保持過去点数
    if (past_points_.size() >= MAX_PAST_POINTS)
    {
      past_points_.pop_front(); // 古いポイントを削除
    }
    past_points_.push_back(centroid);

    // それぞれPublish
    visualization_msgs::msg::Marker trajectory_marker = create_trajectory_marker(ball_trajectory_points_, remaining_cloud.header);
    trajectory_publisher_->publish(trajectory_marker);
    visualization_msgs::msg::Marker past_points_marker = create_past_points_marker(past_points_, remaining_cloud.header);
    past_points_publisher_->publish(past_points_marker);
  }
}

std::vector<Point3D> BallDetector::voxel_downsample(const std::vector<Point3D> &input)
{
  std::unordered_map<std::string, std::vector<Point3D>> voxel_map;
  std::vector<Point3D> downsampled_points;

  for (const auto &point : input)
  {
    int voxel_x = static_cast<int>(std::floor(point.x / params_.D_voxel_size_x));
    int voxel_y = static_cast<int>(std::floor(point.y / params_.D_voxel_size_y));
    int voxel_z = static_cast<int>(std::floor(point.z / params_.D_voxel_size_z));
    std::string key = std::to_string(voxel_x) + "_" + std::to_string(voxel_y) + "_" + std::to_string(voxel_z);
    voxel_map[key].push_back(point);
  }

  for (const auto &voxel : voxel_map)
  {
    const auto &points = voxel.second;
    if (points.size() < 4)
      continue;

    float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;
    for (const auto &p : points)
    {
      sum_x += p.x;
      sum_y += p.y;
      sum_z += p.z;
    }

    Point3D centroid;
    centroid.x = sum_x / points.size();
    centroid.y = sum_y / points.size();
    centroid.z = sum_z / points.size();
    downsampled_points.push_back(centroid);
  }

  return downsampled_points;
}

std::vector<Point3D> BallDetector::PC2_to_vector(const sensor_msgs::msg::PointCloud2 &cloud_msg)
{
  std::vector<Point3D> points;
  size_t num_points = cloud_msg.width * cloud_msg.height;
  points.reserve(num_points);

  size_t point_step = cloud_msg.point_step;
  size_t x_offset = cloud_msg.fields[0].offset;
  size_t y_offset = cloud_msg.fields[1].offset;
  size_t z_offset = cloud_msg.fields[2].offset;

  for (size_t i = 0; i < num_points; ++i)
  {
    size_t data_index = i * point_step;
    Point3D point;
    memcpy(&point.x, &cloud_msg.data[data_index + x_offset], sizeof(float));
    memcpy(&point.y, &cloud_msg.data[data_index + y_offset], sizeof(float));
    memcpy(&point.z, &cloud_msg.data[data_index + z_offset], sizeof(float));
    points.push_back(point);
  }

  return points;
}

sensor_msgs::msg::PointCloud2 BallDetector::vector_to_PC2(const std::vector<Point3D> &points)
{
  sensor_msgs::msg::PointCloud2 cloud_msg;
  cloud_msg.header.frame_id = frame_id_;
  cloud_msg.height = 1;
  cloud_msg.width = points.size();
  cloud_msg.fields.resize(3);
  cloud_msg.fields[0].name = "x";
  cloud_msg.fields[1].name = "y";
  cloud_msg.fields[2].name = "z";

  for (int i = 0; i < 3; ++i)
  {
    cloud_msg.fields[i].offset = i * sizeof(float);
    cloud_msg.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud_msg.fields[i].count = 1;
  }

  cloud_msg.point_step = 3 * sizeof(float);
  cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width;
  cloud_msg.data.resize(cloud_msg.row_step * cloud_msg.height);
  cloud_msg.is_bigendian = false;
  cloud_msg.is_dense = true;

  for (size_t i = 0; i < points.size(); ++i)
  {
    size_t data_index = i * cloud_msg.point_step;
    memcpy(&cloud_msg.data[data_index], &points[i].x, sizeof(float));
    memcpy(&cloud_msg.data[data_index + sizeof(float)], &points[i].y, sizeof(float));
    memcpy(&cloud_msg.data[data_index + 2 * sizeof(float)], &points[i].z, sizeof(float));
  }

  return cloud_msg;
}

std::vector<Point3D> BallDetector::filter_points(const std::vector<Point3D> &input)
{
  std::vector<Point3D> output;
  for (const auto &point : input)
  {
    if (point.x != 0.0f && point.y != 0.0f && point.z != 0.0f &&
        point.x >= params_.min_x && point.x <= params_.max_x &&
        point.y >= params_.min_y && point.y <= params_.max_y &&
        point.z >= params_.min_z && point.z <= params_.max_z)
    {
      output.push_back(point);
    }
  }
  return output;
}

Point3D BallDetector::calculate_centroid(const std::vector<Point3D> &points)
{
  double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
  for (const auto &point : points)
  {
    sum_x += point.x;
    sum_y += point.y;
    sum_z += point.z;
  }
  Point3D centroid;
  if (!points.empty())
  {
    centroid.x = static_cast<float>(sum_x / points.size());
    centroid.y = static_cast<float>(sum_y / points.size());
    centroid.z = static_cast<float>(sum_z / points.size());
  }
  else
  {
    centroid = {0.0f, 0.0f, 0.0f};
  }
  return centroid;
}

Point3D BallDetector::calculate_cluster_centroid(const VoxelCluster &cluster)
{
  return calculate_centroid(cluster.points);
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
  marker.lifetime = rclcpp::Duration(0, 1e8); // 約0.1秒
  return marker;
}

visualization_msgs::msg::MarkerArray BallDetector::create_voxel_markers(const std::vector<Voxel> &voxels, const std_msgs::msg::Header &header)
{
  visualization_msgs::msg::MarkerArray marker_array;
  for (size_t i = 0; i < voxels.size(); ++i)
  {
    visualization_msgs::msg::Marker marker;
    marker.header = header;
    marker.ns = "voxel_markers";
    marker.id = i;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // ボクセルの中へのオフセットを計算 (ボクセルの左下前→中心)
    double offset_x = params_.voxel_size_x / 2.0;
    double offset_y = params_.voxel_size_y / 2.0;
    double offset_z = params_.voxel_size_z / 2.0;

    marker.pose.position.x = params_.min_x + (voxels[i].x * params_.voxel_size_x) + offset_x;
    marker.pose.position.y = params_.min_y + (voxels[i].y * params_.voxel_size_y) + offset_y;
    marker.pose.position.z = params_.min_z + (voxels[i].z * params_.voxel_size_z) + offset_z;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = params_.voxel_size_x;
    marker.scale.y = params_.voxel_size_y;
    marker.scale.z = params_.voxel_size_z;

    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 0.5;

    marker.lifetime = rclcpp::Duration(0, 1e8); // 約0.1秒

    marker_array.markers.push_back(marker);
  }

  return marker_array;
}

visualization_msgs::msg::MarkerArray BallDetector::create_voxel_cluster_markers(const std::vector<VoxelCluster> &clusters)
{
  visualization_msgs::msg::MarkerArray marker_array;

  // クラスタごとに異なる色を生成するためのランダムジェネレータ
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis(0.0, 1.0);

  for (size_t i = 0; i < clusters.size(); ++i)
  {
    const auto &cluster = clusters[i];

    // クラスタごとにランダムな色を生成
    float r = dis(gen);
    float g = dis(gen);
    float b = dis(gen);

    for (const auto &voxel : cluster.voxels)
    {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = frame_id_;
      marker.ns = "voxel_cluster_markers";
      marker.id = i * 1000 + marker_array.markers.size(); // ユニークなIDを生成
      marker.type = visualization_msgs::msg::Marker::CUBE;
      marker.action = visualization_msgs::msg::Marker::ADD;

      // ボクセルの中心へのオフセットを計算
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

      // クラスタごとの色を設定
      marker.color.r = r;
      marker.color.g = g;
      marker.color.b = b;
      marker.color.a = 0.9;

      marker.lifetime = rclcpp::Duration(0, 1e8); // 約0.1秒

      marker_array.markers.push_back(marker);
    }
  }

  return marker_array;
}

visualization_msgs::msg::Marker BallDetector::create_detection_area_marker(const std_msgs::msg::Header &header)
{
  visualization_msgs::msg::Marker marker;
  marker.header = header;
  marker.ns = "detection_area";
  marker.id = 2;
  marker.type = visualization_msgs::msg::Marker::CUBE;
  marker.action = visualization_msgs::msg::Marker::ADD;

  marker.pose.position.x = (params_.min_x + params_.max_x) / 2.0;
  marker.pose.position.y = (params_.min_y + params_.max_y) / 2.0;
  marker.pose.position.z = (params_.min_z + params_.max_z) / 2.0;
  marker.pose.orientation.w = 1.0;

  marker.scale.x = params_.max_x - params_.min_x;
  marker.scale.y = params_.max_y - params_.min_y;
  marker.scale.z = params_.max_z - params_.min_z;

  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;
  marker.color.a = 0.1;

  marker.lifetime = rclcpp::Duration(0, 1e8); // 約0.1秒

  return marker;
}

std::vector<Point3D> BallDetector::remove_clustered_points(const std::vector<Point3D> &original_points, const std::vector<VoxelCluster> &clusters)
{
  std::vector<Point3D> remaining_points;
  std::unordered_set<std::string> clustered_voxels;

  for (const auto &cluster : clusters)
  {
    for (const auto &voxel : cluster.voxels)
    {
      std::string key = std::to_string(voxel.x) + "," + std::to_string(voxel.y) + "," + std::to_string(voxel.z);
      clustered_voxels.insert(key);
    }
  }

  for (const auto &point : original_points)
  {
    int vx = static_cast<int>((point.x - params_.min_x) / params_.voxel_size_x);
    int vy = static_cast<int>((point.y - params_.min_y) / params_.voxel_size_y);
    int vz = static_cast<int>((point.z - params_.min_z) / params_.voxel_size_z);
    std::string key = std::to_string(vx) + "," + std::to_string(vy) + "," + std::to_string(vz);

    if (clustered_voxels.find(key) == clustered_voxels.end())
    {
      remaining_points.push_back(point);
    }
  }

  return remaining_points;
}

void BallDetector::collect_cluster_points(VoxelCluster &cluster, const std::vector<Point3D> &points)
{
  clustering_->collect_cluster_points(cluster, points);
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
  marker.color.g = 0.0;
  marker.color.b = 1.0; // 青色
  marker.color.a = 1.0; // 完全不透明

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
  marker.color.a = 1.0; // 完全不透明

  // ライフタイムを0に設定して永続的に表示
  marker.lifetime = rclcpp::Duration(0, 0);

  // 過去の検出点を追加
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

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BallDetector>());
  rclcpp::shutdown();
  return 0;
}
