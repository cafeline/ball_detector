#include "ball_detector.hpp"
#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp/logging.hpp>
#include <algorithm>
#include <random>  // 色生成のために追加


BallDetector::BallDetector() : Node("ball_detector")
{
  subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/livox/lidar", 10, std::bind(&BallDetector::pointcloud_callback, this, std::placeholders::_1));
  
  marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("ball_marker", 10);
  filtered_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("filtered_pointcloud", 10);
  bounding_box_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("bounding_box_marker", 10);

  // 新しいパブリッシャーを追加
  region_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("region_pointcloud", 10);

  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),  // 100msごとに実行
      std::bind(&BallDetector::timer_callback, this));

  this->declare_parameter("min_x", 0.0);
  this->declare_parameter("max_x", 3.0);
  this->declare_parameter("min_y", -0.5);
  this->declare_parameter("max_y", 0.5);
  this->declare_parameter("min_z", 0.0);
  this->declare_parameter("max_z", 2.0);
}

void BallDetector::timer_callback(){
  std_msgs::msg::Header header;
  header.stamp = this->get_clock()->now();
  header.frame_id = frame_id_;
  auto bounding_box_marker = create_bounding_box_marker(header);
  bounding_box_publisher_->publish(bounding_box_marker);
}

void BallDetector::pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  std::vector<Point3D> points = PC2_to_vector(*msg);

  // 指定領域内の点群を抽出
  std::vector<Point3D> filtered_points;
  filter_points(points, filtered_points);
////////////////////////////////////////////////////////////////////////////////////
  // // 重心位置を計算
  // Point3D centroid = calculate_centroid(filtered_points);

  // // マーカーを作成してパブリッシュ
  // std_msgs::msg::Header header;
  // header.stamp = this->get_clock()->now();
  // header.frame_id = frame_id_;  // frame_id を frame_id_ に設定
  // auto marker = create_ball_marker(centroid, header);
  // marker_publisher_->publish(marker);

  // // フィルタリングされた点群をパブリッシュ
  // auto filtered_cloud_msg = vector_to_PC2(filtered_points);
  // filtered_cloud_msg.header = msg->header;
  // filtered_cloud_publisher_->publish(filtered_cloud_msg);
////////////////////////////////////////////////////////////////////////////////////

  // 領域の抽出
  std::vector<Region> regions = extract_regions(filtered_points, 1);

  // 各領域に対して処理を行う（マーカーとバウンディングボックスを作成）
  std_msgs::msg::Header header;
  header.stamp = this->get_clock()->now();
  header.frame_id = frame_id_;

  // ランダムカラー生成の準備
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis(0.0, 1.0);

  for (size_t i = 0; i < regions.size(); ++i)
  {
      // 色の設定（ランダム）
      regions[i].color = {static_cast<float>(dis(gen)), static_cast<float>(dis(gen)), static_cast<float>(dis(gen)), 1.0f};

      // 重心マーカーの作成とパブリッシュ
      Point3D centroid = calculate_centroid(regions[i].points);
      auto marker = create_ball_marker(centroid, header);
      marker.id = static_cast<int>(i + 1); // IDを変更
      marker.color.r = regions[i].color[0];
      marker.color.g = regions[i].color[1];
      marker.color.b = regions[i].color[2];
      marker_publisher_->publish(marker);

      // バウンディングボックスマーカーの作成とパブリッシュ
      auto bbox_marker = create_region_bounding_box_marker(regions[i], header, i + 1);
      bounding_box_publisher_->publish(bbox_marker);
  }

  // フィルタリングされた点群をパブリッシュ
  auto filtered_cloud_msg = vector_to_PC2(filtered_points);
  filtered_cloud_msg.header = msg->header;
  filtered_cloud_publisher_->publish(filtered_cloud_msg);
}

std::vector<Point3D> BallDetector::PC2_to_vector(const sensor_msgs::msg::PointCloud2& cloud_msg)
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
    point.x = *reinterpret_cast<const float*>(&cloud_msg.data[data_index + x_offset]);
    point.y = *reinterpret_cast<const float*>(&cloud_msg.data[data_index + y_offset]);
    point.z = *reinterpret_cast<const float*>(&cloud_msg.data[data_index + z_offset]);
    points.push_back(point);
  }

  return points;
}

sensor_msgs::msg::PointCloud2 BallDetector::vector_to_PC2(const std::vector<Point3D>& points)
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

void BallDetector::filter_points(const std::vector<Point3D>& input, std::vector<Point3D>& output)
{
  double min_x = this->get_parameter("min_x").as_double();
  double max_x = this->get_parameter("max_x").as_double();
  double min_y = this->get_parameter("min_y").as_double();
  double max_y = this->get_parameter("max_y").as_double();
  double min_z = this->get_parameter("min_z").as_double();
  double max_z = this->get_parameter("max_z").as_double();

  for (const auto& point : input)
  {
    if (point.x != 0.0f && point.y != 0.0f && point.z != 0.0f &&
        point.x >= min_x && point.x <= max_x &&
        point.y >= min_y && point.y <= max_y &&
        point.z >= min_z && point.z <= max_z)
    {
      output.push_back(point);
    }
  }
}

Point3D BallDetector::calculate_centroid(const std::vector<Point3D>& points)
{
  Point3D centroid = {0.0f, 0.0f, 0.0f};
  if (points.empty()) {
    return centroid;
  }

  for (const auto& point : points) {
    centroid.x += point.x;
    centroid.y += point.y;
    centroid.z += point.z;
  }

  float num_points = static_cast<float>(points.size());
  centroid.x /= num_points;
  centroid.y /= num_points;
  centroid.z /= num_points;

  return centroid;
}

visualization_msgs::msg::Marker BallDetector::create_ball_marker(const Point3D& centroid, const std_msgs::msg::Header& header){
  visualization_msgs::msg::Marker marker;
  marker.header = header;
  marker.ns = "ball_detector";
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
  marker.lifetime = rclcpp::Duration(0, 1e8);  // 0.1 seconds
  return marker;
}

visualization_msgs::msg::Marker BallDetector::create_bounding_box_marker(const std_msgs::msg::Header& header)
{
  visualization_msgs::msg::Marker marker;
  marker.header = header;
  marker.ns = "ball_detector_bounding_box";
  marker.id = 1;
  marker.type = visualization_msgs::msg::Marker::CUBE;
  marker.action = visualization_msgs::msg::Marker::ADD;

  double min_x = this->get_parameter("min_x").as_double();
  double max_x = this->get_parameter("max_x").as_double();
  double min_y = this->get_parameter("min_y").as_double();
  double max_y = this->get_parameter("max_y").as_double();
  double min_z = this->get_parameter("min_z").as_double();
  double max_z = this->get_parameter("max_z").as_double();

  marker.pose.position.x = (min_x + max_x) / 2.0;
  marker.pose.position.y = (min_y + max_y) / 2.0;
  marker.pose.position.z = (min_z + max_z) / 2.0;
  marker.pose.orientation.w = 1.0;

  marker.scale.x = max_x - min_x;
  marker.scale.y = max_y - min_y;
  marker.scale.z = max_z - min_z;

  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.color.a = 0.1;

  marker.lifetime = rclcpp::Duration(0, 1e8);  // 0.1 seconds

  return marker;
}

std::vector<Region> BallDetector::extract_regions(const std::vector<Point3D>& points, size_t max_regions){
  std::vector<Region> regions;
  double expantion_size = 0.05;

  for (const auto& point : points){
      bool assigned = false;
      for (auto& region : regions)
      {
          // 領域サイズの条件をチェック
          if (point.x >= region.min_x && point.x <= region.max_x &&
              point.y >= region.min_y && point.y <= region.max_y &&
              point.z >= region.min_z && point.z <= region.max_z)
          {
              region.points.push_back(point);
              // 領域の拡張が必要な場合
              region.min_x = std::min(region.min_x, point.x - expantion_size);
              region.max_x = std::max(region.max_x, point.x + expantion_size);
              region.min_y = std::min(region.min_y, point.y - expantion_size);
              region.max_y = std::max(region.max_y, point.y + expantion_size);
              region.min_z = std::min(region.min_z, point.z - expantion_size);
              region.max_z = std::max(region.max_z, point.z + expantion_size);
              assigned = true;
              break;
          }
      }
      if (!assigned && regions.size() < max_regions)
      {
          Region new_region;
          new_region.min_x = point.x - expantion_size;
          new_region.max_x = point.x + expantion_size;
          new_region.min_y = point.y - expantion_size;
          new_region.max_y = point.y + expantion_size;
          new_region.min_z = point.z - expantion_size;
          new_region.max_z = point.z + expantion_size;
          new_region.points.push_back(point);
          // ランダムカラーの初期化
          new_region.color = {1.0f, 0.0f, 0.0f, 1.0f};  // デフォルトは赤
          regions.push_back(new_region);
      }
      if (regions.size() >= max_regions)
      {
          break;
      }
  }
  return regions;
}

// 新しく追加するメソッド：領域ごとのバウンディングボックスマーカーを作成
visualization_msgs::msg::Marker BallDetector::create_region_bounding_box_marker(const Region& region, const std_msgs::msg::Header& header, int id)
{
  visualization_msgs::msg::Marker marker;
  marker.header = header;
  marker.ns = "ball_detector_region_bounding_box";
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::CUBE;
  marker.action = visualization_msgs::msg::Marker::ADD;

  // バウンディングボックスの中心を計算
  marker.pose.position.x = (region.min_x + region.max_x) / 2.0;
  marker.pose.position.y = (region.min_y + region.max_y) / 2.0;
  marker.pose.position.z = 2.0 / 2.0;
  marker.pose.orientation.w = 1.0;

  // スケールを設定
  marker.scale.x = region.max_x - region.min_x;
  marker.scale.y = region.max_y - region.min_y;
  marker.scale.z = region.max_z - region.min_z;

  // 色を設定（領域ごとの色）
  marker.color.r = region.color[0];
  marker.color.g = region.color[1];
  marker.color.b = region.color[2];
  marker.color.a = 0.5;  // 半透明

  marker.lifetime = rclcpp::Duration(0, 1e8);  // 0.1 seconds

  return marker;
}

// 新しく追加する関数：領域内の点群をPointCloud2メッセージに変換
sensor_msgs::msg::PointCloud2 BallDetector::region_to_PC2(const Region& region)
{
  sensor_msgs::msg::PointCloud2 cloud_msg;
  cloud_msg.header.frame_id = frame_id_;
  cloud_msg.height = 1;
  cloud_msg.width = region.points.size();
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

  for (size_t i = 0; i < region.points.size(); ++i)
  {
    size_t data_index = i * cloud_msg.point_step;
    memcpy(&cloud_msg.data[data_index], &region.points[i].x, sizeof(float));
    memcpy(&cloud_msg.data[data_index + sizeof(float)], &region.points[i].y, sizeof(float));
    memcpy(&cloud_msg.data[data_index + 2 * sizeof(float)], &region.points[i].z, sizeof(float));
  }

  return cloud_msg;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BallDetector>());
  rclcpp::shutdown();
  return 0;
}