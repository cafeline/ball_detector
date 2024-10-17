#include "ball_detector.hpp"
#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp/logging.hpp>
#include <algorithm>
#include <random>  // 色生成のために追加
#include <unordered_map>  // ボクセルベースのクラスタリングのために追加
#include <cmath>  // ボクセルベースのクラスタリングのために追加
#include <chrono>

// 新しい定数を追加
const float VOXEL_SIZE_X = 0.1f;
const float VOXEL_SIZE_Y = 0.1f;
const float VOXEL_SIZE_Z = 0.1f;

BallDetector::BallDetector() : Node("ball_detector") {
  subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/livox/lidar", 10, std::bind(&BallDetector::pointcloud_callback, this, std::placeholders::_1));
  marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("occupied_voxels", 10);
  filtered_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("filtered_pointcloud", 10);
  detection_area_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("bounding_box_marker", 10);

  this->declare_parameter("min_x", 0.0);
  this->declare_parameter("max_x", 3.0);
  this->declare_parameter("min_y", -3.0);
  this->declare_parameter("max_y", 3.0);
  this->declare_parameter("min_z", 0.0);
  this->declare_parameter("max_z", 5.0);
}

void BallDetector::pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  std::vector<Point3D> points = PC2_to_vector(*msg);

  // 指定領域内の点群を抽出
  std::vector<Point3D> filtered_points = filter_points(points);

  auto start_time = std::chrono::steady_clock::now();
  // ボクセルベースのクラスタリングを実行
  std::vector<Voxel> occupied_voxels = voxel_clustering(filtered_points);
  RCLCPP_INFO(this->get_logger(), "Voxel clustering time: %ld microseconds", std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start_time).count());

  // マーカーを作成してパブリッシュ
  std_msgs::msg::Header header;
  header.stamp = this->get_clock()->now();
  header.frame_id = frame_id_;

  // 占有されたボクセルをマーカーとしてパブリッシュ
  auto voxel_markers = create_voxel_markers(occupied_voxels, header);
  marker_publisher_->publish(voxel_markers);

  // 点群をパブリッシュ
  auto output_cloud_msg = vector_to_PC2(filtered_points);
  output_cloud_msg.header = header;
  filtered_cloud_publisher_->publish(output_cloud_msg);

  auto detection_area_marker = create_detection_area_marker(header);
  detection_area_publisher_->publish(detection_area_marker);
}

std::vector<Point3D> BallDetector::PC2_to_vector(const sensor_msgs::msg::PointCloud2& cloud_msg) {
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

sensor_msgs::msg::PointCloud2 BallDetector::vector_to_PC2(const std::vector<Point3D>& points) {
  sensor_msgs::msg::PointCloud2 cloud_msg;
  cloud_msg.header.frame_id = frame_id_;
  cloud_msg.height = 1;
  cloud_msg.width = points.size();
  cloud_msg.fields.resize(3);
  cloud_msg.fields[0].name = "x";
  cloud_msg.fields[1].name = "y";
  cloud_msg.fields[2].name = "z";

  for (int i = 0; i < 3; ++i) {
    cloud_msg.fields[i].offset = i * sizeof(float);
    cloud_msg.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud_msg.fields[i].count = 1;
  }

  cloud_msg.point_step = 3 * sizeof(float);
  cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width;
  cloud_msg.data.resize(cloud_msg.row_step * cloud_msg.height);
  cloud_msg.is_bigendian = false;
  cloud_msg.is_dense = true;

  for (size_t i = 0; i < points.size(); ++i) {
    size_t data_index = i * cloud_msg.point_step;
    memcpy(&cloud_msg.data[data_index], &points[i].x, sizeof(float));
    memcpy(&cloud_msg.data[data_index + sizeof(float)], &points[i].y, sizeof(float));
    memcpy(&cloud_msg.data[data_index + 2 * sizeof(float)], &points[i].z, sizeof(float));
  }

  return cloud_msg;
}

std::vector<Point3D> BallDetector::filter_points(const std::vector<Point3D>& input) {
  std::vector<Point3D> output;
  double min_x = this->get_parameter("min_x").as_double();
  double max_x = this->get_parameter("max_x").as_double();
  double min_y = this->get_parameter("min_y").as_double();
  double max_y = this->get_parameter("max_y").as_double();
  double min_z = this->get_parameter("min_z").as_double();
  double max_z = this->get_parameter("max_z").as_double();

  for (const auto& point : input) {
    if (point.x != 0.0f && point.y != 0.0f && point.z != 0.0f &&
      point.x >= min_x && point.x <= max_x &&
      point.y >= min_y && point.y <= max_y &&
      point.z >= min_z && point.z <= max_z){
      output.push_back(point);
    }
  }
  return output;
}

Point3D BallDetector::calculate_centroid(const std::vector<Point3D>& points) {
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

std::vector<Voxel> BallDetector::voxel_clustering(const std::vector<Point3D>& points){
  std::unordered_map<std::string, Voxel> occupied_voxels;

  double min_x = this->get_parameter("min_x").as_double();
  double min_y = this->get_parameter("min_y").as_double();
  double min_z = this->get_parameter("min_z").as_double();

  for (const auto& point : points){
    int vx = static_cast<int>((point.x - min_x) / VOXEL_SIZE_X);
    int vy = static_cast<int>((point.y - min_y) / VOXEL_SIZE_Y);
    int vz = static_cast<int>((point.z - min_z) / VOXEL_SIZE_Z);
    std::string key = std::to_string(vx) + "," + std::to_string(vy) + "," + std::to_string(vz);
    if (occupied_voxels.find(key) == occupied_voxels.end()){  // キーがマップ内に存在しない場合、新しいVoxelをマップに追加
      occupied_voxels[key] = Voxel(vx, vy, vz);
    }
  }

  // mapからvectorに変換
  std::vector<Voxel> result;
  for (const auto& pair : occupied_voxels){
    result.push_back(pair.second);
  }

  return result;
}

visualization_msgs::msg::MarkerArray BallDetector::create_voxel_markers(const std::vector<Voxel>& voxels, const std_msgs::msg::Header& header){
  visualization_msgs::msg::MarkerArray marker_array;

  double min_x = this->get_parameter("min_x").as_double();
  double min_y = this->get_parameter("min_y").as_double();
  double min_z = this->get_parameter("min_z").as_double();

  RCLCPP_INFO(this->get_logger(), "voxel size: %d", voxels.size());
  for (size_t i = 0; i < voxels.size(); ++i){
    visualization_msgs::msg::Marker marker;
    marker.header = header;
    marker.ns = "voxel_markers";
    marker.id = i;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = min_x + (voxels[i].x + 0.5) * VOXEL_SIZE_X;
    marker.pose.position.y = min_y + (voxels[i].y + 0.5) * VOXEL_SIZE_Y;
    marker.pose.position.z = min_z + (voxels[i].z + 0.5) * VOXEL_SIZE_Z;
    marker.pose.orientation.w = 1.0;
    // RCLCPP_INFO(this->get_logger(), "min_x: %f  voxels[i].x: %d, VOXEL_SIZE_X: %f", min_x, voxels[i].x, VOXEL_SIZE_X);

    marker.scale.x = VOXEL_SIZE_X;
    marker.scale.y = VOXEL_SIZE_Y;
    marker.scale.z = VOXEL_SIZE_Z;

    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 0.5;

    marker.lifetime = rclcpp::Duration(0, 1e8);  // 0.1 seconds

    marker_array.markers.push_back(marker);
  }

  return marker_array;
}

visualization_msgs::msg::Marker BallDetector::create_detection_area_marker(const std_msgs::msg::Header& header) {
  visualization_msgs::msg::Marker marker;
  marker.header = header;
  marker.ns = "detection_area";
  marker.id = 2;
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
  marker.color.g = 0.0;
  marker.color.b = 1.0;
  marker.color.a = 0.1;

  marker.lifetime = rclcpp::Duration(0, 1e8);  // 0.1 seconds

  return marker;
}

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BallDetector>());
  rclcpp::shutdown();
  return 0;
}
