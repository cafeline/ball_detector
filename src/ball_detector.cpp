#include "ball_detector.hpp"
#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp/logging.hpp>
#include <algorithm>
#include <random>  // 色生成のために追加
#include <unordered_map>  // ボクセルベースのクラスタリングのために追加
#include <cmath>  // ボクセルベースのクラスタリングのために追加
#include <chrono>
#include <queue>

BallDetector::BallDetector() : Node("ball_detector") {
  subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/livox/lidar", 10, std::bind(&BallDetector::pointcloud_callback, this, std::placeholders::_1));
  marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("occupied_voxels", 10);
  filtered_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("filtered_pointcloud", 10);
  detection_area_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("bounding_box_marker", 10);

  load_parameters();
}

void BallDetector::load_parameters(){
  this->declare_parameter("min_x", -10.0);
  this->declare_parameter("max_x", 10.0);
  this->declare_parameter("min_y", -10.0);
  this->declare_parameter("max_y", 10.0);
  this->declare_parameter("min_z", -2.0);
  this->declare_parameter("max_z", 5.0);
  this->declare_parameter("voxel_size_x", 0.1);
  this->declare_parameter("voxel_size_y", 0.1);
  this->declare_parameter("voxel_size_z", 0.1);
  params_.min_x = this->get_parameter("min_x").as_double();
  params_.max_x = this->get_parameter("max_x").as_double();
  params_.min_y = this->get_parameter("min_y").as_double();
  params_.max_y = this->get_parameter("max_y").as_double();
  params_.min_z = this->get_parameter("min_z").as_double();
  params_.max_z = this->get_parameter("max_z").as_double();
  params_.voxel_size_x = this->get_parameter("voxel_size_x").as_double();
  params_.voxel_size_y = this->get_parameter("voxel_size_y").as_double();
  params_.voxel_size_z = this->get_parameter("voxel_size_z").as_double();
}

void BallDetector::pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "************************");
  std::vector<Point3D> points = PC2_to_vector(*msg);

  // 指定領域内の点群を抽出
  std::vector<Point3D> filtered_points = filter_points(points);

  auto start_time = std::chrono::steady_clock::now();
  // ボクセルベースのクラスタリングを実行
  std::vector<VoxelCluster> clusters = create_voxel_clustering(filtered_points);
  RCLCPP_INFO(this->get_logger(), "Voxel clustering time: %ld microseconds", std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start_time).count());

  // マーカーを作成してパブリッシュ
  std_msgs::msg::Header header;
  header.stamp = this->get_clock()->now();
  header.frame_id = frame_id_;

  // 占有されたボクセルをマーカーとしてパブリッシュ
  auto voxel_markers = create_voxel_cluster_markers(clusters, header);
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
    memcpy(&point.x, &cloud_msg.data[data_index + x_offset], sizeof(float));
    memcpy(&point.y, &cloud_msg.data[data_index + y_offset], sizeof(float));
    memcpy(&point.z, &cloud_msg.data[data_index + z_offset], sizeof(float));
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
  for (const auto& point : input) {
    if (point.x != 0.0f && point.y != 0.0f && point.z != 0.0f &&
      point.x >= params_.min_x && point.x <= params_.max_x &&
      point.y >= params_.min_y && point.y <= params_.max_y &&
      point.z >= params_.min_z && point.z <= params_.max_z){
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

std::vector<Voxel> BallDetector::create_voxel(const std::vector<Point3D>& points){
  std::unordered_map<std::string, Voxel> occupied_voxels;
  for (const auto& point : points){
    int vx = static_cast<int>((point.x - params_.min_x) / params_.voxel_size_x);
    int vy = static_cast<int>((point.y - params_.min_y) / params_.voxel_size_y);
    int vz = static_cast<int>((point.z - params_.min_z) / params_.voxel_size_z);
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
  RCLCPP_INFO(this->get_logger(), "voxel size: %d", voxels.size());
  for (size_t i = 0; i < voxels.size(); ++i){
    visualization_msgs::msg::Marker marker;
    marker.header = header;
    marker.ns = "voxel_markers";
    marker.id = i;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = params_.min_x + (voxels[i].x + 0.5) * params_.voxel_size_x;
    marker.pose.position.y = params_.min_y + (voxels[i].y + 0.5) * params_.voxel_size_y;
    marker.pose.position.z = params_.min_z + (voxels[i].z + 0.5) * params_.voxel_size_z;
    marker.pose.orientation.w = 1.0;
    // RCLCPP_INFO(this->get_logger(), "params_.min_x: %f  voxels[i].x: %d, params_.voxel_size_x: %f", params_.min_x, voxels[i].x, params_.voxel_size_x);

    marker.scale.x = params_.voxel_size_x;
    marker.scale.y = params_.voxel_size_y;
    marker.scale.z = params_.voxel_size_z;

    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 0.5;

    marker.lifetime = rclcpp::Duration(0, 1e8);  // 0.1 seconds

    marker_array.markers.push_back(marker);
  }

  return marker_array;
}

visualization_msgs::msg::MarkerArray BallDetector::create_voxel_cluster_markers(const std::vector<VoxelCluster>& clusters, const std_msgs::msg::Header& header) {
    visualization_msgs::msg::MarkerArray marker_array;
    RCLCPP_INFO(this->get_logger(), "Number of clusters: %zu", clusters.size());

    // クラスタごとに異なる色を生成するためのランダムジェネレータ
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0.0, 1.0);

    for (size_t i = 0; i < clusters.size(); ++i) {
        const auto& cluster = clusters[i];
        
        // クラスタごとにランダムな色を生成
        float r = dis(gen);
        float g = dis(gen);
        float b = dis(gen);

        for (const auto& voxel : cluster.voxels) {
            visualization_msgs::msg::Marker marker;
            marker.header = header;
            marker.ns = "voxel_cluster_markers";
            marker.id = i * 1000 + marker_array.markers.size(); // ユニークなIDを生成
            marker.type = visualization_msgs::msg::Marker::CUBE;
            marker.action = visualization_msgs::msg::Marker::ADD;

            marker.pose.position.x = params_.min_x + (voxel.x + 0.5) * params_.voxel_size_x;
            marker.pose.position.y = params_.min_y + (voxel.y + 0.5) * params_.voxel_size_y;
            marker.pose.position.z = params_.min_z + (voxel.z + 0.5) * params_.voxel_size_z;
            marker.pose.orientation.w = 1.0;

            marker.scale.x = params_.voxel_size_x;
            marker.scale.y = params_.voxel_size_y;
            marker.scale.z = params_.voxel_size_z;

            // クラスタごとの色を設定
            marker.color.r = r;
            marker.color.g = g;
            marker.color.b = b;
            marker.color.a = 0.8; // 少し透明に

            marker.lifetime = rclcpp::Duration(0, 1e8);  // 約0.1秒

            marker_array.markers.push_back(marker);
        }
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

  marker.lifetime = rclcpp::Duration(0, 1e8);  // 0.1 seconds

  return marker;
}

void BallDetector::publish_clusters(const std::vector<VoxelCluster>& clusters) {
    visualization_msgs::msg::MarkerArray marker_array;

    for (size_t i = 0; i < clusters.size(); ++i) {
        VoxelCluster cluster = clusters[i];

        // クラスタの中心点を計算
        double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
        for (const auto& voxel : cluster.voxels) {
            sum_x += voxel.x;
            sum_y += voxel.y;
            sum_z += voxel.z;
        }
        double center_x = params_.min_x + (sum_x / cluster.voxels.size() + 0.5) * params_.voxel_size_x;
        double center_y = params_.min_y + (sum_y / cluster.voxels.size() + 0.5) * params_.voxel_size_y;
        double center_z = params_.min_z + (sum_z / cluster.voxels.size() + 0.5) * params_.voxel_size_z;

        // マーカーの作成
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.ns = "clusters";
        marker.id = i;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = center_x;
        marker.pose.position.y = center_y;
        marker.pose.position.z = center_z;
        marker.pose.orientation.w = 1.0;

        // マーカーのスケール（例：ボクセルサイズと同じ）
        marker.scale.x = params_.voxel_size_x;
        marker.scale.y = params_.voxel_size_y;
        marker.scale.z = params_.voxel_size_z;

        // マーカーの色
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0f;

        // ライフタイムの設定（無限）
        marker.lifetime = rclcpp::Duration(0, 0);

        // マーカーを配列に追加
        marker_array.markers.push_back(marker);
    }

    // マーカーをパブリッシュ
    marker_publisher_->publish(marker_array);
}

std::vector<VoxelCluster> BallDetector::create_voxel_clustering(const std::vector<Point3D>& points) {
  // 実行時間計測開始
  auto start = std::chrono::steady_clock::now();
  std::unordered_map<std::string, Voxel> occupied_voxels;
  // ボクセルの占有をマップに記録
  for (const auto& point : points) {
    int vx = static_cast<int>((point.x - params_.min_x) / params_.voxel_size_x);
    int vy = static_cast<int>((point.y - params_.min_y) / params_.voxel_size_y);
    int vz = static_cast<int>((point.z - params_.min_z) / params_.voxel_size_z);
    std::string key = std::to_string(vx) + "," + std::to_string(vy) + "," + std::to_string(vz);
    RCLCPP_DEBUG(this->get_logger(), "key: %s", key.c_str());
    if (occupied_voxels.find(key) == occupied_voxels.end()) {
      occupied_voxels[key] = Voxel(vx, vy, vz);
    }
  }
  // クラスタリングの開始
  std::vector<VoxelCluster> clusters;
  std::unordered_map<std::string, bool> visited;
  for (const auto& pair : occupied_voxels) {
    const std::string& key = pair.first;
    if (visited.find(key) != visited.end()) {
      continue; // 既に訪問済み
    }
    // 新しいクラスタの開始
    VoxelCluster cluster;
    std::queue<std::string> q;
    q.push(key);
    visited[key] = true;
    while (!q.empty()) {
      std::string current_key = q.front();
      q.pop();
     // 現在のボクセルをクラスタに追加
      cluster.voxels.push_back(occupied_voxels[current_key]);
     // 現在のボクセルの隣接ボクセルを探索
      int cx, cy, cz;
      sscanf(current_key.c_str(), "%d,%d,%d", &cx, &cy, &cz);
      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          for (int dz = -1; dz <= 1; ++dz) {
            if (dx == 0 && dy == 0 && dz == 0) {
              continue; // 自身はスキップ
            }
            int nx = cx + dx;
            int ny = cy + dy;
            int nz = cz + dz;
            std::string neighbor_key = std::to_string(nx) + "," + std::to_string(ny) + "," + std::to_string(nz);
            // 隣接ボクセルが占有されており、未訪問の場合
            if (occupied_voxels.find(neighbor_key) != occupied_voxels.end() &&
              visited.find(neighbor_key) == visited.end()) {
              q.push(neighbor_key);
              visited[neighbor_key] = true;
            }
          }
        }
      }
    }
    // クラスタが存在する場合に追加
    if (!cluster.voxels.empty()) {
      clusters.push_back(cluster);
    }
  }

  // 実行時間計測終了
  auto end = std::chrono::steady_clock::now();
  auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
  RCLCPP_INFO(this->get_logger(), "voxel_clustering 実行時間: %ld ms, Total Clusters Found: %zu", elapsed, clusters.size());
  return clusters;
}

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BallDetector>());
  rclcpp::shutdown();
  return 0;
}
