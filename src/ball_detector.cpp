#include "ball_detector.hpp"
#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp/logging.hpp>
#include <algorithm>
#include <random>        // 色生成のために追加
#include <unordered_map> // ボクセルベースのクラスタリングのために追加
#include <cmath>
#include <chrono>
#include <queue>
#include <limits>

const float BALL_RADIUS = 0.05;
const int voxel_search_range = 3;

BallDetector::BallDetector() : Node("ball_detector")
{
  subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/livox/lidar", 10, std::bind(&BallDetector::pointcloud_callback, this, std::placeholders::_1));
  marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("occupied_voxels", 10);
  ball_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("ball", 10);
  filtered_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("filtered_pointcloud", 10);
  detection_area_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("bounding_box_marker", 10);

  load_parameters();
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

void BallDetector::pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "***********************************************");
  std::vector<Point3D> points = PC2_to_vector(*msg);
  std::vector<Point3D> filtered_points = filter_points(points);

  auto start_time = std::chrono::high_resolution_clock::now();
  std::vector<VoxelCluster> clusters = create_voxel_clustering(filtered_points);

  std::vector<Point3D> remaining_points = remove_clustered_points(filtered_points, clusters);
  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
  RCLCPP_INFO(this->get_logger(), "Time taken for clustering: %ld ms", duration.count());
  clustered_points_ = std::move(remaining_points);

  visualization_msgs::msg::MarkerArray marker_array = create_voxel_cluster_markers(clusters);

  marker_publisher_->publish(marker_array);

  sensor_msgs::msg::PointCloud2 remaining_cloud = vector_to_PC2(clustered_points_);
  filtered_cloud_publisher_->publish(remaining_cloud);

  // 重心を計算
  RCLCPP_INFO(this->get_logger(), "clustered_points_.size(): %zu", clustered_points_.size());
  Point3D centroid = calculate_centroid(clustered_points_);

  // マーカーを作成
  visualization_msgs::msg::Marker centroid_marker = create_ball_marker(centroid, remaining_cloud.header);

  // マーカーをパブリッシュ
  visualization_msgs::msg::MarkerArray ball_marker_array;
  ball_marker_array.markers.push_back(centroid_marker);
  ball_publisher_->publish(ball_marker_array);
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
  Point3D centroid = {0.0f, 0.0f, 0.0f};
  if (points.empty())
  {
    return centroid;
  }

  for (const auto &point : points)
  {
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

visualization_msgs::msg::Marker BallDetector::create_ball_marker(const Point3D &centroid, const std_msgs::msg::Header &header)
{
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
  marker.lifetime = rclcpp::Duration(0, 1e8); // 0.1 seconds
  return marker;
}

std::vector<Voxel> BallDetector::create_voxel(const std::vector<Point3D> &points)
{
  std::unordered_map<std::string, Voxel> occupied_voxels;
  for (const auto &point : points)
  {
    int vx = static_cast<int>((point.x - params_.min_x) / params_.voxel_size_x);
    int vy = static_cast<int>((point.y - params_.min_y) / params_.voxel_size_y);
    int vz = static_cast<int>((point.z - params_.min_z) / params_.voxel_size_z);
    std::string key = std::to_string(vx) + "," + std::to_string(vy) + "," + std::to_string(vz);
    if (occupied_voxels.find(key) == occupied_voxels.end())
    { // キがマップ内に存在しない場合、新しいVoxelをマップに追加
      occupied_voxels[key] = Voxel(vx, vy, vz);
    }
  }

  // mapからvectorに変換
  std::vector<Voxel> result;
  for (const auto &pair : occupied_voxels)
  {
    result.push_back(pair.second);
  }

  return result;
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

    // ボクセルの中心へのオフセットを計算 (ボクセルの左下前→中心)
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

    marker.lifetime = rclcpp::Duration(0, 1e8); // 0.1 seconds

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

  marker.lifetime = rclcpp::Duration(0, 1e8); // 0.1 seconds

  return marker;
}

std::vector<VoxelCluster> BallDetector::create_voxel_clustering(const std::vector<Point3D> &points)
{
  std::unordered_map<std::string, Voxel> occupied_voxels;

  // ボクセルの占有をマップに記録し、点群数をカウント
  for (const auto &point : points)
  {
    int vx = static_cast<int>((point.x - params_.min_x) / params_.voxel_size_x);
    int vy = static_cast<int>((point.y - params_.min_y) / params_.voxel_size_y);
    int vz = static_cast<int>((point.z - params_.min_z) / params_.voxel_size_z);
    std::string key = std::to_string(vx) + "," + std::to_string(vy) + "," + std::to_string(vz);
    RCLCPP_DEBUG(this->get_logger(), "key: %s", key.c_str());

    if (occupied_voxels.find(key) == occupied_voxels.end())
    {
      occupied_voxels[key] = Voxel(vx, vy, vz);
    }
    else
    {
      occupied_voxels[key].increment();
    }
  }

  // クラスタリングの開始
  std::vector<VoxelCluster> clusters;
  std::unordered_map<std::string, bool> visited;

  for (const auto &pair : occupied_voxels)
  {
    const std::string &key = pair.first;
    if (visited.find(key) != visited.end())
    {
      continue; // 既に訪問済み
    }

    // 新しいクラスタの開始
    VoxelCluster cluster;
    std::queue<std::string> q;
    q.push(key);
    visited[key] = true;

    while (!q.empty())
    {
      std::string current_key = q.front();
      q.pop();

      // 現在のボクセルをクラスタに追加
      cluster.voxels.push_back(occupied_voxels[current_key]);

      // 現在のボクセルの隣接ボクセルを探索
      int cx, cy, cz;
      sscanf(current_key.c_str(), "%d,%d,%d", &cx, &cy, &cz);
      for (int dx = -voxel_search_range; dx <= voxel_search_range; ++dx)
      {
        for (int dy = -voxel_search_range; dy <= voxel_search_range; ++dy)
        {
          for (int dz = -voxel_search_range; dz <= voxel_search_range; ++dz)
          {
            if (dx == 0 && dy == 0 && dz == 0)
            {
              continue; // 自身はスキップ
            }
            int nx = cx + dx;
            int ny = cy + dy;
            int nz = cz + dz;
            std::string neighbor_key = std::to_string(nx) + "," + std::to_string(ny) + "," + std::to_string(nz);

            // 隣接ボクセルが占有されており、未訪問の場合
            if (occupied_voxels.find(neighbor_key) != occupied_voxels.end() && visited.find(neighbor_key) == visited.end())
            {
              q.push(neighbor_key);
              visited[neighbor_key] = true;
            }
          }
        }
      }
    }

    // クラスタ内の点群のサイズを点群の座標に基づいて計算
    double min_x = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::lowest();
    double min_y = std::numeric_limits<double>::max();
    double max_y = std::numeric_limits<double>::lowest();
    double min_z = std::numeric_limits<double>::max();
    double max_z = std::numeric_limits<double>::lowest();

    // クラスタに属する全ての点を取得
    for (const auto &point : points)
    {
      // 点がクラスタのボクセル内に属しているか確認
      bool in_cluster = false;
      for (const auto &voxel : cluster.voxels)
      {
        if (point.x >= params_.min_x + voxel.x * params_.voxel_size_x &&
            point.x < params_.min_x + (voxel.x + 1) * params_.voxel_size_x &&
            point.y >= params_.min_y + voxel.y * params_.voxel_size_y &&
            point.y < params_.min_y + (voxel.y + 1) * params_.voxel_size_y &&
            point.z >= params_.min_z + voxel.z * params_.voxel_size_z &&
            point.z < params_.min_z + (voxel.z + 1) * params_.voxel_size_z)
        {
          in_cluster = true;
          break;
        }
      }
      if (in_cluster)
      {
        if (point.x < min_x)
          min_x = point.x;
        if (point.x > max_x)
          max_x = point.x;
        if (point.y < min_y)
          min_y = point.y;
        if (point.y > max_y)
          max_y = point.y;
        if (point.z < min_z)
          min_z = point.z;
        if (point.z > max_z)
          max_z = point.z;
      }
    }

    double size_x = max_x - min_x;
    double size_y = max_y - min_y;
    double size_z = max_z - min_z;

    // クラスタの条件を評価（点群のサイズと点群数）
    if (size_x <= 2 * BALL_RADIUS && size_y <= 2 * BALL_RADIUS && size_z <= 2 * BALL_RADIUS && cluster.voxels.size() > 1)
    {
      cluster.total_point_count = cluster.voxels.size(); // 合計点群数を記録
      clusters.push_back(cluster);
      RCLCPP_ERROR(this->get_logger(), "cluster size: %f, %f, %f, point count: %d", size_x, size_y, size_z, cluster.voxels.size());
    }
  }

  return clusters;
}

std::vector<Point3D> BallDetector::remove_clustered_points(const std::vector<Point3D> &original_points, const std::vector<VoxelCluster> &clusters)
{
  std::vector<Point3D> remaining_points;
  std::unordered_set<std::string> clustered_points;

  // クラスタリングされた点をセットに追加
  for (const auto &cluster : clusters)
  {
    for (const auto &voxel : cluster.voxels)
    {
      std::string key = std::to_string(voxel.x) + "," + std::to_string(voxel.y) + "," + std::to_string(voxel.z);
      clustered_points.insert(key);
    }
  }

  // クラスタリングされていない点を残りの点群に追加
  for (const auto &point : original_points)
  {
    int vx = static_cast<int>((point.x - params_.min_x) / params_.voxel_size_x);
    int vy = static_cast<int>((point.y - params_.min_y) / params_.voxel_size_y);
    int vz = static_cast<int>((point.z - params_.min_z) / params_.voxel_size_z);
    std::string key = std::to_string(vx) + "," + std::to_string(vy) + "," + std::to_string(vz);

    if (clustered_points.find(key) == clustered_points.end())
    {
      remaining_points.push_back(point);
    }
  }

  return remaining_points;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BallDetector>());
  rclcpp::shutdown();
  return 0;
}
