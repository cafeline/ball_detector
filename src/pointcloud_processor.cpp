#include "ball_detector/pointcloud_processor.hpp"
#include <unordered_map>
#include <cmath>
#include <cstring>
#include <chrono>
#include <rclcpp/rclcpp.hpp>

PointCloudProcessor::PointCloudProcessor(const Parameters &params) : params_(params) {}

std::vector<Point3D> PointCloudProcessor::process_pointcloud_old(const sensor_msgs::msg::PointCloud2 &cloud_msg)
{
  auto points = PC2_to_vector(cloud_msg);
  auto transformed_points = axis_image2robot(points);
  auto filtered_points = filter_points(transformed_points);
  voxel_downsample(filtered_points);
  return filtered_points;
}

std::vector<Point3D> PointCloudProcessor::process_pointcloud(const sensor_msgs::msg::PointCloud2 &cloud_msg)
{
  size_t num_points = cloud_msg.width * cloud_msg.height;
  std::vector<Point3D> processed_points;
  processed_points.reserve(num_points);

  const size_t point_step = cloud_msg.point_step;
  const size_t x_offset = cloud_msg.fields[0].offset;
  const size_t y_offset = cloud_msg.fields[1].offset;
  const size_t z_offset = cloud_msg.fields[2].offset;

  // PC2_to_vector、axis_image2robot、filter_points を統合して効率化
  for (size_t i = 0; i < num_points; ++i)
  {
    size_t data_index = i * point_step;
    Point3D point;
    memcpy(&point.x, &cloud_msg.data[data_index + x_offset], sizeof(float));
    memcpy(&point.y, &cloud_msg.data[data_index + y_offset], sizeof(float));
    memcpy(&point.z, &cloud_msg.data[data_index + z_offset], sizeof(float));

    // 座標変換
    Point3D transformed;
    transformed.x = point.z;
    transformed.y = point.x;
    transformed.z = -point.y;

    // フィルタリング
    if (transformed.x != 0.0f && transformed.y != 0.0f && transformed.z != 0.0f &&
        transformed.x >= params_.min_x && transformed.x <= params_.max_x &&
        transformed.y >= params_.min_y && transformed.y <= params_.max_y &&
        transformed.z >= params_.min_z && transformed.z <= params_.max_z)
    {
      processed_points.emplace_back(transformed);
    }
  }
  voxel_downsample(processed_points);
  return downsampled_points_;
}

std::vector<Point3D> PointCloudProcessor::get_downsampled_points() const
{
  return downsampled_points_;
}

sensor_msgs::msg::PointCloud2 PointCloudProcessor::vector_to_PC2(const std::vector<Point3D> &points) const
{
  sensor_msgs::msg::PointCloud2 cloud_msg;
  cloud_msg.header.frame_id = "map";
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

std::vector<Point3D> PointCloudProcessor::PC2_to_vector(const sensor_msgs::msg::PointCloud2 &cloud_msg) const
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

std::vector<Point3D> PointCloudProcessor::rotate_pitch(const std::vector<Point3D> &input, int degree) const
{
  std::vector<Point3D> rotated_points;
  rotated_points.reserve(input.size());

  // 30度をラジアンに変換
  double angle_rad = degree * M_PI / 180.0;

  // ピッチ方向（Y軸）への回転行列
  double cos_angle = std::cos(angle_rad);
  double sin_angle = std::sin(angle_rad);

  for (const auto &point : input)
  {
    Point3D rotated;
    rotated.x = point.x * cos_angle + point.z * sin_angle;
    rotated.y = point.y;
    rotated.z = -point.x * sin_angle + point.z * cos_angle;
    rotated_points.emplace_back(rotated);
  }

  return rotated_points;
}

std::vector<Point3D> PointCloudProcessor::axis_image2robot(const std::vector<Point3D> &input) const
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

std::vector<Point3D> PointCloudProcessor::filter_points_base_origin(double x, double y, double angle, const std::vector<Point3D> &input) const
{
  // 回転行列の計算
  double cos_angle = std::cos(angle);
  double sin_angle = std::sin(angle);

  // ロボットの位置から抽出範囲を設定
  double temp_min_x = params_.min_x - x;
  double temp_min_y = params_.min_y - y;
  double temp_max_x = params_.max_x - x;
  double temp_max_y = params_.max_y - y;
  double temp_min_z = params_.min_z;
  double temp_max_z = params_.max_z;

  // 除去したい半径
  const double exclusion_radius = 1.0;

  std::vector<Point3D> output;
  output.reserve(input.size());

  for (const auto &point : input)
  {
    // ロボットの角度に合わせて点を回転
    double rotated_x = point.x * cos_angle + point.y * sin_angle;
    double rotated_y = -(-point.x * sin_angle + point.y * cos_angle);
    double rotated_z = point.z;

    // フィルタリング条件の適用
    bool within_bounds = rotated_x != 0.0f && rotated_y != 0.0f && rotated_z != 0.0f &&
                         rotated_x >= temp_min_x && rotated_x <= temp_max_x &&
                         rotated_y >= temp_min_y && rotated_y <= temp_max_y &&
                         rotated_z >= temp_min_z && rotated_z <= temp_max_z;

    // 距離による除去条件の追加
    double distance_squared = point.x * point.x + point.y * point.y + point.z * point.z;
    bool outside_exclusion = distance_squared >= (exclusion_radius * exclusion_radius);

    if (within_bounds && outside_exclusion)
    {
      output.emplace_back(point);
    }
  }
  return output;
}

std::vector<Point3D> PointCloudProcessor::transform_pointcloud(double x, double y, double angle, const std::vector<Point3D> &input) const
{
  std::vector<Point3D> transformed_points;
  transformed_points.reserve(input.size());

  double cos_angle = std::cos(angle);
  double sin_angle = std::sin(angle);

  for (const auto &point : input)
  {
    Point3D transformed;
    // 回転
    transformed.x = point.x * cos_angle - point.y * sin_angle;
    transformed.y = point.x * sin_angle + point.y * cos_angle;
    transformed.z = point.z;

    // 平行移動
    transformed.x += x;
    transformed.y += y;

    transformed_points.emplace_back(transformed);
  }

  return transformed_points;
}

std::vector<Point3D> PointCloudProcessor::filter_points_pre(const std::vector<Point3D> &input) const
{
  double min_x = 0.0;
  double max_x = 3.0;
  std::vector<Point3D> output;
  for (const auto &point : input)
  {
    if (point.x != 0.0f && point.y != 0.0f && point.z != 0.0f &&
        point.x >= min_x && point.x <= max_x &&
        point.y >= params_.min_y && point.y <= params_.max_y &&
        point.z >= params_.min_z && point.z <= params_.max_z)
    {
      output.push_back(point);
    }
  }
  return output;
}

std::vector<Point3D> PointCloudProcessor::filter_points(const std::vector<Point3D> &input) const
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
// void PointCloudProcessor::voxel_downsample(const std::vector<Point3D> &input)
// {
//   std::unordered_map<std::string, std::vector<Point3D>> voxel_map;
//   downsampled_points_.clear();

//   for (const auto &point : input)
//   {
//     int voxel_x = static_cast<int>(std::floor(point.x / params_.D_voxel_size_x));
//     int voxel_y = static_cast<int>(std::floor(point.y / params_.D_voxel_size_y));
//     int voxel_z = static_cast<int>(std::floor(point.z / params_.D_voxel_size_z));
//     std::string key = std::to_string(voxel_x) + "_" + std::to_string(voxel_y) + "_" + std::to_string(voxel_z);
//     voxel_map[key].push_back(point);
//   }

//   for (const auto &voxel : voxel_map)
//   {
//     const auto &points = voxel.second;
//     if (points.size() < 4)
//       continue;

//     float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;
//     for (const auto &p : points)
//     {
//       sum_x += p.x;
//       sum_y += p.y;
//       sum_z += p.z;
//     }

//     Point3D centroid;
//     centroid.x = sum_x / points.size();
//     centroid.y = sum_y / points.size();
//     centroid.z = sum_z / points.size();
//     downsampled_points_.push_back(centroid);
//   }
// }

void PointCloudProcessor::voxel_downsample(const std::vector<Point3D> &input)
{
  struct Voxel
  {
    int x, y, z;

    bool operator==(const Voxel &other) const
    {
      return x == other.x && y == other.y && z == other.z;
    }
  };

  struct VoxelHash
  {
    std::size_t operator()(const Voxel &voxel) const
    {
      return std::hash<int>()(voxel.x) ^ (std::hash<int>()(voxel.y) << 1) ^ (std::hash<int>()(voxel.z) << 2);
    }
  };

  std::unordered_map<Voxel, std::vector<Point3D>, VoxelHash> voxel_map;
  voxel_map.reserve(input.size() / 4); // 予備サイズを設定
  downsampled_points_.clear();
  downsampled_points_.reserve(input.size() / 4); // 予備サイズを設定

  for (const auto &point : input)
  {
    Voxel voxel;
    voxel.x = static_cast<int>(std::floor(point.x / params_.D_voxel_size_x));
    voxel.y = static_cast<int>(std::floor(point.y / params_.D_voxel_size_y));
    voxel.z = static_cast<int>(std::floor(point.z / params_.D_voxel_size_z));
    voxel_map[voxel].emplace_back(point);
  }

  downsampled_points_.reserve(voxel_map.size());

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
    downsampled_points_.emplace_back(centroid);
  }
}