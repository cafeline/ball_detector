#include "ball_detector/pointcloud_processor.hpp"

#include <algorithm>
#include <unordered_map>
#include <cmath>
#include <cstring>
#include <chrono>
#include <rclcpp/rclcpp.hpp>

PointCloudProcessor::PointCloudProcessor(const Parameters &params) : params_(params) {}

std::vector<Point3D> PointCloudProcessor::process(const sensor_msgs::msg::PointCloud2::SharedPtr &msg, double self_x, double self_y, double self_angle) const
{
  auto points = PC2_to_vector(*msg);
  rotate_pitch(points);
  filter_points_base_origin(self_x, self_y, self_angle, points);
  transform_pointcloud(self_x, self_y, self_angle, points);
  return points;
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

void PointCloudProcessor::rotate_pitch(std::vector<Point3D> &points) const
{
  double angle_rad = params_.livox_pitch * M_PI / 180.0;
  double cos_angle = std::cos(angle_rad);
  double sin_angle = std::sin(angle_rad);

  // 各点を直接更新
  for (auto &point : points)
  {
    float old_x = point.x;
    float old_z = point.z;
    point.x = old_x * cos_angle + old_z * sin_angle;
    point.z = -old_x * sin_angle + old_z * cos_angle;
    // point.y は変化なし
  }
}

void PointCloudProcessor::filter_points_base_origin(double x, double y, double angle, std::vector<Point3D> &points) const
{
  double cos_angle = std::cos(angle);
  double sin_angle = std::sin(angle);

  double temp_min_x = params_.min_x - x;
  double temp_min_y = params_.min_y - y;
  double temp_max_x = params_.max_x - x;
  double temp_max_y = params_.max_y - y;
  double temp_min_z = params_.min_z;
  double temp_max_z = params_.max_z;
  const double exclusion_radius_sq = 1.0 * 1.0;

  auto new_end = std::remove_if(points.begin(), points.end(), [&](const Point3D &point)
                                {
        double rotated_x = point.x * cos_angle + point.y * sin_angle;
        double rotated_y = point.x * sin_angle - point.y * cos_angle;
        double rotated_z = point.z;

        bool within_bounds = (rotated_x != 0.0 && rotated_y != 0.0 && rotated_z != 0.0) &&
                             (rotated_x >= temp_min_x && rotated_x <= temp_max_x) &&
                             (rotated_y >= temp_min_y && rotated_y <= temp_max_y) &&
                             (rotated_z >= temp_min_z && rotated_z <= temp_max_z);

        double distance_squared = point.x * point.x + point.y * point.y + point.z * point.z;
        bool outside_exclusion = (distance_squared >= exclusion_radius_sq);

        // 条件を満たさない場合は削除
        return !(within_bounds && outside_exclusion); });

  points.erase(new_end, points.end());
}

void PointCloudProcessor::transform_pointcloud(double x, double y, double angle, std::vector<Point3D> &points) const
{
  double cos_angle = std::cos(angle);
  double sin_angle = std::sin(angle);

  // 各点を回転＆平行移動
  for (auto &point : points)
  {
    float old_x = point.x;
    float old_y = point.y;
    point.x = old_x * cos_angle - old_y * sin_angle + x;
    point.y = old_x * sin_angle + old_y * cos_angle + y;
    // point.z は変化なし
  }
}