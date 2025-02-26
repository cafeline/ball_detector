#pragma once

#include "types.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <vector>
#include <memory>

class PointCloudProcessor
{
public:
  PointCloudProcessor(const Parameters &params);

  std::vector<Point3D> process(const sensor_msgs::msg::PointCloud2::SharedPtr &msg, double self_x, double self_y, double self_angle) const;
  sensor_msgs::msg::PointCloud2 vector_to_PC2(const std::vector<Point3D> &points) const;
  std::vector<Point3D> PC2_to_vector(const sensor_msgs::msg::PointCloud2 &cloud_msg) const;
  void rotate_pitch(std::vector<Point3D> &points) const;
  void filter_points_base_origin(double x, double y, double angle, std::vector<Point3D> &points) const;
  void transform_pointcloud(double x, double y, double angle, std::vector<Point3D> &points) const;

private:
  Parameters params_;
};
