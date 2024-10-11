#include "ball_detector.hpp"
#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp/logging.hpp>

BallDetector::BallDetector() : Node("ball_detector")
{
  subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/livox/lidar", 10, std::bind(&BallDetector::pointcloud_callback, this, std::placeholders::_1));
  
  marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("ball_marker", 10);
  filtered_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("filtered_pointcloud", 10);

  this->declare_parameter("min_x", 0.0);
  this->declare_parameter("max_x", 3.0);
  this->declare_parameter("min_y", -0.5);
  this->declare_parameter("max_y", 0.5);
  this->declare_parameter("min_z", 0.0);
  this->declare_parameter("max_z", 1.0);
}

void BallDetector::pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  std::vector<Point3D> points = PC2_to_vector(*msg);

  // 指定領域内の点群を抽出
  std::vector<Point3D> filtered_points;
  filter_points(points, filtered_points);

  // 重心位置を計算
  Point3D centroid = calculate_centroid(filtered_points);

  // マーカーを作成してパブリッシュ
  auto marker = create_ball_marker(centroid, msg->header);
  marker_publisher_->publish(marker);

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

visualization_msgs::msg::Marker BallDetector::create_ball_marker(const Point3D& centroid, const std_msgs::msg::Header& header)
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
  marker.lifetime = rclcpp::Duration(0, 1e8);  // 0.1 seconds
  return marker;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BallDetector>());
  rclcpp::shutdown();
  return 0;
}