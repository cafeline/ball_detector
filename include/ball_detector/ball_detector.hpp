#ifndef BALL_DETECTOR_HPP
#define BALL_DETECTOR_HPP

#include "ball_detector/visibility.h"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <vector>
#include <deque>
#include <cstring>
#include <visualization_msgs/msg/marker.hpp>
#include <array>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "ball_detector/clustering.hpp"
#include "pointcloud_processor/types.hpp"

namespace ball_detector
{
  class BallDetector : public rclcpp::Node
  {
  public:
    BALL_DETECTOR_PUBLIC
    explicit BallDetector(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    BALL_DETECTOR_PUBLIC
    explicit BallDetector(const std::string &name_space, const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  private:
    void load_parameters();
    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    std::vector<Point3D> PC2_to_vector(const sensor_msgs::msg::PointCloud2 &cloud_msg);
    std::vector<Point3D> filter_points(const std::vector<Point3D> &input);
    std::vector<Point3D> voxel_downsample(const std::vector<Point3D> &input);
    Point3D calculate_centroid(const std::vector<Point3D> &points);
    Point3D calculate_cluster_centroid(const VoxelCluster &cluster);
    visualization_msgs::msg::Marker create_ball_marker(const Point3D &centroid, const std_msgs::msg::Header &header);
    visualization_msgs::msg::MarkerArray create_voxel_markers(const std::vector<Voxel> &voxels, const std_msgs::msg::Header &header);
    visualization_msgs::msg::MarkerArray create_voxel_cluster_markers(const std::vector<VoxelCluster> &clusters);
    visualization_msgs::msg::Marker create_detection_area_marker(const std_msgs::msg::Header &header);
    visualization_msgs::msg::Marker create_trajectory_marker(const std::deque<Point3D> &trajectory, const std_msgs::msg::Header &header);
    visualization_msgs::msg::Marker create_past_points_marker(const std::deque<Point3D> &past_points, const std_msgs::msg::Header &header);
    std::vector<Point3D> remove_clustered_points(const std::vector<Point3D> &original_points, const std::vector<VoxelCluster> &clusters);
    void collect_cluster_points(VoxelCluster &cluster, const std::vector<Point3D> &points);
    std::vector<VoxelCluster> process_clustering(const std::vector<Point3D> &points);
    void publish_markers(const std::vector<VoxelCluster> &clusters, const sensor_msgs::msg::PointCloud2 &remaining_cloud);
    void update_trajectory(const std::vector<VoxelCluster> &clusters, const sensor_msgs::msg::PointCloud2 &remaining_cloud);
    std::vector<Point3D> axis_image2robot(const std::vector<Point3D> &input);
    visualization_msgs::msg::Marker create_custom_marker(const Point3D &point, const std_msgs::msg::Header &header);
    void detect_human(const std::vector<Point3D> &points);

    // メンバー変数
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr clustered_voxel_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr ball_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_cloud_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr trajectory_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr past_points_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr human_publisher_;
    std::string frame_id_ = "map";

    Parameters params_;

    std::vector<Point3D> clustered_points_;
    std::deque<Point3D> ball_trajectory_points_;
    std::deque<Point3D> past_points_;

    // VoxelProcessor と Clustering のインスタンス
    std::unique_ptr<VoxelProcessor> voxel_processor_;
    std::unique_ptr<Clustering> clustering_;

    Point3D self_pose_;
    double livox_pitch_=0.0;
  };
}
#endif // BALL_DETECTOR_HPP
