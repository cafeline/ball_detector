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
#include "ball_detector/pointcloud_processor.hpp"
#include <std_msgs/msg/bool.hpp>

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
    void autonomous_callback(const std_msgs::msg::Bool::SharedPtr msg);

    std::vector<Point3D> preprocess_pointcloud(const sensor_msgs::msg::PointCloud2 &msg);

    visualization_msgs::msg::Marker create_ball_marker(const Point3D &centroid, const std_msgs::msg::Header &header);
    visualization_msgs::msg::MarkerArray create_voxel_cluster_markers(const std::vector<VoxelCluster> &clusters);
    visualization_msgs::msg::Marker create_trajectory_marker(const std::deque<Point3D> &trajectory, const std_msgs::msg::Header &header);
    visualization_msgs::msg::Marker create_past_points_marker(const std::deque<Point3D> &past_points, const std_msgs::msg::Header &header);

    void publish_markers(Point3D &ball_position, const std::vector<VoxelCluster> &clusters, const sensor_msgs::msg::PointCloud2 &remaining_cloud);
    Point3D calculate_ball_position(const std::vector<VoxelCluster> &clusters);
    void update_trajectory(const Point3D &centroid, const sensor_msgs::msg::PointCloud2 &remaining_cloud);

    std::vector<Point3D> previous_centroids_;
    rclcpp::Time previous_time_;
    std::mutex centroid_mutex_; // スレッドセーフのためのミューテックス

    // メンバー変数
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr autonomous_subscription_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr clustered_voxel_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr ball_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_cloud_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr trajectory_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr past_points_publisher_;
    std::string frame_id_ = "map";

    Parameters params_;

    std::vector<Point3D> clustered_points_;
    std::deque<Point3D> ball_trajectory_points_;
    std::deque<Point3D> past_points_;

    // VoxelProcessor と Clustering のインスタンス
    std::unique_ptr<VoxelProcessor> voxel_processor_;
    std::unique_ptr<Clustering> clustering_;

    // Clusteringが管理するインデックスを利用する
    // ボール,動的クラスタなどのインデックスはclustering_経由で取得
    Point3D self_pose_;
    double livox_pitch_ = 0.0;
    double ball_vel_min_ = 0.0;
    double max_distance_for_association_ = 0.0;

    bool is_autonomous = false;
  };
}
#endif // BALL_DETECTOR_HPP
