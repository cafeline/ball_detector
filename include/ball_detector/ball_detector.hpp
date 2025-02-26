#ifndef BALL_DETECTOR_HPP
#define BALL_DETECTOR_HPP

#include "ball_detector/visibility.h"

#include <vector>
#include <deque>
#include <cstring>
#include <array>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>

#include "ball_detector/clustering.hpp"
#include "ball_detector/pointcloud_processor.hpp"

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
    // 初期化関連
    void load_parameters();
    void setup_publishers_and_subscribers();

    // コールバック関数
    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void autonomous_callback(const std_msgs::msg::Bool::SharedPtr msg);

    // ボール検出関連
    Point3D detect_ball(const std::vector<Point3D> &processed_points, const rclcpp::Time &current_time, double dt);
    Point3D calculate_ball_position(const std::vector<VoxelCluster> &clusters);

    // 視覚化関連
    void publish_visualization(Point3D &ball_position, const std::vector<VoxelCluster> &clusters,
                               const sensor_msgs::msg::PointCloud2 &cloud_msg);
    visualization_msgs::msg::Marker create_ball_marker(const Point3D &centroid, const std_msgs::msg::Header &header);
    visualization_msgs::msg::MarkerArray create_voxel_cluster_markers(const std::vector<VoxelCluster> &clusters);
    visualization_msgs::msg::Marker create_trajectory_marker(const std::deque<Point3D> &trajectory, const std_msgs::msg::Header &header);
    visualization_msgs::msg::Marker create_past_points_marker(const std::deque<Point3D> &past_points, const std_msgs::msg::Header &header);
    void update_trajectory(const Point3D &centroid, const sensor_msgs::msg::PointCloud2 &remaining_cloud);

    // サブスクリプション
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr autonomous_subscription_;

    // パブリッシャー
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr clustered_voxel_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr ball_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_cloud_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr trajectory_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr past_points_publisher_;

    // 処理に必要な状態変数
    std::vector<Point3D> previous_centroids_;
    rclcpp::Time previous_time_;
    std::mutex centroid_mutex_;
    std::string frame_id_ = "map";
    Parameters params_;

    // 軌跡関連
    std::vector<Point3D> clustered_points_;
    std::deque<Point3D> ball_trajectory_points_;
    std::deque<Point3D> past_points_;

    // 処理クラス
    std::unique_ptr<VoxelProcessor> voxel_processor_;
    std::unique_ptr<Clustering> clustering_;

    // 自己位置
    Point3D self_pose_;
    bool is_autonomous = false;
  };
}
#endif // BALL_DETECTOR_HPP
