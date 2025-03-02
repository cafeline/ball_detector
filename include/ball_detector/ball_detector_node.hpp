#ifndef BALL_DETECTOR_NODE_HPP_
#define BALL_DETECTOR_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "ball_detector/core/ball_detector_core.hpp"

namespace ball_detector
{
  class BallDetectorNode : public rclcpp::Node
  {
  public:
    explicit BallDetectorNode(const rclcpp::NodeOptions &options);

  private:
    void load_parameters();
    void setup_publishers_and_subscribers();
    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void autonomous_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void publish_visualization(const DetectionResult &result, const std::vector<Point3D> processed_points, const std_msgs::msg::Header &header);

    std::unique_ptr<BallDetectorCore> ball_detector_core_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr autonomous_subscription_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr ball_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_cloud_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr trajectory_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr past_points_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr clustered_voxel_publisher_;

    rclcpp::Time previous_time_;
    Point3D self_pose_;
    bool is_autonomous = false;
    Parameters params_;
  };
} // namespace ball_detector

#endif // BALL_DETECTOR_NODE_HPP_