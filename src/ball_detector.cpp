#include "ball_detector/ball_detector.hpp"
#include "pointcloud_processor/pointcloud_processor.hpp"
#include <chrono>
#include <unordered_set>
#include <random>
#include <cmath>

namespace ball_detector
{
  BallDetector::BallDetector(const rclcpp::NodeOptions &options) : BallDetector("", options) {}

  BallDetector::BallDetector(const std::string &name_space, const rclcpp::NodeOptions &options)
      : rclcpp::Node("ball_detector", name_space, options)
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/livox/lidar", 10, std::bind(&BallDetector::pointcloud_callback, this, std::placeholders::_1));
    pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/laser_pose", 10, std::bind(&BallDetector::pose_callback, this, std::placeholders::_1));
    autonomous_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
        "autonomous", 10, std::bind(&BallDetector::autonomous_callback, this, std::placeholders::_1));

    ball_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("tennis_ball", 10);
    filtered_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("ball_detector_pointcloud", 10);
    trajectory_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("ball_trajectory", 10);
    past_points_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("past_ball_points", 10);
    clustered_voxel_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("clustered_voxel", 10);

    load_parameters();

    voxel_processor_ = std::make_unique<VoxelProcessor>(params_);
    clustering_ = std::make_unique<Clustering>(params_);
    // RCLCPP_INFO(this->get_logger(), "ball_detector initialized");
  }

  void BallDetector::load_parameters()
  {
    params_.min_x = this->get_parameter("min_x").as_double();
    params_.max_x = this->get_parameter("max_x").as_double();
    params_.min_y = this->get_parameter("min_y").as_double();
    params_.max_y = this->get_parameter("max_y").as_double();
    params_.min_z = this->get_parameter("min_z").as_double();
    params_.max_z = this->get_parameter("max_z").as_double();
    livox_pitch_ = this->get_parameter("livox_pitch").as_double();
    params_.voxel_size_x = this->get_parameter("voxel_size_x").as_double();
    params_.voxel_size_y = this->get_parameter("voxel_size_y").as_double();
    params_.voxel_size_z = this->get_parameter("voxel_size_z").as_double();
    params_.voxel_search_range = this->get_parameter("voxel_search_range").as_int();
    params_.ball_radius = this->get_parameter("ball_radius").as_double();
    ball_vel_min_ = this->get_parameter("ball_vel_min").as_double();
    max_distance_for_association_ = this->get_parameter("max_distance_for_association").as_double();
    missing_count_threshold_ = this->get_parameter("missing_count_threshold").as_int();
  }

  void BallDetector::pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // if (!is_autonomous)
    //   return;
    ball_cluster_indices_.clear();
    dynamic_cluster_indices_.clear();

    auto start_time = std::chrono::high_resolution_clock::now();

    std::vector<Point3D> processed_points = preprocess_pointcloud(*msg);

    // // RCLCPP_INFO(this->get_logger(), "Time taken for voxelization and clustering: %ld ms", std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start_time).count());

    // ボクセル化の実行
    std::vector<Voxel> voxels = voxel_processor_->create_voxel(processed_points);
    // クラスタリングの実行
    std::vector<VoxelCluster> clusters = clustering_->create_voxel_clustering(processed_points, voxels);
    std::vector<VoxelCluster> ball_clusters = extract_ball_clusters(clusters, processed_points);
    std::vector<VoxelCluster> dynamic_clusters;
    process_clusters(clusters, dynamic_clusters, processed_points, msg->header);
    
    // 残りの点群をPointCloud2形式に変換してパブリッシュ
    PointCloudProcessor processor(params_);
    sensor_msgs::msg::PointCloud2 remaining_cloud = processor.vector_to_PC2(processed_points);
    filtered_cloud_publisher_->publish(remaining_cloud);

    // マーカーのパブリッシュ
    publish_markers(clusters, ball_clusters, remaining_cloud);

    // 軌道と過去の検出点の更新
    update_trajectory(dynamic_clusters, remaining_cloud);

    // トラックの状態をログ出力
    for (const auto &track : tracks_)
    {
      // RCLCPP_INFO(this->get_logger(), "Track ID: %d, Last Centroid: x=%f, y=%f, z=%f, Missing Count: %d", track.first, track.second.last_centroid.x, track.second.last_centroid.y, track.second.last_centroid.z, track.second.missing_count);
    }
  }

  std::vector<Point3D> BallDetector::preprocess_pointcloud(const sensor_msgs::msg::PointCloud2 &msg)
  {
    PointCloudProcessor processor(params_);
    auto tmp_points = processor.PC2_to_vector(msg);
    auto filtered_points = processor.filter_points_base_origin(self_pose_.x, self_pose_.y, self_pose_.z, tmp_points);
    auto transformed_points = processor.transform_pointcloud(self_pose_.x, self_pose_.y, self_pose_.z, filtered_points);
    return processor.rotate_pitch(transformed_points, livox_pitch_);
  }

  std::vector<VoxelCluster> BallDetector::extract_ball_clusters(const std::vector<VoxelCluster> &clusters, const std::vector<Point3D> &processed_points)
  {
    std::vector<VoxelCluster> ball_clusters;
    ball_dynamic_cluster_indices_.clear();
    for (size_t i = 0; i < clusters.size(); ++i)
    {
      double size_x, size_y, size_z;
      clustering_->calculate_cluster_size(clusters[i], processed_points, size_x, size_y, size_z);
      if (is_ball_size(size_x, size_y, size_z))
      {
        ball_clusters.emplace_back(clusters[i]);
        ball_cluster_indices_.insert(i);
        // ボールクラスタが元々clustersの何番目だったかを記録
        ball_dynamic_cluster_indices_.push_back(i);
      }
    }
    return ball_clusters;
  }

  bool BallDetector::is_ball_size(double size_x, double size_y, double size_z) const
  {
    return (size_x <= 2 * params_.ball_radius &&
            size_y <= 2 * params_.ball_radius &&
            size_z <= 2 * params_.ball_radius);
  }

  void BallDetector::process_clusters(const std::vector<VoxelCluster> &clusters,
                                      std::vector<VoxelCluster> &dynamic_clusters,
                                      const std::vector<Point3D> &processed_points,
                                      const std_msgs::msg::Header &header)
  {
    rclcpp::Time current_time = this->now();
    double dt = (previous_time_.nanoseconds() == 0) ? 0.0 : (current_time - previous_time_).seconds();
    previous_time_ = current_time;

    auto assignments = associate_clusters(clusters, tracks_, max_distance_for_association_, current_time, dt);
    remove_missing_tracks();

    dynamic_clusters = filter_by_speed(clusters, assignments, tracks_, dt, ball_vel_min_);
    identify_dynamic_clusters(clusters, dynamic_clusters);

    // その他の処理やロギング
    RCLCPP_INFO(this->get_logger(), "Clusters size: %zu, Dynamic clusters size: %zu, Tracks size: %zu",
                clusters.size(), dynamic_clusters.size(), tracks_.size());
  }

  void BallDetector::remove_missing_tracks()
  {
    for (auto it = tracks_.begin(); it != tracks_.end();)
    {
      if (it->second.missing_count > missing_count_threshold_)
        it = tracks_.erase(it);
      else
        ++it;
    }
  }

  void BallDetector::identify_dynamic_clusters(const std::vector<VoxelCluster> &clusters,
                                               const std::vector<VoxelCluster> &dynamic_clusters)
  {
    for (const auto &dyn_cluster : dynamic_clusters)
    {
      Point3D dyn_center = compute_cluster_centroid(dyn_cluster);
      for (size_t i = 0; i < clusters.size(); ++i)
      {
        Point3D orig_center = compute_cluster_centroid(clusters[i]);
        if (are_centroids_close(orig_center, dyn_center))
        {
          dynamic_cluster_indices_.insert(i);
          break;
        }
      }
    }
  }

  bool BallDetector::are_centroids_close(const Point3D &a, const Point3D &b) const
  {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    double dz = a.z - b.z;
    double tol = 1e-6;
    return (dx * dx + dy * dy + dz * dz) < (tol * tol);
  }

  Point3D BallDetector::compute_cluster_centroid(const VoxelCluster &cluster)
  {
    Point3D centroid{0.0f, 0.0f, 0.0f};
    if (cluster.points.empty())
    {
      return centroid;
    }

    double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
    for (const auto &p : cluster.points)
    {
      sum_x += p.x;
      sum_y += p.y;
      sum_z += p.z;
    }
    double n = static_cast<double>(cluster.points.size());
    centroid.x = static_cast<float>(sum_x / n);
    centroid.y = static_cast<float>(sum_y / n);
    centroid.z = static_cast<float>(sum_z / n);
    return centroid;
  }

  std::vector<int> BallDetector::associate_clusters(const std::vector<VoxelCluster> &current_clusters,
                                                    std::map<int, ClusterTrack> &tracks,
                                                    double max_distance_for_association,
                                                    rclcpp::Time current_time,
                                                    double dt)
  {
    std::vector<int> assignments(current_clusters.size(), -1);
    // 使用済みトラックを記録
    std::vector<bool> track_used(tracks.size(), false);

    // ベクトル化
    std::vector<std::map<int, ClusterTrack>::iterator> track_list;
    track_list.reserve(tracks.size());
    for (auto it = tracks.begin(); it != tracks.end(); ++it)
    {
      track_list.push_back(it);
    }

    // クラスタごとに最近傍トラックを探す
    for (size_t i = 0; i < current_clusters.size(); i++)
    {
      Point3D c = compute_cluster_centroid(current_clusters[i]);
      double best_dist = std::numeric_limits<double>::max();
      int best_idx = -1;
      for (size_t j = 0; j < track_list.size(); j++)
      {
        if (track_used[j])
          continue;
        Point3D t = track_list[j]->second.last_centroid;
        double dx = c.x - t.x;
        double dy = c.y - t.y;
        double dz = c.z - t.z;
        double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
        if (dist < best_dist && dist < max_distance_for_association)
        {
          best_dist = dist;
          best_idx = (int)j;
        }
      }
      if (best_idx >= 0)
      {
        // 対応付け成功
        // // RCLCPP_INFO(this->get_logger(), "対応付け成功");
        assignments[i] = track_list[best_idx]->first;
        track_used[best_idx] = true;
        track_list[best_idx]->second.last_update_time = current_time;
        track_list[best_idx]->second.missing_count = 0;
      }
    }

    // 対応なしのクラスタは新規トラックとして登録
    for (size_t i = 0; i < assignments.size(); i++)
    {
      if (assignments[i] < 0)
      {
        // // RCLCPP_INFO(this->get_logger(), "New track");
        int new_id = tracks.empty() ? 0 : tracks.rbegin()->first + 1;
        Point3D c = compute_cluster_centroid(current_clusters[i]);
        ClusterTrack new_track{new_id, c, current_time, 0};
        tracks[new_id] = new_track;
        assignments[i] = new_id;
      }
    }

    // 対応無しになったトラックはmissing_count++
    for (size_t j = 0; j < track_list.size(); j++)
    {
      if (!track_used[j])
      {
        // // RCLCPP_INFO(this->get_logger(), "Missing track");
        track_list[j]->second.missing_count += 1;
      }
    }

    return assignments;
  }

  std::vector<VoxelCluster> BallDetector::filter_by_speed(const std::vector<VoxelCluster> &current_clusters,
                                                          const std::vector<int> &assignments,
                                                          std::map<int, ClusterTrack> &tracks,
                                                          double dt,
                                                          double speed_threshold)
  {
    std::vector<VoxelCluster> result;
    for (size_t i = 0; i < current_clusters.size(); i++)
    {
      int id = assignments[i];
      if (id < 0)
      {
        // 新トラックは速度判定できないのでそのまま通す
        result.push_back(current_clusters[i]);
        continue;
      }
      auto it = tracks.find(id);
      if (it == tracks.end())
      {
        // トラック見つからないならそのまま通す
        result.push_back(current_clusters[i]);
        continue;
      }

      // 前回更新時と今回で速度を計算
      Point3D last_c = it->second.last_centroid;
      Point3D cur_c = compute_cluster_centroid(current_clusters[i]);
      double dx = cur_c.x - last_c.x;
      double dy = cur_c.y - last_c.y;
      double dz = cur_c.z - last_c.z;
      double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
      double speed = dist / dt;

      if (speed >= speed_threshold)
      {
        result.push_back(current_clusters[i]);
      }
      tracks[id].last_centroid = cur_c; // 重心更新
    }

    return result;
  }

  void BallDetector::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    double w = msg->pose.orientation.w;
    double x = msg->pose.orientation.x;
    double y = msg->pose.orientation.y;
    double z = msg->pose.orientation.z;
    // ヨー角の算出
    double siny_cosp = 2.0 * (w * z + x * y);
    double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    double yaw = std::atan2(siny_cosp, cosy_cosp);
    self_pose_.x = msg->pose.position.x;
    self_pose_.y = msg->pose.position.y;
    self_pose_.z = yaw;
  }

  void BallDetector::autonomous_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    is_autonomous = msg->data;
    // // RCLCPP_INFO(this->get_logger(), "自動：%d", is_autonomous);
  }

  void BallDetector::publish_markers(const std::vector<VoxelCluster> &all_clusters, const std::vector<VoxelCluster> &ball_clusters, const sensor_msgs::msg::PointCloud2 &remaining_cloud)
  {
    // すべてのクラスタを可視化
    visualization_msgs::msg::MarkerArray voxel_marker_array = create_voxel_cluster_markers(all_clusters, ball_clusters);
    clustered_voxel_publisher_->publish(voxel_marker_array);

    // ボールクラスタのみをマークする場合の処理
    for (size_t j = 0; j < ball_clusters.size(); ++j)
    {
      const auto &cluster = ball_clusters[j];
      if (cluster.points.empty())
      {
        continue;
      }

      size_t orig_idx = ball_dynamic_cluster_indices_[j];
      bool is_dynamic = (dynamic_cluster_indices_.find(orig_idx) != dynamic_cluster_indices_.end());
      // 動的クラスタでない場合はtennis_ball発行しない
      if (!is_dynamic)
      {
        continue;
      }

      Point3D centroid = compute_cluster_centroid(cluster);
      visualization_msgs::msg::Marker marker = create_ball_marker(centroid, remaining_cloud.header);
      ball_publisher_->publish(marker);
    }
  }

  visualization_msgs::msg::MarkerArray BallDetector::create_voxel_cluster_markers(const std::vector<VoxelCluster> &all_clusters, const std::vector<VoxelCluster> &ball_clusters)
  {
    visualization_msgs::msg::MarkerArray marker_array;

    for (size_t i = 0; i < all_clusters.size(); ++i)
    {
      const auto &cluster = all_clusters[i];

      bool is_ball_cluster = (ball_cluster_indices_.find(i) != ball_cluster_indices_.end());
      bool is_dynamic = (dynamic_cluster_indices_.find(i) != dynamic_cluster_indices_.end());

      float r, g, b;
      if (is_ball_cluster && is_dynamic)
      {
        r = 0.0f;
        g = 1.0f;
        b = 0.0f; // 緑
      }
      else if (is_ball_cluster)
      {
        r = 1.0f;
        g = 0.0f;
        b = 0.0f; // 赤
      }
      else
      {
        r = 0.0f;
        g = 0.0f;
        b = 1.0f; // 青
      }

      for (size_t v = 0; v < cluster.voxels.size(); ++v)
      {
        const auto &voxel = cluster.voxels[v];
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = frame_id_;
        marker.ns = "voxel_cluster_markers";
        marker.id = static_cast<int>(i * 10000 + v); // ユニークなIDを生成
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;

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

        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = 0.3;

        marker.lifetime = rclcpp::Duration(0, 0); // 約0.1秒

        marker_array.markers.push_back(marker);
      }
    }

    return marker_array;
  }

  visualization_msgs::msg::Marker BallDetector::create_custom_marker(const Point3D &point, const std_msgs::msg::Header &header)
  {
    visualization_msgs::msg::Marker marker;
    marker.header = header;
    marker.ns = "min_x_points";
    marker.id = static_cast<int>(point.x * 1000); // 一意のIDを設定（例: x座標を基に）
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // 点の位置を設定
    marker.pose.position.x = point.x;
    marker.pose.position.y = point.y;
    marker.pose.position.z = point.z;

    // マーカーのスケールを設定
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;

    // マーカーの色を設定 (例: 青色)
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;

    // ライフタイムを0に設定して永続的に表示
    marker.lifetime = rclcpp::Duration(0, 0);

    return marker;
  }
  void BallDetector::update_trajectory(const std::vector<VoxelCluster> &clusters, const sensor_msgs::msg::PointCloud2 &remaining_cloud)
  {
    VoxelCluster ball_cluster;
    for (size_t i = 0; i < clusters.size(); ++i)
    {
      bool is_ball = (ball_cluster_indices_.find(i) != ball_cluster_indices_.end());
      bool is_dynamic = (dynamic_cluster_indices_.find(i) != dynamic_cluster_indices_.end());
      // ボールクラスタかつ動的クラスタのみ合成
      if (is_ball && is_dynamic)
      {
        for (const auto &point : clusters[i].points)
        {
          ball_cluster.points.push_back(point);
        }
      }
    }

    if (!ball_cluster.points.empty())
    {
      Point3D centroid = calculate_cluster_centroid(ball_cluster);

      // 軌道コンテナに追加
      const size_t MAX_TRAJECTORY_POINTS = 200; // 最大保持ポイント数
      if (ball_trajectory_points_.size() >= MAX_TRAJECTORY_POINTS)
      {
        ball_trajectory_points_.pop_front(); // 古いポイントを削除
      }
      ball_trajectory_points_.push_back(centroid);

      // 過去の検出点コンテナに追加
      const size_t MAX_PAST_POINTS = 20; // 最大保持過去点数を10に変更
      if (past_points_.size() >= MAX_PAST_POINTS)
      {
        past_points_.pop_front(); // 古いポイントを削除
      }
      past_points_.push_back(centroid);

      // それぞれPublish
      visualization_msgs::msg::Marker trajectory_marker = create_trajectory_marker(ball_trajectory_points_, remaining_cloud.header);
      trajectory_publisher_->publish(trajectory_marker);
      visualization_msgs::msg::Marker past_points_marker = create_past_points_marker(past_points_, remaining_cloud.header);
      past_points_publisher_->publish(past_points_marker);
    }
  }

  std::vector<Point3D> BallDetector::voxel_downsample(const std::vector<Point3D> &input)
  {
    std::unordered_map<std::string, std::vector<Point3D>> voxel_map;
    std::vector<Point3D> downsampled_points;

    for (const auto &point : input)
    {
      int voxel_x = static_cast<int>(std::floor(point.x / params_.D_voxel_size_x));
      int voxel_y = static_cast<int>(std::floor(point.y / params_.D_voxel_size_y));
      int voxel_z = static_cast<int>(std::floor(point.z / params_.D_voxel_size_z));
      std::string key = std::to_string(voxel_x) + "_" + std::to_string(voxel_y) + "_" + std::to_string(voxel_z);
      voxel_map[key].push_back(point);
    }

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
      downsampled_points.push_back(centroid);
    }

    return downsampled_points;
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
    double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
    for (const auto &point : points)
    {
      sum_x += point.x;
      sum_y += point.y;
      sum_z += point.z;
    }
    Point3D centroid;
    if (!points.empty())
    {
      centroid.x = static_cast<float>(sum_x / points.size());
      centroid.y = static_cast<float>(sum_y / points.size());
      centroid.z = static_cast<float>(sum_z / points.size());
    }
    else
    {
      centroid = {0.0f, 0.0f, 0.0f};
    }
    return centroid;
  }

  Point3D BallDetector::calculate_cluster_centroid(const VoxelCluster &cluster)
  {
    return calculate_centroid(cluster.points);
  }

  visualization_msgs::msg::Marker BallDetector::create_ball_marker(const Point3D &centroid, const std_msgs::msg::Header &header)
  {
    visualization_msgs::msg::Marker marker;
    marker.header = header;
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
    marker.lifetime = rclcpp::Duration(0, 1e8); // 約0.1秒
    return marker;
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

    marker.lifetime = rclcpp::Duration(0, 1e8); // 約0.1秒

    return marker;
  }

  std::vector<Point3D> BallDetector::remove_clustered_points(const std::vector<Point3D> &original_points, const std::vector<VoxelCluster> &clusters)
  {
    std::vector<Point3D> remaining_points;
    std::unordered_set<std::string> clustered_voxels;

    for (const auto &cluster : clusters)
    {
      for (const auto &voxel : cluster.voxels)
      {
        std::string key = std::to_string(voxel.x) + "," + std::to_string(voxel.y) + "," + std::to_string(voxel.z);
        clustered_voxels.insert(key);
      }
    }

    for (const auto &point : original_points)
    {
      int vx = static_cast<int>((point.x - params_.min_x) / params_.voxel_size_x);
      int vy = static_cast<int>((point.y - params_.min_y) / params_.voxel_size_y);
      int vz = static_cast<int>((point.z - params_.min_z) / params_.voxel_size_z);
      std::string key = std::to_string(vx) + "," + std::to_string(vy) + "," + std::to_string(vz);

      if (clustered_voxels.find(key) == clustered_voxels.end())
      {
        remaining_points.push_back(point);
      }
    }

    return remaining_points;
  }

  void BallDetector::collect_cluster_points(VoxelCluster &cluster, const std::vector<Point3D> &points)
  {
    clustering_->collect_cluster_points(cluster, points);
  }

  visualization_msgs::msg::Marker BallDetector::create_trajectory_marker(const std::deque<Point3D> &trajectory, const std_msgs::msg::Header &header)
  {
    visualization_msgs::msg::Marker marker;
    marker.header = header;
    marker.ns = "ball_trajectory";
    marker.id = 1;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // 線の幅
    marker.scale.x = 0.05; // 線の太さ

    // 線の色
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0; // 青色
    marker.color.a = 1.0; // 完全不透明

    // ライフタイムを0に設定して永続的に表示
    marker.lifetime = rclcpp::Duration(0, 0);

    // ポイントを追加
    for (const auto &point : trajectory)
    {
      geometry_msgs::msg::Point geom_point;
      geom_point.x = point.x;
      geom_point.y = point.y;
      geom_point.z = point.z;
      marker.points.push_back(geom_point);
    }

    return marker;
  }

  visualization_msgs::msg::Marker BallDetector::create_past_points_marker(const std::deque<Point3D> &past_points, const std_msgs::msg::Header &header)
  {
    visualization_msgs::msg::Marker marker;
    marker.header = header;
    marker.ns = "past_ball_points";
    marker.id = 2;
    marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // 各点のスケール
    marker.scale.x = 0.05; // ボールの点の半径
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;

    // ボールの点の色
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0; // 緑色
    marker.color.a = 1.0; // 完全不透明

    // ライフタイムを0に設定して永続的に表示
    marker.lifetime = rclcpp::Duration(0, 0);

    // 過去の検出点を追加（最大10点）
    for (const auto &point : past_points)
    {
      geometry_msgs::msg::Point geom_point;
      geom_point.x = point.x;
      geom_point.y = point.y;
      geom_point.z = point.z;
      marker.points.push_back(geom_point);
    }

    return marker;
  }
}
// int main(int argc, char **argv)
// {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<BallDetector>());
//   rclcpp::shutdown();
//   return 0;
// }
