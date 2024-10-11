sensor_msgs::msg::PointCloud2 MsgConverter::vector_to_PC2(vector<LaserPoint>& points) {
  vector<vector<float>> points_;
  vector<float> point;
  for (size_t i = 0; i < points.size(); i++) {
    point = {static_cast<float>(points[i].x), static_cast<float>(points[i].y), 0};
    points_.push_back(point);
  }

  sensor_msgs::msg::PointCloud2 cloud_msg;
  cloud_msg.header.frame_id = "map";
  cloud_msg.height = 1;
  cloud_msg.width = points_.size();
  cloud_msg.fields.resize(3);
  cloud_msg.fields[0].name = "x";
  cloud_msg.fields[1].name = "y";
  cloud_msg.fields[2].name = "z";

  for (int i = 0; i < 3; ++i) {
    cloud_msg.fields[i].offset = i * 4;
    cloud_msg.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud_msg.fields[i].count = 1;
  }

  cloud_msg.point_step = 4 * 3;
  cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width;
  cloud_msg.data.resize(cloud_msg.row_step * cloud_msg.height);
  cloud_msg.is_bigendian = false;
  cloud_msg.is_dense = true;

  // ポイントデータをPointCloud2メッセージに追加
  int j = 0;
  for (auto& point : points_) {
    memcpy(&cloud_msg.data[j], &point[0], sizeof(float) * 3);
    j += cloud_msg.point_step;
  }

  return cloud_msg;
}