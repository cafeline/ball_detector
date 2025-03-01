#pragma once

struct Parameters
{
  double min_x, max_x, min_y, max_y, min_z, max_z;
  double voxel_size_x, voxel_size_y, voxel_size_z;

  int voxel_search_range;
  double ball_radius;

  double max_distance_for_association;
  double ball_vel_min;
  double livox_pitch;
};