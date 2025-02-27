#ifndef BALL_DETECTOR_VOXEL_CLUSTERER_HPP_
#define BALL_DETECTOR_VOXEL_CLUSTERER_HPP_

#include "ball_detector/types.hpp"
#include <vector>
#include <string>
#include <unordered_map>
#include <queue>

namespace ball_detector
{

  class VoxelClusterer
  {
  public:
    explicit VoxelClusterer(const Parameters &params) : params_(params) {}
    std::vector<VoxelCluster> create_clusters(const std::vector<Point3D> &points, const std::vector<Voxel> &voxels);

  private:
    Parameters params_;
    std::vector<std::string> get_adjacent_keys(const std::string &key) const;
  };

} // namespace ball_detector

#endif // BALL_DETECTOR_VOXEL_CLUSTERER_HPP_
