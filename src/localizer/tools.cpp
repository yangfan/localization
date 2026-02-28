#include "localizer/tools.h"

#include <pcl/filters/voxel_grid.h>

namespace localization {

PointCloudPtr VoxelGridFilter(PointCloudPtr input, const double voxel_size) {
  pcl::VoxelGrid<PointType> voxel;
  voxel.setLeafSize(voxel_size, voxel_size, voxel_size);
  voxel.setInputCloud(input);

  PointCloudPtr output(new PointCloudType);
  voxel.filter(*output);
  return output;
}

} // namespace localization