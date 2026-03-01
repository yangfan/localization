#pragma once

#include <map>

#include "common/eigen_types.h"
#include "common/nav_state.h"
#include "common/point_types.h"

#include "localizer/tools.h"
#include "sensors/LidarPointType.h"
#include "sensors/imu.h"

namespace sad {

std::map<Vec2i, sad::CloudPtr, sad::less_vec<2>>
convert_map(const std::unordered_map<Eigen::Vector2i, PointCloudPtr,
                                     localization::hash_pt2> &map_data);
sad::NavStated convert_state(const IMUState &state);

} // namespace sad