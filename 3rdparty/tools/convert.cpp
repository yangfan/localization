#include "convert.h"

namespace sad {

std::map<Vec2i, sad::CloudPtr, sad::less_vec<2>>
convert_map(const std::unordered_map<Eigen::Vector2i, PointCloudPtr,
                                     localization::hash_pt2> &map_data) {

  std::map<Vec2i, sad::CloudPtr, sad::less_vec<2>> global_cloud;
  for (const auto &[id, cloud] : map_data) {
    global_cloud.insert({id, cloud});
  }
  return global_cloud;
}

sad::NavStated convert_state(const IMUState &state) {
  return NavStated(state.timestamp, state.rot, state.pos, state.vel,
                   state.bias_g, state.bias_a);
}

} // namespace sad