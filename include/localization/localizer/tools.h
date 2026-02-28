#pragma once

#include <Eigen/Core>
#include <glog/logging.h>
#include <sophus/se3.hpp>

#include "sensors/LidarPointType.h"

namespace localization {

constexpr double kdeg2rad = M_PI / 180.0;

PointCloudPtr VoxelGridFilter(PointCloudPtr input, const double voxel_size);

struct hash_pt2 {
  size_t operator()(const Eigen::Vector2i &pt) const {
    return size_t((pt[0] * 73856093) ^ (pt[1] * 83492791) % 10000000);
  }
};

template <typename S>
inline Eigen::Matrix<S, 3, 1> VecFromArray(const std::vector<S> &v) {
  return Eigen::Matrix<S, 3, 1>(v[0], v[1], v[2]);
}

template <typename S>
inline Eigen::Matrix<S, 3, 3> MatFromArray(const std::vector<S> &v) {
  Eigen::Matrix<S, 3, 3> m;
  m << v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8];
  return m;
}

template <typename S>
inline Sophus::SE3d Mat4ToSE3(const Eigen::Matrix<S, 4, 4> &m) {
  Eigen::Quaterniond q(m.template block<3, 3>(0, 0).template cast<double>());
  q.normalize();
  return Sophus::SE3d(q, m.template block<3, 1>(0, 3).template cast<double>());
}
template <typename PointT = LidarPointType>
inline PointCloudPtr
ConvertToCloud(typename pcl::PointCloud<PointT>::Ptr input) {
  PointCloudPtr cloud(new PointCloudType);
  for (auto &pt : input->points) {
    PointType p;
    p.x = pt.x;
    p.y = pt.y;
    p.z = pt.z;
    p.intensity = 0;
    cloud->points.emplace_back(p);
  }
  cloud->width = input->width;
  return cloud;
}

template <typename T, typename C, typename FT, typename FP>
inline bool PoseInterp(double query_time, C &&data, FT &&take_time_func,
                       FP &&take_pose_func, Sophus::SE3d &result, T &best_match,
                       float time_th = 0.5) {
  if (data.empty()) {
    LOG(WARNING) << "cannot interp because data is empty. ";
    return false;
  }

  double last_time = take_time_func(*data.rbegin());
  if (query_time > last_time) {
    if (query_time < (last_time + time_th)) {
      result = take_pose_func(*data.rbegin());
      best_match = *data.rbegin();
      return true;
    }
    // LOG(WARNING) << "scan pt time is greater than imu time. diff: "
    //              << query_time - last_time;
    return false;
  }

  auto match_iter = data.begin();
  for (auto iter = data.begin(); iter != data.end(); ++iter) {
    auto next_iter = iter;
    next_iter++;

    if (take_time_func(*iter) < query_time &&
        take_time_func(*next_iter) >= query_time) {
      match_iter = iter;
      break;
    }
  }

  auto match_iter_n = match_iter;
  match_iter_n++;

  double dt = take_time_func(*match_iter_n) - take_time_func(*match_iter);
  double s = (query_time - take_time_func(*match_iter)) / dt; // s=0
  if (fabs(dt) < 1e-6) {
    best_match = *match_iter;
    result = take_pose_func(*match_iter);
    return true;
  }

  Sophus::SE3d pose_first = take_pose_func(*match_iter);
  Sophus::SE3d pose_next = take_pose_func(*match_iter_n);
  result = {pose_first.unit_quaternion().slerp(s, pose_next.unit_quaternion()),
            pose_first.translation() * (1 - s) + pose_next.translation() * s};
  best_match = s < 0.5 ? *match_iter : *match_iter_n;
  return true;
}

} // namespace localization