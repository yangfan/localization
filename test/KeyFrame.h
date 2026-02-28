#pragma once

#include <Eigen/Core>
#include <sophus/se3.hpp>

#include <iostream>

#include "sensors/LidarPointType.h"

struct KeyFrame {

  KeyFrame() = default;
  KeyFrame(const double time, const unsigned long int i,
           const Sophus::SE3d pose, PointCloudPtr cloud)
      : timestamp(time), id(i), pose_lio(pose), scan(cloud) {}

  void save(std::ostream &os) const;
  void load(std::istream &is);

  double timestamp;
  unsigned long int id = 0;

  Sophus::SE3d pose_lio;
  Sophus::SE3d pose_opt1;
  Sophus::SE3d pose_opt2;

  bool valid_gnss = false;
  Eigen::Vector3d pos_gnss;

  PointCloudPtr scan = nullptr;
};