#pragma once

#include <Eigen/Core>
#include <sophus/se3.hpp>

#include <memory>

struct IMU {
  IMU() = default;
  IMU(const double time, const Eigen::Vector3d &omega, const Eigen::Vector3d &a)
      : timestamp(time), gyr(omega), acc(a) {}
  double timestamp{0.0};
  Eigen::Vector3d gyr{};
  Eigen::Vector3d acc{};
};
using IMUPtr = std::shared_ptr<IMU>;

struct IMUState {
  IMUState() = default;
  IMUState(const Eigen::Matrix<double, 18, 1> &vec)
      : pos(vec.head<3>()), vel(vec.segment<3>(3)),
        rot(Sophus::SO3d::exp(vec.segment<3>(6))), bias_g(vec.segment<3>(9)),
        bias_a(vec.segment<3>(12)), gravity(vec.tail<3>()) {}

  Eigen::Matrix<double, 18, 1> vec() const {
    Eigen::Matrix<double, 18, 1> res;
    res << pos, vel, rot.log(), bias_g, bias_a, gravity;
    return res;
  }
  Sophus::SE3d SE3() const { return Sophus::SE3d(rot, pos); }

  double timestamp{0.0};
  Eigen::Vector3d pos = Eigen::Vector3d::Zero();
  Eigen::Vector3d vel = Eigen::Vector3d::Zero();
  Sophus::SO3d rot{};
  Eigen::Vector3d bias_g = Eigen::Vector3d::Zero();
  Eigen::Vector3d bias_a = Eigen::Vector3d::Zero();
  Eigen::Vector3d gravity{0, 0, -9.8};
};
