#pragma once
#include <Eigen/Core>
#include <sophus/se3.hpp>

#include "localizer/tools.h"
#include "sensors/ImuInitializer.h"
#include "sensors/imu.h"

class ESKF {
public:
  struct Noise {
    double var_bg_ = 1e-6; // contineous: rad / (s * sqrt(s))
    double var_ba_ = 1e-4; // continuous: m / (s^2 * sqr(s))
    double var_odom = 0.25;
    // double var_gnss_pos = 0.01;
    // double var_gnss_height = 0.01;
    double var_gnss_pos = 0.1;
    double var_gnss_height = 0.1;
    // double var_gnss_ang = 1.0 * kdeg2rad_ * kdeg2rad_;
    double var_gnss_ang = 1e-2;
    Eigen::Vector3d dia_var_g_ = Eigen::Vector3d::Zero(); // discrete rad / s
    Eigen::Vector3d dia_var_a_ = Eigen::Vector3d::Zero(); // discrete m / (s*s)
  };

  bool noise_initialized() const { return set_init_noise_; }
  bool pose_initialized() const { return set_init_pose_; }
  IMUState state() const { return nominal_state_; }
  Sophus::SE3d state_SE3() const {
    return Sophus::SE3d(nominal_state_.rot, nominal_state_.pos);
  }

  void initialize_noise(const ImuInitializer &initializer);
  void initialize_pose(const Sophus::SE3d &Tob, const double timestamp);

  bool predict_imu(const IMU &imu_data);
  bool correct_pose(const Sophus::SE3d &Tob, const double timestamp);
  bool correct_pose(const Sophus::SE3d &Tob, const double timestamp,
                    const double var_pos, const double var_ang);

  bool correct_state();
  bool reset_error();

  bool update_time(const double time);

  void set_state(const IMUState state) { nominal_state_ = state; }
  void set_cov(const Eigen::Matrix<double, 18, 18> cov) { cov_ = cov; }

private:
  IMUState nominal_state_;
  IMUState error_state_;
  Eigen::Matrix<double, 18, 18> cov_ = Eigen::Matrix<double, 18, 18>::Zero();

  Noise noise_;
  double timestamp_ = 0.0;

  bool set_init_pose_ = false;
  bool set_init_noise_ = false;
};