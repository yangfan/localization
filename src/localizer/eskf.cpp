#include "localizer/eskf.h"

#include <glog/logging.h>

void ESKF::initialize_noise(const ImuInitializer &initializer) {
  // noise_.dia_var_g_ = initializer.var_g();
  // noise_.dia_var_a_ = initializer.var_a();
  // ref
  noise_.dia_var_g_ =
      Eigen::Vector3d::Ones() * std::sqrt(initializer.var_g()[0]);
  noise_.dia_var_a_ =
      Eigen::Vector3d::Ones() * std::sqrt(initializer.var_a()[0]);
  nominal_state_.gravity = initializer.gravity();
  nominal_state_.vel = Eigen::Vector3d::Zero(); // static status
  nominal_state_.bias_g = initializer.bias_g();
  nominal_state_.bias_a = initializer.bias_a();
  cov_ = Eigen::Matrix<double, 18, 18>::Identity() * 1e-4;
  set_init_noise_ = true;
  update_time(initializer.timestamp());
  LOG(INFO) << "Noise initialized.";
}
void ESKF::initialize_pose(const Sophus::SE3d &Tob, const double timestamp) {
  nominal_state_.pos = Tob.translation();
  nominal_state_.rot = Tob.so3();
  set_init_pose_ = true;
  update_time(timestamp);
  LOG(INFO) << "Pose initialized.";
}

bool ESKF::update_time(const double time) {
  if (timestamp_ > time) {
    LOG(INFO) << "time " << time << " is older than current: " << timestamp_;
    return false;
  }
  timestamp_ = time;
  nominal_state_.timestamp = time;
  error_state_.timestamp = time;
  return true;
}

bool ESKF::predict_imu(const IMU &imu_data) {
  const double dt = imu_data.timestamp - nominal_state_.timestamp;
  if (timestamp_ == 0 || dt <= 0) {
    LOG(INFO) << "Invalid time interval. Skip current data.";
    update_time(imu_data.timestamp);
    return false;
  }
  IMUState predicted_state;
  predicted_state.pos =
      nominal_state_.pos + nominal_state_.vel * dt +
      0.5 * (nominal_state_.rot * (imu_data.acc - nominal_state_.bias_a)) * dt *
          dt +
      0.5 * nominal_state_.gravity * dt * dt;
  predicted_state.vel =
      nominal_state_.vel +
      nominal_state_.rot * (imu_data.acc - nominal_state_.bias_a) * dt +
      nominal_state_.gravity * dt;
  predicted_state.rot =
      nominal_state_.rot *
      Sophus::SO3d::exp((imu_data.gyr - nominal_state_.bias_g) * dt);
  // ba, bg, g unchanged

  Eigen::Matrix<double, 18, 18> F =
      Eigen::Matrix<double, 18, 18>::Identity(); // jacobian of motion model
  F.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;
  F.block<3, 3>(3, 6) =
      -(nominal_state_.rot.matrix() *
        Sophus::SO3d::hat(imu_data.acc - nominal_state_.bias_a) * dt);
  F.block<3, 3>(3, 12) = -nominal_state_.rot.matrix() * dt;
  F.block<3, 3>(3, 15) = Eigen::Matrix3d::Identity() * dt;
  F.block<3, 3>(6, 6) =
      Sophus::SO3d::exp(-(imu_data.gyr - nominal_state_.bias_g) * dt).matrix();
  F.block<3, 3>(6, 9) = -Eigen::Matrix3d::Identity() * dt;

  // error state unchanged
  // IMUState predict_err_state(F * error_state_.vec());

  // Eigen::Matrix<double, 18, 18> Q =
  //     Eigen::Matrix<double, 18, 18>::Zero(); // covariance of motion noise
  // Q.diagonal() << 0, 0, 0, noise_.dia_var_a_ * dt * dt,
  //     noise_.dia_var_g_ * dt * dt, noise_.var_bg_ * dt, noise_.var_bg_ * dt,
  //     noise_.var_bg_ * dt, noise_.var_ba_ * dt, noise_.var_ba_ * dt,
  //     noise_.var_ba_ * dt, 0, 0, 0;
  // ref
  Eigen::Matrix<double, 18, 18> Q =
      Eigen::Matrix<double, 18, 18>::Zero(); // covariance of motion noise
  Q.diagonal() << 0, 0, 0, noise_.dia_var_a_, noise_.dia_var_g_, noise_.var_bg_,
      noise_.var_bg_, noise_.var_bg_, noise_.var_ba_, noise_.var_ba_,
      noise_.var_ba_, 0, 0, 0;

  cov_ = F * cov_.eval() * F.transpose() + Q;
  nominal_state_.rot = predicted_state.rot;
  nominal_state_.pos = predicted_state.pos;
  nominal_state_.vel = predicted_state.vel;
  update_time(imu_data.timestamp);

  return true;
}

bool ESKF::correct_pose(const Sophus::SE3d &Tob, const double timestamp) {
  if (timestamp < timestamp_) {
    LOG(WARNING) << "The correction timestamp is older than the system current "
                    "timestamp. Skip.";
    return false;
  }
  // gnss_pose = Tob;
  Eigen::Matrix<double, 6, 1> obs_err;
  obs_err.head<3>() = Tob.translation() - nominal_state_.pos;
  obs_err.tail<3>() = (nominal_state_.rot.inverse() * Tob.so3()).log();

  Eigen::Matrix<double, 6, 18> H =
      Eigen::Matrix<double, 6, 18>::Zero(); // jacobian of observation model
  H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
  H.block<3, 3>(3, 6) = Eigen::Matrix3d::Identity();

  Eigen::Matrix<double, 6, 6> V =
      Eigen::Matrix<double, 6, 6>::Zero(); // covariance of observation noise
  V.diagonal() << noise_.var_gnss_pos, noise_.var_gnss_pos,
      noise_.var_gnss_height, noise_.var_gnss_ang, noise_.var_gnss_ang,
      noise_.var_gnss_ang;
  Eigen::Matrix<double, 18, 6> K =
      cov_ * H.transpose() *
      ((H * cov_ * H.transpose() + V).inverse()); // Kalman gain

  Eigen::Matrix<double, 18, 1> err_state = K * obs_err;
  error_state_ = IMUState(err_state);
  cov_ = (Eigen::Matrix<double, 18, 18>::Identity() - K * H) * cov_.eval();

  correct_state();

  reset_error();
  update_time(timestamp);

  return true;
}

bool ESKF::correct_state() {
  nominal_state_.pos += error_state_.pos;
  nominal_state_.vel += error_state_.vel;
  nominal_state_.rot *= error_state_.rot;
  nominal_state_.bias_g += error_state_.bias_g;
  nominal_state_.bias_a += error_state_.bias_a;
  nominal_state_.gravity += error_state_.gravity;

  return true;
}

bool ESKF::reset_error() {
  Eigen::Matrix<double, 18, 18> J =
      Eigen::Matrix<double, 18, 18>::Identity(); // jacobian of reset function
  J.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() -
                        0.5 * Sophus::SO3d::hat(error_state_.rot.log());
  cov_ = J * cov_ * J.transpose();
  error_state_ = IMUState(Eigen::Matrix<double, 18, 1>::Zero());
  error_state_.rot = Sophus::SO3d();
  return true;
}

bool ESKF::correct_pose(const Sophus::SE3d &Tob, const double timestamp,
                        const double var_pos, const double var_ang) {
  if (timestamp < timestamp_) {
    LOG(WARNING) << "The correction timestamp is older than the system current "
                    "timestamp. Skip.";
    return false;
  }
  // gnss_pose = Tob;
  Eigen::Matrix<double, 6, 1> obs_err;
  obs_err.head<3>() = Tob.translation() - nominal_state_.pos;
  obs_err.tail<3>() = (nominal_state_.rot.inverse() * Tob.so3()).log();

  Eigen::Matrix<double, 6, 18> H =
      Eigen::Matrix<double, 6, 18>::Zero(); // jacobian of observation model
  H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
  H.block<3, 3>(3, 6) = Eigen::Matrix3d::Identity();

  Eigen::Matrix<double, 6, 6> V =
      Eigen::Matrix<double, 6, 6>::Zero(); // covariance of observation noise
  V.diagonal() << var_pos, var_pos, var_pos, var_ang, var_ang, var_ang;
  Eigen::Matrix<double, 18, 6> K =
      cov_ * H.transpose() *
      ((H * cov_ * H.transpose() + V).inverse()); // Kalman gain

  Eigen::Matrix<double, 18, 1> err_state = K * obs_err;
  error_state_ = IMUState(err_state);
  cov_ = (Eigen::Matrix<double, 18, 18>::Identity() - K * H) * cov_.eval();

  correct_state();

  reset_error();
  update_time(timestamp);

  return true;
}
