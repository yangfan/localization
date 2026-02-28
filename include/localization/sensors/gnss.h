#pragma once

#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sophus/se3.hpp>

class GNSS {
public:
  struct UTM {
    long zone = 0;
    bool north_hemi = true;
    Eigen::Vector2d xy = Eigen::Vector2d::Zero();
  };
  enum class Status { FIX, NO_FIX };
  GNSS() = default;
  GNSS(const double time, const double lat, const double lon, const double alt)
      : timestamp_(time), geodetic_(lat, lon, alt) {}

  explicit GNSS(const sensor_msgs::msg::NavSatFix &msg)
      : geodetic_(msg.latitude, msg.longitude, msg.altitude) {

    timestamp_ = rclcpp::Time(msg.header.stamp).seconds();
    status_ = int(msg.status.status) >=
                      int(sensor_msgs::msg::NavSatStatus::STATUS_FIX)
                  ? Status::FIX
                  : Status::NO_FIX;
    convert_utm();
  }

  bool convert_utm();

  Eigen::Vector3d utm() const {
    return Eigen::Vector3d(utm_.xy.x(), utm_.xy.y(), geodetic_.z());
  }

  Status status() const { return status_; }

  double timestamp() const { return timestamp_; }

  Eigen::Vector3d pos() const { return position_; }
  void set_pos(const Eigen::Vector3d &pos) { position_ = pos; }

private:
  double timestamp_ = 0;
  Eigen::Vector3d geodetic_; // lat, lon (deg), alt
  UTM utm_;

  Eigen::Vector3d position_;

  Status status_ = Status::NO_FIX;
};