#pragma once

#include "localizer/eskf.h"
#include "localizer/tools.h"
#include "sensors/ImuInitializer.h"
#include "sensors/LidarPointType.h"
// #include "sensors/MapViewer.h"
#include "sensors/Sync.h"
#include "sensors/gnss.h"
#include "tools/ui/pangolin_window.h"

#include <Eigen/Core>
#include <glog/logging.h>
#include <pcl/registration/ndt.h>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <unordered_set>

class LocalizerEskfMT {
public:
  using MapId = Eigen::Vector2i;
  struct Map {
    std::unordered_map<MapId, PointCloudPtr, localization::hash_pt2> data;
    std::unordered_set<MapId, localization::hash_pt2> idx;
    PointCloudPtr cloud;

    std::string dir;
    double grid_size = 0.0;
    Eigen::Vector2d grid_origin = Eigen::Vector2d::Zero();

    Eigen::Vector2i map_id(const Eigen::Vector2d &pos) {
      return Eigen::Vector2i(
          int(std::floor((pos.x() - grid_origin.x()) / grid_size)),
          int(std::floor((pos.y() - grid_origin.y()) / grid_size)));
    }
  };
  struct Params {
    double min_kf_dist = 1.0;
    double min_kf_deg = 10;
    double min_relocalize_score = 4.5;
    bool viewer_on = true;
  };
  enum class EstimateMethod { GNSS, ESKF };
  bool config(const std::string &yaml);

  void add_scan(std::unique_ptr<sensor_msgs::msg::PointCloud2> scan_msg);
  void add_imu(std::unique_ptr<sensor_msgs::msg::Imu> imu_msg);
  void add_gnss(std::unique_ptr<sensor_msgs::msg::NavSatFix> gnss_msg);

  bool load_map(const Sophus::SE3d &cur_pose);
  PointCloudPtr map_cloud() { return map_.cloud; }

  void quit();

  ~LocalizerEskfMT() { quit(); }

private:
  ESKF eskf_;
  ImuInitializer imu_initializer_;
  std::vector<IMUState> states_;

  Map map_;
  pcl::NormalDistributionsTransform<PointType, PointType> ndt_;
  std::unique_ptr<sad::ui::PangolinWindow> viewer_;

  Sync lidar_imu_sync_;
  Sync::DataGroup lidar_imu_;
  PointCloudPtr source_cloud_;

  Sophus::SE3d T_IL_;
  Params params_;

  Eigen::Vector3d gnss_origin_;
  std::unique_ptr<GNSS> last_gnss_;

  EstimateMethod method_ = EstimateMethod::GNSS;

  bool map_updated_ = false;
  std::condition_variable cv_update_target_;
  std::thread thd_update_target_;
  std::mutex mtx_ndt_;
  bool update_thd_on_ = true;

  void update_target();

  bool load_map_info(const std::string &map_info);

  void process_callback(const Sync::DataGroup &lidar_imu);
  bool initialize_imu(const Sync::DataGroup &lidar_imu);

  void relocalize();
  bool search_gnss(Sophus::SE3d &gnss_pose);
  void multi_level_align(Sophus::SE3d &pose, double &score);

  void predict();
  void undistort();
  void correct();

  pcl::PointCloud<pcl::PointXYZI>::Ptr desampling(const double leaf_sz);
  Sophus::SE3d interpolation(const double ratio, const IMUState &state0,
                             const IMUState &state1) const;
  Sophus::SE3d integrate_imu(const IMUState &state, const IMUPtr &imu_measure,
                             const double dt) const;

  LidarPointCloudPtr scan_undistort_;
  PointCloudPtr current_scan_;
  void normalize_vel();
};