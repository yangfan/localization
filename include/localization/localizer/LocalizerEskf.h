#pragma once

#include "localizer/eskf.h"
#include "localizer/tools.h"
// #include "matching/NDT.h"
// #include "matching/mypcl/ndt.h"
// #include "ref/eskf.hpp"
#include "sensors/ImuInitializer.h"
#include "sensors/LidarPointType.h"
#include "sensors/MapViewer.h"
#include "sensors/Sync.h"
#include "sensors/gnss.h"

#include <Eigen/Core>
#include <glog/logging.h>
#include <pcl/registration/ndt.h>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <unordered_map>
#include <unordered_set>

class LocalizerEskf {
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
    // NDT::Params ndt_params;
  };
  enum class EstimateMethod { GNSS, ESKF };
  bool config(const std::string &yaml);

  void add_scan(std::unique_ptr<sensor_msgs::msg::PointCloud2> scan_msg);
  void add_imu(std::unique_ptr<sensor_msgs::msg::Imu> imu_msg);
  void add_gnss(std::unique_ptr<sensor_msgs::msg::NavSatFix> gnss_msg);

  bool load_map(const Sophus::SE3d &cur_pose);
  PointCloudPtr map_cloud() { return map_.cloud; }

  void spin();

private:
  ESKF eskf_;
  ImuInitializer imu_initializer_;
  std::vector<IMUState> states_;

  // sad::ESKFD eskfd_;
  // std::vector<sad::NavStated> imu_states_;

  Map map_;
  // NDT ndt_;
  pcl::NormalDistributionsTransform<PointType, PointType> ndt_;
  // mypcl::NormalDistributionsTransform<PointType, PointType> ndt_;
  std::unique_ptr<MapViewer> viewer_;

  Sync lidar_imu_sync_;
  Sync::DataGroup lidar_imu_;
  PointCloudPtr source_cloud_;

  Sophus::SE3d T_IL_;
  Params params_;

  Eigen::Vector3d gnss_origin_;
  std::unique_ptr<GNSS> last_gnss_;

  EstimateMethod method_ = EstimateMethod::GNSS;

  bool load_map_info(const std::string &map_info);

  void process_callback(const Sync::DataGroup &lidar_imu);
  bool initialize_imu(const Sync::DataGroup &lidar_imu);

  void relocalize();
  bool search_gnss(Sophus::SE3d &gnss_pose);
  void multi_level_align(Sophus::SE3d &pose, double &score);
  // void multi_level_align(Sophus::SE3d &pose, double &score, Sophus::SE3d
  // &npose,
  //                        double &ndt_score);
  void predict();
  void undistort();
  void correct();

  pcl::PointCloud<pcl::PointXYZI>::Ptr desampling(const double leaf_sz);
  Sophus::SE3d interpolation(const double ratio, const IMUState &state0,
                             const IMUState &state1) const;
  Sophus::SE3d integrate_imu(const IMUState &state, const IMUPtr &imu_measure,
                             const double dt) const;

  // Sophus::SE3d interpolation(const double ratio, const sad::NavStated
  // &state0,
  //                            const sad::NavStated &state1) const;
  // Sophus::SE3d integrate_imu(const sad::NavStated &state,
  //                            const IMUPtr &imu_measure, const double dt)
  //                            const;

  LidarPointCloudPtr scan_undistort_;
  PointCloudPtr current_scan_;
  template <typename PointT = LidarPointType>
  PointCloudPtr ConvertToCloud(typename pcl::PointCloud<PointT>::Ptr input) {
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
  inline PointCloudPtr VoxelCloud(PointCloudPtr cloud, float voxel_size = 0.1) {
    pcl::VoxelGrid<PointType> voxel;
    voxel.setLeafSize(voxel_size, voxel_size, voxel_size);
    voxel.setInputCloud(cloud);

    PointCloudPtr output(new PointCloudType);
    voxel.filter(*output);
    return output;
  }
  void NormalizeVelocity();
  void normalize_vel();

  // Sophus::SE3d interpolation(const double query_time, const sad::NavStated
  // &state0,
  //                            const sad::NavStated &state1) const;

  // template <typename S> inline SE3 Mat4ToSE3(const Eigen::Matrix<S, 4, 4> &m)
  // {
  //   /// 对R做归一化，防止sophus里的检查不过
  //   Quatd q(m.template block<3, 3>(0, 0).template cast<double>());
  //   q.normalize();
  //   return SE3(q, m.template block<3, 1>(0, 3).template cast<double>());
  // }
};