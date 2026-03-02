#include "localizer/LocalizerEskfMT.h"
#include "localizer/tools.h"

#include <pcl/io/pcd_io.h>
#include <yaml-cpp/yaml.h>

#include <execution>
#include <fstream>
#include <sstream>

bool LocalizerEskfMT::config(const std::string &yaml) {

  lidar_imu_sync_ = Sync([this](const Sync::DataGroup &lidar_imu) {
    process_callback(lidar_imu);
  });

  try {
    auto config = YAML::LoadFile(yaml);

    map_.dir = config["map_data"].as<std::string>();
    load_map_info(map_.dir + "submaps_info.txt");

    const std::vector<double> ext_t =
        config["mapping"]["extrinsic_T"].as<std::vector<double>>();
    const std::vector<double> ext_r =
        config["mapping"]["extrinsic_R"].as<std::vector<double>>();

    const Eigen::Vector3d lidar_T_wrt_IMU = localization::VecFromArray(ext_t);
    const Eigen::Matrix3d lidar_R_wrt_IMU = localization::MatFromArray(ext_r);
    T_IL_ = Sophus::SE3d(lidar_R_wrt_IMU, lidar_T_wrt_IMU);
    LOG(INFO) << "Lidar IMU Extrinsic T_IL: "
              << T_IL_.translation().transpose();

    std::vector<double> g_origin = config["origin"].as<std::vector<double>>();
    gnss_origin_ = Eigen::Map<Eigen::Vector3d>(g_origin.data());

    LOG(INFO) << "GNSS origin: " << gnss_origin_.transpose();

  } catch (...) {
    LOG(ERROR) << "Unable to load config file at " << yaml;
    return false;
  }
  lidar_imu_sync_.config(yaml);

  imu_initializer_.config(yaml);

  if (params_.viewer_on) {
    viewer_ = std::make_unique<sad::ui::PangolinWindow>();
    viewer_->SetWinName("Localizer");
    viewer_->Init();
    viewer_->SetCurrentScanSize(50);
  }

  ndt_.setResolution(3.0);

  return true;
};

bool LocalizerEskfMT::load_map_info(const std::string &map_info) {

  std::ifstream ifs(map_info);
  if (!ifs.is_open()) {
    LOG(WARNING) << "Unable to load map info file at " << map_info;
    return false;
  }
  std::string line;
  std::stringstream ss;
  std::getline(ifs, line);
  ss << line;
  line.clear();
  int num = 0;
  ss >> num >> map_.grid_size >> map_.grid_origin.x() >> map_.grid_origin.y();

  while (std::getline(ifs, line)) {
    if (line.empty()) {
      continue;
    }
    ss << line;
    int id_a = 0, id_b = 0;
    ss >> id_a >> id_b;
    map_.idx.emplace(id_a, id_b);

    ss.clear();
    line.clear();
  }

  LOG(INFO) << "grid sz: " << map_.grid_size
            << ", origin: " << map_.grid_origin.transpose();

  return true;
}

bool LocalizerEskfMT::initialize_imu(const Sync::DataGroup &lidar_imu) {
  for (const auto &imu : lidar_imu.imu_sequence) {
    if (imu_initializer_.addImu(*imu)) {
      break;
    }
  }
  if (imu_initializer_.success()) {
    LOG(INFO) << "init gravity: " << imu_initializer_.gravity().transpose();
    LOG(INFO) << "init bg: " << imu_initializer_.bias_g().transpose()
              << ", ba: " << imu_initializer_.bias_a().transpose();
    LOG(INFO) << "init var_g: " << imu_initializer_.var_g().transpose()
              << ", var_g: " << imu_initializer_.var_a().transpose();

    eskf_.initialize_noise(imu_initializer_);
    eskf_.initialize_pose(Sophus::SE3d(), imu_initializer_.timestamp());
  }
  return true;
}

void LocalizerEskfMT::add_scan(
    std::unique_ptr<sensor_msgs::msg::PointCloud2> scan_msg) {
  lidar_imu_sync_.add_cloud(std::move(scan_msg));
}
void LocalizerEskfMT::add_imu(std::unique_ptr<sensor_msgs::msg::Imu> imu_msg) {
  lidar_imu_sync_.add_imu(std::move(imu_msg));
}
void LocalizerEskfMT::add_gnss(
    std::unique_ptr<sensor_msgs::msg::NavSatFix> gnss_msg) {
  last_gnss_ = std::make_unique<GNSS>(*gnss_msg);
  last_gnss_->set_pos(last_gnss_->utm() - gnss_origin_);
}

void LocalizerEskfMT::process_callback(const Sync::DataGroup &lidar_imu) {
  lidar_imu_ = lidar_imu;
  if (!imu_initializer_.success()) {
    initialize_imu(lidar_imu_);
    return;
  }

  if (method_ == EstimateMethod::ESKF) {
    predict();
    undistort();
    correct();
  } else if (last_gnss_) {
    relocalize();
  }
}

bool LocalizerEskfMT::load_map(const Sophus::SE3d &cur_pose) {
  const MapId map_id = map_.map_id(cur_pose.translation().head<2>());
  map_updated_ = false;

  for (int xx = -1; xx < 2; ++xx) {
    for (int yy = -1; yy < 2; ++yy) {
      const MapId m_id(map_id.x() + xx, map_id.y() + yy);
      if (map_.idx.find(m_id) == map_.idx.end() ||
          map_.data.find(m_id) != map_.data.end()) {
        continue;
      }
      PointCloudPtr submap(new PointCloudType);
      pcl::io::loadPCDFile(map_.dir + std::to_string(m_id.x()) + '_' +
                               std::to_string(m_id.y()) + ".pcd",
                           *submap);
      LOG(INFO) << "submap " << m_id.transpose() << " loaded.";
      map_.data.insert({m_id, submap});

      map_updated_ = true;
    }
  }

  for (auto it = map_.data.begin(); it != map_.data.end();) {
    const MapId &m_id = it->first;
    if ((m_id - map_id).cast<double>().norm() > 3.0) {
      LOG(INFO) << "submap " << m_id.transpose() << " deleted.";
      it = map_.data.erase(it);
      map_updated_ = true;
    } else {
      it++;
    }
  }

  if (map_updated_) {
    map_.cloud.reset(new PointCloudType);
    for (const auto &submap : map_.data) {
      *map_.cloud += *submap.second;
    }
    if (method_ == EstimateMethod::ESKF) {
      cv_update_target_.notify_one();
    } else if (method_ == EstimateMethod::GNSS) {

      if (viewer_) {
        viewer_->UpdatePointCloudGlobal(map_.data);
      }
      ndt_.setInputTarget(map_.cloud);
      map_updated_ = false;
      thd_update_target_ = std::thread([this]() { this->update_target(); });
    }
  }

  return map_updated_;
}

void LocalizerEskfMT::update_target() {
  while (update_thd_on_) {
    std::unique_lock ndt_lock(mtx_ndt_);
    cv_update_target_.wait(ndt_lock, [this] { return map_updated_; });
    LOG(INFO) << "updating ndt target.";
    ndt_.setInputTarget(map_.cloud);
    ndt_lock.unlock();

    if (viewer_) {
      viewer_->UpdatePointCloudGlobal(map_.data);
    }
    map_updated_ = false;
  }
}

// make sure robot is static
void LocalizerEskfMT::relocalize() {
  Sophus::SE3d initial_pose;
  initial_pose.translation() = last_gnss_->pos();
  LOG(INFO) << "gnss pos: " << initial_pose.translation().transpose();
  load_map(initial_pose);

  if (!search_gnss(initial_pose)) {
    return;
  }

  auto cur_state = eskf_.state();
  cur_state.pos = initial_pose.translation();
  cur_state.rot = initial_pose.so3();
  // assuming robot not moving
  cur_state.vel = Eigen::Vector3d::Zero();
  eskf_.set_state(cur_state);

  Eigen::Matrix<double, 18, 18> cov =
      Eigen::Matrix<double, 18, 18>::Identity() * 1e-4;
  cov.block<12, 12>(6, 6) = Eigen::Matrix<double, 12, 12>::Identity() * 1e-6;
  eskf_.set_cov(cov);

  method_ = EstimateMethod::ESKF;

  if (viewer_) {
    viewer_->UpdateNavState(eskf_.state());
    viewer_->UpdateScan(source_cloud_, initial_pose);
  }
}

bool LocalizerEskfMT::search_gnss(Sophus::SE3d &initial_pose) {

  constexpr double deg_inc = 5;
  constexpr int num = 360 / deg_inc;
  using Candidate = std::pair<double, Sophus::SE3d>;
  std::vector<Candidate> candidates;
  candidates.reserve(num);

  for (double deg = 0; deg < 360; deg += deg_inc) {
    Sophus::SE3d pose(Sophus::SO3d::rotZ(localization::kdeg2rad * deg),
                      initial_pose.translation());
    candidates.emplace_back(Candidate(0.0, pose));
  }

  source_cloud_ = desampling(0.1);
  LOG(INFO) << "number of source: " << source_cloud_->size();

  std::for_each(std::execution::par_unseq, candidates.begin(), candidates.end(),
                [this](Candidate &guess) {
                  multi_level_align(guess.second, guess.first);
                });

  auto max_it = std::max_element(
      candidates.begin(), candidates.end(),
      [](const Candidate &a, const Candidate &b) { return a.first < b.first; });

  LOG(INFO) << "max score: " << max_it->first;
  if (max_it->first < params_.min_relocalize_score) {
    return false;
  }
  LOG(INFO) << "matrix:\n" << max_it->second.matrix();
  initial_pose = max_it->second;
  LOG(INFO) << "p: " << initial_pose.translation().transpose();
  LOG(INFO) << "qaut: " << initial_pose.so3().unit_quaternion();

  return true;
}

void LocalizerEskfMT::multi_level_align(Sophus::SE3d &pose, double &score) {
  if (source_cloud_->empty()) {
    return;
  }

  pcl::NormalDistributionsTransform<PointType, PointType> ndt;
  ndt.setTransformationEpsilon(0.05);
  ndt.setStepSize(0.7);
  ndt.setMaximumIterations(40);

  ndt.setInputTarget(map_.cloud);
  ndt.setInputSource(source_cloud_);
  PointCloudPtr output(new PointCloudType);

  Eigen::Matrix<float, 4, 4> T_w_s = pose.matrix().cast<float>();
  const std::vector<double> resolutions = {10.0, 5.0, 4.0, 3.0};

  for (const auto &res : resolutions) {
    ndt.setResolution(res);
    ndt.align(*output, T_w_s);
    T_w_s = ndt.getFinalTransformation();
  }
  Eigen::Quaterniond quat(T_w_s.block<3, 3>(0, 0).cast<double>());
  quat.normalize();
  const Eigen::Vector3d trans(T_w_s.block<3, 1>(0, 3).cast<double>());

  pose = Sophus::SE3d(quat, trans);
  score = ndt.getTransformationLikelihood();
}

pcl::PointCloud<pcl::PointXYZI>::Ptr
LocalizerEskfMT::desampling(const double leaf_sz) {

  LidarPointCloudPtr cloud_I = lidar_imu_.scan;

  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl_cloud->points.resize(cloud_I->size());
  std::vector<size_t> idx(cloud_I->size());
  std::iota(idx.begin(), idx.end(), 0);
  std::for_each(std::execution::par_unseq, idx.begin(), idx.end(),
                [&pcl_cloud, &cloud_I](const size_t pid) {
                  pcl_cloud->points[pid].x = cloud_I->points[pid].x;
                  pcl_cloud->points[pid].y = cloud_I->points[pid].y;
                  pcl_cloud->points[pid].z = cloud_I->points[pid].z;
                  pcl_cloud->points[pid].intensity = 0;
                });
  pcl_cloud->height = 1;
  pcl_cloud->width = pcl_cloud->size();

  pcl::VoxelGrid<pcl::PointXYZI> vg;
  vg.setLeafSize(leaf_sz, leaf_sz, leaf_sz);
  vg.setInputCloud(pcl_cloud);
  pcl::PointCloud<pcl::PointXYZI>::Ptr desmapled_cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  vg.filter(*desmapled_cloud);

  return desmapled_cloud;
}

void LocalizerEskfMT::predict() {
  states_.clear();
  states_.reserve(lidar_imu_.imu_sequence.size() + 1);
  states_.emplace_back(eskf_.state());

  for (const auto &imu_data : lidar_imu_.imu_sequence) {
    eskf_.predict_imu(*imu_data);
    states_.emplace_back(eskf_.state());
  }

  return;
}

void LocalizerEskfMT::undistort() {
  auto cloud = lidar_imu_.scan;
  auto imu_state = eskf_.state();
  Sophus::SE3d Tw_end(imu_state.rot, imu_state.pos);

  std::for_each(std::execution::par_unseq, cloud->points.begin(),
                cloud->points.end(), [&](auto &pt) {
                  Sophus::SE3d Twi = Tw_end;
                  IMUState match;

                  localization::PoseInterp<IMUState>(
                      lidar_imu_.scan_start_time + pt.time * 1e-3, states_,
                      [](const IMUState &s) { return s.timestamp; },
                      [](const IMUState &s) { return s.SE3(); }, Twi, match);

                  Eigen::Vector3d pi =
                      pt.getVector3fMap().template cast<double>();
                  Eigen::Vector3d p_compensate =
                      T_IL_.inverse() * Tw_end.inverse() * Twi * T_IL_ * pi;
                  // Vec3d p_compensate = T_end.inverse() * Ti * T_IL_ * pi;

                  pt.x = p_compensate(0);
                  pt.y = p_compensate(1);
                  pt.z = p_compensate(2);
                });
  scan_undistort_ = cloud;
}

void LocalizerEskfMT::correct() {

  LidarPointCloudPtr scan_undistort_trans(new LidarPointCloud);
  pcl::transformPointCloud(*scan_undistort_, *scan_undistort_trans,
                           T_IL_.matrix());
  scan_undistort_ = scan_undistort_trans;

  current_scan_ = localization::ConvertToCloud<LidarPointType>(scan_undistort_);
  current_scan_ = localization::VoxelGridFilter(current_scan_, 0.5);

  std::unique_lock ndt_lock(mtx_ndt_);
  ndt_.setInputSource(current_scan_);
  PointCloudPtr output(new PointCloudType);
  ndt_.align(*output, eskf_.state_SE3().matrix().cast<float>());

  Sophus::SE3d pose = localization::Mat4ToSE3(ndt_.getFinalTransformation());
  LOG(INFO) << "NDT score: " << ndt_.getTransformationLikelihood();
  ndt_lock.unlock();

  eskf_.correct_pose(pose, states_.back().timestamp, 1e-1, 1e-2);
  normalize_vel();

  // LOG(INFO) << "vel: "
  //           << (eskf_.state().rot.inverse() *
  //           eskf_.state().vel).transpose();
  // LOG(INFO) << "vel norm: " << eskf_.state().vel.transpose().norm();

  if (viewer_) {
    viewer_->UpdateNavState(eskf_.state());
    viewer_->UpdateScan(current_scan_, eskf_.state_SE3());
  }

  load_map(eskf_.state_SE3());
}

Sophus::SE3d LocalizerEskfMT::integrate_imu(const IMUState &state,
                                            const IMUPtr &imu_measure,
                                            const double dt) const {
  const Eigen::Vector3d last_acc = imu_measure->acc;
  const Eigen::Vector3d last_omega = imu_measure->gyr;

  const Eigen::Vector3d pos =
      state.pos + state.vel * dt + 0.5 * state.gravity * dt * dt +
      0.5 * (state.rot * (last_acc - state.bias_a)) * dt * dt;
  const Sophus::SO3d rot =
      state.rot * Sophus::SO3d::exp((last_omega - state.bias_g) * dt);
  return Sophus::SE3d(rot, pos);
}

Sophus::SE3d LocalizerEskfMT::interpolation(const double ratio,
                                            const IMUState &state0,
                                            const IMUState &state1) const {
  const Sophus::SE3d pose0(state0.rot, state0.pos);
  const Sophus::SE3d pose1(state1.rot, state1.pos);

  const Eigen::Vector3d pos =
      (1 - ratio) * pose0.translation() + ratio * pose1.translation();
  const Sophus::SO3d rot = Sophus::SO3d(
      pose0.unit_quaternion().slerp(ratio, pose1.unit_quaternion()));

  return Sophus::SE3d(rot, pos);
}

void LocalizerEskfMT::quit() {
  update_thd_on_ = false;
  map_updated_ = true;
  cv_update_target_.notify_one();

  if (thd_update_target_.joinable()) {
    thd_update_target_.join();
  }
  if (viewer_) {
    viewer_->Quit();
  }
}

void LocalizerEskfMT::normalize_vel() {
  Eigen::Vector3d vel_body = eskf_.state().rot.inverse() * eskf_.state().vel;

  // from -0.1 to 0.1
  vel_body[0] = std::min(vel_body[0], 0.1);
  vel_body[0] = std::max(vel_body[0], -0.1);

  // from -2.0  to 0.0
  vel_body[1] = std::min(vel_body[1], 0.0);
  vel_body[1] = std::max(vel_body[1], -2.0);

  vel_body[2] = 0;

  IMUState state = eskf_.state();
  state.vel = state.rot * vel_body;
  eskf_.set_state(state);
}