#include "NDT.h"

#include <Eigen/SVD>
#include <execution>
#include <numeric>
#include <queue>

#include <glog/logging.h>

void NDT::set_neighbors(const NeighborType type) {
  if (type == NeighborType::NB0) {
    neighbors_ = {VoxelId(0, 0, 0)};
  } else if (type == NeighborType::NB6) {
    neighbors_ = {VoxelId(0, 0, 0), VoxelId(1, 0, 0),  VoxelId(-1, 0, 0),
                  VoxelId(0, 1, 0), VoxelId(0, -1, 0), VoxelId(0, 0, 1),
                  VoxelId(0, 0, -1)};
  } else if (type == NeighborType::NB14) {
    neighbors_ = {
        VoxelId(0, 0, 0),   VoxelId(1, 0, 0),   VoxelId(-1, 0, 0),
        VoxelId(0, 1, 0),   VoxelId(0, -1, 0),  VoxelId(0, 0, 1),
        VoxelId(0, 0, -1),  VoxelId(1, 1, 1),   VoxelId(1, 1, -1),
        VoxelId(1, -1, 1),  VoxelId(-1, 1, 1),  VoxelId(-1, -1, 1),
        VoxelId(1, -1, -1), VoxelId(-1, 1, -1),
    };
  }
}

bool NDT::set_target_cloud(NDT::PointCloudPtr cloud) {
  target_cloud_ = cloud;
  if (target_cloud_->empty()) {
    return false;
  }
  if (params_.guess_translation) {
    target_center_ =
        std::accumulate(target_cloud_->points.begin(),
                        target_cloud_->points.end(),
                        Eigen::Vector3d::Zero().eval(),
                        [this](const Eigen::Vector3d &sum,
                               const pcl::PointXYZI &pt) -> Eigen::Vector3d {
                          return sum + pos(pt);
                        }) /
        target_cloud_->size();
    LOG(INFO) << "target center: " << target_center_.transpose();
  }
  return build_grid();
}

bool NDT::set_source_cloud(NDT::PointCloudPtr cloud) {
  source_cloud_ = cloud;
  if (source_cloud_->empty()) {
    return false;
  }
  if (params_.guess_translation) {
    source_center_ =
        std::accumulate(source_cloud_->points.begin(),
                        source_cloud_->points.end(),
                        Eigen::Vector3d::Zero().eval(),
                        [this](const Eigen::Vector3d &sum,
                               const pcl::PointXYZI &pt) -> Eigen::Vector3d {
                          return sum + pos(pt);
                        }) /
        source_cloud_->size();
    LOG(INFO) << "source center: " << source_center_.transpose();
  }

  return true;
}

bool NDT::build_grid() {
  if (target_cloud_->empty()) {
    return false;
  }
  grid_.clear();
  std::vector<size_t> idx(target_cloud_->size());
  std::iota(idx.begin(), idx.end(), 0);

  std::for_each(idx.begin(), idx.end(), [this](const size_t tid) {
    const VoxelId vid = get_id(pos(target_cloud_->points[tid]));
    if (grid_.find(vid) == grid_.end()) {
      grid_.insert(std::pair(vid, Voxel(tid)));
    } else {
      grid_[vid].pids.emplace_back(tid);
    }
  });

  std::for_each(
      std::execution::par_unseq, grid_.begin(), grid_.end(), [this](auto &pr) {
        Voxel &voxel = pr.second;
        if (voxel.pids.size() < params_.min_vx_pt) {
          return;
        }
        mean_cov(voxel, voxel.mean, voxel.cov);
        Eigen::JacobiSVD svd(voxel.cov,
                             Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Vector3d singulars = svd.singularValues();
        singulars[1] = std::max(singulars[1], singulars[0] * 1e-3);
        singulars[2] = std::max(singulars[2], singulars[0] * 1e-3);
        Eigen::Matrix3d singulars_inv =
            Eigen::Vector3d(1.0 / singulars[0], 1.0 / singulars[1],
                            1.0 / singulars[2])
                .asDiagonal();
        voxel.info = svd.matrixV() * singulars_inv * svd.matrixU().transpose();
      });

  //   for (auto it = grid_.cbegin(); it != grid_.end();) {
  //     if (it->second.pids.size() < params_.min_vx_pt) {
  //       it = grid_.erase(it);
  //     } else {
  //       ++it;
  //     }
  //   }
  // LOG(INFO) << "number of voxels in grid: " << grid_.size();
  // LOG(INFO) << "total number of pt: " << point_num();

  return true;
}

bool NDT::mean_cov(const Voxel &voxel, Eigen::Vector3d &mean,
                   Eigen::Matrix3d &cov) const {
  if (voxel.pids.size() < 2) {
    return false;
  }
  mean = std::accumulate(voxel.pids.begin(), voxel.pids.end(),
                         Eigen::Vector3d::Zero().eval(),
                         [this](const Eigen::Vector3d &sum,
                                const size_t pid) -> Eigen::Vector3d {
                           return sum + pos(target_cloud_->points[pid]);
                         }) /
         voxel.pids.size();
  cov = std::accumulate(voxel.pids.begin(), voxel.pids.end(),
                        Eigen::Matrix3d::Zero().eval(),
                        [&mean, this](const Eigen::Matrix3d &sum,
                                      const size_t pid) -> Eigen::Matrix3d {
                          const Eigen::Vector3d diff =
                              pos(target_cloud_->points[pid]) - mean;
                          return sum + diff * diff.transpose();
                        }) /
        (voxel.pids.size() - 1);
  return true;
}

bool NDT::nearest_neighbors(const pcl::PointXYZI &query_pt, const size_t k,
                            std::vector<int> &nearest_idx,
                            std::vector<double> &nearest_dist) {
  if (grid_.empty()) {
    return false;
  }
  if (neighbors_.empty()) {
    set_neighbors(params_.nb_type);
  }
  nearest_idx.clear();
  nearest_idx.reserve(k);
  nearest_dist.clear();
  nearest_dist.reserve(k);

  std::priority_queue<std::pair<double, int>> candidates;

  const VoxelId cur_id = get_id(pos(query_pt));
  for (const auto &nb : neighbors_) {

    auto nb_it = grid_.find(cur_id + nb);
    if (nb_it == grid_.end()) {
      continue;
    }
    for (const size_t tid : nb_it->second.pids) {
      const double sq_dist =
          (pos(query_pt) - pos(target_cloud_->points[tid])).squaredNorm();
      if (candidates.size() < k) {
        candidates.emplace(sq_dist, tid);
      } else if (candidates.top().first > sq_dist) {
        candidates.pop();
        candidates.emplace(sq_dist, tid);
      }
    }
  }

  nearest_dist.resize(candidates.size());
  nearest_idx.resize(candidates.size());
  for (int i = candidates.size() - 1; i >= 0; --i) {
    const auto &[sq_dist, nidx] = candidates.top();
    candidates.pop();
    nearest_dist[i] = sq_dist;
    nearest_idx[i] = nidx;
  }

  return true;
}

bool NDT::nearest_neighbors_kmt(
    const PointCloudPtr &query_pc, const size_t k,
    std::vector<std::vector<int>> &nearest_idx,
    std::vector<std::vector<double>> &nearest_dist) {
  if (query_pc->empty() || grid_.empty()) {
    return false;
  }
  const size_t sz = query_pc->size();
  nearest_idx.resize(sz);
  nearest_dist.resize(sz);

  std::vector<size_t> idx(sz);
  std::iota(idx.begin(), idx.end(), 0);

  std::for_each(
      std::execution::par_unseq, idx.begin(), idx.end(),
      [this, &k, &query_pc, &nearest_idx, &nearest_dist](const size_t qid) {
        nearest_neighbors(query_pc->points[qid], k, nearest_idx[qid],
                          nearest_dist[qid]);
      });

  return true;
}

bool NDT::align(Sophus::SE3d &Tts) {
  if (target_cloud_->empty() || source_cloud_->empty()) {
    return false;
  }
  if (neighbors_.empty()) {
    set_neighbors(params_.nb_type);
  }
  const size_t sz = source_cloud_->size();
  size_t nb_num = neighbors_.size();
  const size_t edge_sz = sz * nb_num;

  Sophus::SE3d pose = Tts;
  if (params_.guess_translation) {
    pose.translation() = target_center_ - source_center_;
    LOG(INFO) << "Initial translation: " << pose.translation().transpose();
  }

  std::vector<size_t> idx(sz);
  std::iota(idx.begin(), idx.end(), 0);

  std::vector<Mat36, Eigen::aligned_allocator<Mat36>> Js(edge_sz);
  std::vector<Vec3, Eigen::aligned_allocator<Vec3>> es(edge_sz);
  std::vector<VoxelId, Eigen::aligned_allocator<VoxelId>> vids(edge_sz);
  std::vector<bool> valid(edge_sz, false);

  double last_chi2 = std::numeric_limits<double>::max();
  int valid_cnt = 0;

  for (int i = 0; i < params_.iterations; ++i) {
    double cur_chi2 = 0.0;
    valid_cnt = 0;

    std::for_each(std::execution::par_unseq, idx.begin(), idx.end(),
                  [this, &Js, &es, &valid, &vids, &cur_chi2, &valid_cnt,
                   &nb_num, &pose](const size_t sid) {
                    const Eigen::Vector3d query_pt =
                        pose * pos(source_cloud_->points[sid]);
                    const VoxelId cur_id = get_id(query_pt);

                    for (size_t nid = 0; nid < nb_num; ++nid) {
                      const auto &nb = neighbors_[nid];
                      auto nb_it = grid_.find(cur_id + nb);
                      const size_t eid = sid * nb_num + nid;

                      if (nb_it == grid_.end() ||
                          nb_it->second.pids.size() < params_.min_vx_pt) {
                        valid[eid] = false;
                        continue;
                      }
                      const Voxel &voxel = nb_it->second;
                      es[eid] = query_pt - voxel.mean;
                      const double chi2 =
                          es[eid].transpose() * voxel.info * es[eid];

                      if (std::isnan(chi2) || chi2 > params_.chi2_th) {
                        valid[eid] = false;
                        continue;
                      }
                      Js[eid].block<3, 3>(0, 0) =
                          -pose.so3().matrix() *
                          Sophus::SO3d::hat(pos(source_cloud_->points[sid]));
                      Js[eid].block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();
                      vids[eid] = nb_it->first;
                      valid[eid] = true;

                      cur_chi2 += chi2;
                      valid_cnt++;
                    }
                  });
    if (valid_cnt < params_.min_valid) {
      // LOG(INFO) << "not enough valid pt: " << valid_cnt;
      return false;
    }
    const double avg_chi2 = cur_chi2 / valid_cnt;
    // LOG(INFO) << "It " << i << " cur_chi2: " << cur_chi2
    //           << ", valid_cnt: " << valid_cnt << ", avg_chi2: " << avg_chi2;
    if (avg_chi2 > 1.2 * last_chi2) {
      // LOG(INFO) << "Stop optimizing.";
      break;
    }
    last_chi2 = avg_chi2;

    Mat6 H = Mat6::Zero();
    Vec6 b = Vec6::Zero();
    for (size_t eid = 0; eid < valid.size(); ++eid) {
      if (valid[eid]) {
        H += Js[eid].transpose() * grid_[vids[eid]].info * Js[eid];
        b += -Js[eid].transpose() * grid_[vids[eid]].info * es[eid];
      }
    }
    Vec6 delta = H.ldlt().solve(b);
    if (std::isnan(delta[0]) || delta.norm() < params_.eps) {
      // LOG(INFO) << "coverged, delta: " << delta.transpose();
      break;
    }
    pose.so3() = pose.so3() * Sophus::SO3d::exp(delta.head<3>());
    pose.translation() = pose.translation() + delta.tail<3>();
  }
  Tts = pose;

  return true;
}

size_t NDT::point_num() const {
  size_t cnt = 0;
  for (const auto &[vid, vx] : grid_) {
    cnt += vx.pids.size();
  }
  return cnt;
}
