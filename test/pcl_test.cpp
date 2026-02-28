#include <Eigen/Core>
#include <glog/logging.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/ndt.h>
#include <sophus/se3.hpp>

// #include "matching/NDT.h"
// #include "pclomp/ndt_omp_impl.hpp"
#include "sensors/LidarPointType.h"

// #include "matching/mypcl/ndt.h"
// #include "matching/mypcl/ndt.hpp"
// #include "mypcl/voxel_grid_covariance.hpp"

#include <fstream>

template <typename S>
inline Sophus::SE3d Mat4ToSE3(const Eigen::Matrix<S, 4, 4> &m) {
  /// 对R做归一化，防止sophus里的检查不过
  Eigen::Quaterniond q(m.template block<3, 3>(0, 0).template cast<double>());
  q.normalize();
  return Sophus::SE3d(q, m.template block<3, 1>(0, 3).template cast<double>());
}

int main(int argc, char **argv) {

  FLAGS_stderrthreshold = google::INFO;
  FLAGS_colorlogtostderr = true;
  google::InitGoogleLogging(argv[0]);

  PointCloudPtr target(new PointCloudType);
  PointCloudPtr source(new PointCloudType);
  pcl::io::loadPCDFile("/home/fan/ssd/Projects/ros2_ws/src/localization/data/"
                       "input/test/target.pcd",
                       *target);
  pcl::io::loadPCDFile("/home/fan/ssd/Projects/ros2_ws/src/localization/data/"
                       "input/test/scan.pcd",
                       *source);

  // pcl::NormalDistributionsTransform<PointType, PointType> ndt;
  // mypcl::NormalDistributionsTransform<PointType, PointType> myndt;
  // myndt.setResolution(1.0);
  // myndt.setInputTarget(target);
  // myndt.setInputSource(source);

  pcl::NormalDistributionsTransform<PointType, PointType> ndt;
  ndt.setResolution(1.0);
  ndt.setInputTarget(target);
  ndt.setInputSource(source);

  // using NDTType = pclomp::NormalDistributionsTransform<PointType, PointType>;
  // NDTType pcl_ndt_;
  // pcl_ndt_.setResolution(1.0);
  // pcl_ndt_.setNeighborhoodSearchMethod(pclomp::DIRECT26);
  // // pcl_ndt_.setStepSize(0.1);
  // pcl_ndt_.setMaximumIterations(200);
  // pcl_ndt_.setNumThreads(4);

  // pcl_ndt_.AddTarget(target);
  // pcl_ndt_.ComputeTargetGrids();
  // pcl_ndt_.setInputSource(source);
  // pcl_ndt_.initCompute();

  // NDT mndt;
  // mndt.set_target_cloud(target);
  // mndt.build_grid();
  // mndt.set_source_cloud(source);

  std::ifstream ifs("/home/fan/ssd/Projects/ros2_ws/src/localization/data/"
                    "input/test/pose.txt");
  Eigen::Matrix<double, 6, 1> log;

  for (int i = 0; i < 6; ++i) {
    ifs >> log[i];
  }
  std::cout << "before log: " << log.transpose() << '\n';
  Sophus::SE3d pose = Sophus::SE3d::exp(log);

  PointCloudPtr output(new PointCloudType);

  // myndt.align(*output, pose.matrix().cast<float>());

  // Sophus::SE3d res = Mat4ToSE3(myndt.getFinalTransformation());

  // // std::cout << "score: " << ndt.getTransformationLikelihood() << '\n';
  // std::cout << "my score: " << myndt.getTransformationProbability() << '\n';
  // std::cout << "my after log: " << res.log().transpose() << '\n';

  ndt.align(*output, pose.matrix().cast<float>());
  Sophus::SE3d res = Mat4ToSE3(ndt.getFinalTransformation());

  std::cout << "score: " << ndt.getTransformationLikelihood() << '\n';
  std::cout << "after log: " << res.log().transpose() << '\n';

  // pcl_ndt_.align(*output, pose.matrix().cast<float>());

  // Sophus::SE3d pres = Mat4ToSE3(pcl_ndt_.getFinalTransformation());
  // std::cout << "p score: " << pcl_ndt_.getTransformationProbability() <<
  // '\n'; std::cout << "p after log: " << pres.log().transpose() << '\n';

  // Sophus::SE3d mpose = pose;
  // mndt.align(mpose);
  // std::cout << "mlog: " << mpose.log().transpose() << '\n';

  return 0;
}