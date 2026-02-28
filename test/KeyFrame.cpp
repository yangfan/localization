#include "KeyFrame.h"

#include <glog/logging.h>
#include <iomanip>
#include <pcl/io/pcd_io.h>

void KeyFrame::save(std::ostream &os) const {
  // lio pose: [3:10], gnss pos: [10:13]
  // opt1 pose:[13:20], opt2 pose: [20:27]
  auto save_pose = [](std::ostream &os, const Sophus::SE3d &pose) {
    const Eigen::Quaterniond quat = pose.so3().unit_quaternion();
    const Eigen::Vector3d translation = pose.translation();
    os << quat.x() << " " << quat.y() << " " << quat.z() << " " << quat.w()
       << " " << translation.x() << " " << translation.y() << " "
       << translation.z() << " ";
  };
  os << std::setprecision(18) << timestamp << " " << id << " " << valid_gnss
     << " ";
  save_pose(os, pose_lio);
  os << pos_gnss.x() << " " << pos_gnss.y() << " " << pos_gnss.z() << " ";
  save_pose(os, pose_opt1);
  save_pose(os, pose_opt2);
  os << std::endl;
}

void KeyFrame::load(std::istream &is) {
  auto load_pose = [](std::istream &is) -> Sophus::SE3d {
    double x = 0.0, y = 0.0, z = 0.0, qx = 0.0, qy = 0.0, qz = 0.0, qw = 0.0;
    is >> qx >> qy >> qz >> qw >> x >> y >> z;
    return Sophus::SE3d(Sophus::SO3d(Eigen::Quaterniond(qw, qx, qy, qz)),
                        Eigen::Vector3d(x, y, z));
  };
  is >> timestamp >> id >> valid_gnss;
  pose_lio = load_pose(is);

  double x = 0.0, y = 0.0, z = 0.0;
  is >> x >> y >> z;
  pos_gnss = Eigen::Vector3d(x, y, z);

  pose_opt1 = load_pose(is);
  pose_opt2 = load_pose(is);
}
