#include "KeyFrame.h"
#include "localizer/LocalizerEskf.h"
// #include "sensors/LidarPointType.h"
#include "sensors/MapViewer.h"

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include <thread>

// clang-format off
DEFINE_string(
    config_file,
    "/home/fan/ssd/Projects/ros2_ws/src/localization/config/mapping.yaml",
    "path of configuration file");
DEFINE_string(kf_info,
              "/home/fan/ssd/Projects/ros2_ws/src/offline_mapping/data/output/"
              "keyframes/kf_info.txt",
              "keyframe info file");

// clang-format on

bool load_keyframes(const std::string &kf_path,
                    std::vector<std::unique_ptr<KeyFrame>> &kfs) {
  LOG(INFO) << "loading kfs at " << kf_path;
  std::ifstream ifs(kf_path);
  if (!ifs.is_open()) {
    return false;
  }
  std::string line;
  std::getline(ifs, line);
  const int num = std::stoi(line);
  line.clear();
  kfs.clear();
  kfs.reserve(num);
  LOG(INFO) << "Number of keyframes: " << num;
  std::stringstream ss;
  while (std::getline(ifs, line)) {
    if (line.empty()) {
      continue;
    }
    ss << line;
    kfs.emplace_back(std::make_unique<KeyFrame>());
    kfs.back()->load(ss);
    ss.clear();
  }
  return true;
}

TEST(Map, Viewer) {
  std::vector<std::unique_ptr<KeyFrame>> kfs;
  load_keyframes(FLAGS_kf_info, kfs);
  LOG(INFO) << "Number of kfs: " << kfs.size();

  LocalizerEskf localizer;
  localizer.config(FLAGS_config_file);

  MapViewer viewer("localizer", 1.0);

  for (const auto &kf : kfs) {
    LOG(INFO) << "location: " << kf->pose_opt2.translation().transpose();
    if (localizer.load_map(kf->pose_opt2)) {
      viewer.set_map(localizer.map_cloud());
    }
    viewer.set_pose(kf->pose_opt2);
    // std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  FLAGS_stderrthreshold = google::INFO;
  FLAGS_colorlogtostderr = true;
  google::InitGoogleLogging(argv[0]);

  google::ParseCommandLineFlags(&argc, &argv, true);

  return RUN_ALL_TESTS();
}
