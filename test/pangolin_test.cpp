#include <gflags/gflags.h>
#include <glog/logging.h>
#include <pcl/io/pcd_io.h>

#include "tools/ui/pangolin_window.h"

// clang-format off
DEFINE_string(source,
              "/home/fan/ssd/Projects/ros2_ws/src/localization/data/input/test/target.pcd",
              "path of pcd file");
// clang-format on

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_stderrthreshold = google::INFO;
  FLAGS_colorlogtostderr = true;
  google::ParseCommandLineFlags(&argc, &argv, true);

  sad::ui::PangolinWindow ui;
  ui.Init();
  sad::CloudPtr source(new sad::PointCloudType);
  pcl::io::loadPCDFile(fLS::FLAGS_source, *source);

  LOG(INFO) << "set state";
  LOG(INFO) << "source size: " << source->size();
  ui.UpdateScan(source, SE3());

  LOG(INFO) << "waiting";
  sleep(60);
  ui.Quit();

  return 0;
}
