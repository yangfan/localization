#include "localizer/LocalizerEskfMT.h"
#include "sensors/BagIO.h"

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <yaml-cpp/yaml.h>

// clang-format off
DEFINE_string(
    config_file,
    "/home/fan/ssd/Projects/ros2_ws/src/localization/config/mapping.yaml",
    "path of configuration file");
// clang-format on

int main(int argc, char **argv) {

  FLAGS_stderrthreshold = google::INFO;
  FLAGS_colorlogtostderr = true;

  google::InitGoogleLogging(argv[0]);

  google::ParseCommandLineFlags(&argc, &argv, true);

  std::string lidar_topic, imu_topic, gnss_topic, bag_file;
  try {
    auto config = YAML::LoadFile(FLAGS_config_file);
    lidar_topic = config["common"]["lid_topic"].as<std::string>();
    imu_topic = config["common"]["imu_topic"].as<std::string>();
    gnss_topic = config["common"]["gnss_topic"].as<std::string>();
    bag_file = config["bag_path"].as<std::string>();

  } catch (...) {
    LOG(ERROR) << "Unable to load config file at " << FLAGS_config_file;
    return -1;
  }
  LocalizerEskfMT localizer;
  localizer.config(FLAGS_config_file);

  BagIO bag_io(bag_file);
  bag_io
      .AddPointCloudHandle(
          lidar_topic,
          [&localizer](
              std::unique_ptr<sensor_msgs::msg::PointCloud2> cloud_msg) {
            localizer.add_scan(std::move(cloud_msg));
            return true;
          })
      .AddIMUHandle(
          imu_topic,
          [&localizer](std::unique_ptr<sensor_msgs::msg::Imu> imu_msg) {
            localizer.add_imu(std::move(imu_msg));
            return true;
          })
      .AddGNSSHandle(
          gnss_topic,
          [&localizer](std::unique_ptr<sensor_msgs::msg::NavSatFix> gnss_msg) {
            localizer.add_gnss(std::move(gnss_msg));
            return true;
          })
      .Process();
  localizer.quit();

  return 0;
}