#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_transport/reader_writer_factory.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <functional>
#include <map>
#include <string>

class BagIO {
public:
  using ProcessFunc = std::function<bool(
      rosbag2_storage::SerializedBagMessageConstSharedPtr msg)>;
  using PointCloud2Handle =
      std::function<bool(std::unique_ptr<sensor_msgs::msg::PointCloud2>)>;
  using IMUHandle = std::function<bool(std::unique_ptr<sensor_msgs::msg::Imu>)>;
  using GNSSHandle =
      std::function<bool(std::unique_ptr<sensor_msgs::msg::NavSatFix>)>;

  explicit BagIO(const std::string &file) : bag_file_(file){};

  BagIO &AddHandle(const std::string &topic_name, ProcessFunc process_func);
  BagIO &AddPointCloudHandle(const std::string &topic_name,
                             PointCloud2Handle func);
  BagIO &AddIMUHandle(const std::string &topic_name, IMUHandle func);
  BagIO &AddGNSSHandle(const std::string &topic_name, GNSSHandle func);

  void clearHandles() { process_funcs_.clear(); }

  void Process();

private:
  std::map<std::string, ProcessFunc> process_funcs_;
  std::string bag_file_;
};