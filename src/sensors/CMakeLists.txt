add_library(sensors MapViewer.cpp BagIO.cpp PointCloudConverter.cpp Sync.cpp ImuInitializer.cpp gnss.cpp 3rdparty/utm.cc 3rdparty/tranmerc.cc)
target_link_libraries(sensors PUBLIC ${PCL_LIBRARIES} yaml-cpp::yaml-cpp Eigen3::Eigen Sophus::Sophus rclcpp::rclcpp rosbag2_transport::rosbag2_transport pcl_conversions::pcl_conversions ${sensor_msgs_TARGETS})
target_include_directories(sensors PUBLIC
    ${CMAKE_SOURCE_DIR}/include/localization/sensors
    ${CMAKE_SOURCE_DIR}/include/localization/sensors/3rdparty
)