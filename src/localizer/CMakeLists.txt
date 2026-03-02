
add_library(localizer eskf.cpp LocalizerEskf.cpp tools.cpp)
target_link_libraries(localizer sensors yaml-cpp::yaml-cpp Eigen3::Eigen Sophus::Sophus TBB::tbb ${PCL_LIBRARIES} rclcpp::rclcpp ${sensor_msgs_TARGETS})
target_include_directories(localizer PRIVATE ${CMAKE_SOURCE_DIR}/include/localization)

add_library(localizer_mt eskf.cpp LocalizerEskfMT.cpp tools.cpp)
target_link_libraries(localizer_mt gui sensors yaml-cpp::yaml-cpp Eigen3::Eigen Sophus::Sophus TBB::tbb ${PCL_LIBRARIES} rclcpp::rclcpp ${sensor_msgs_TARGETS})
target_include_directories(localizer_mt PUBLIC ${CMAKE_SOURCE_DIR}/include/localization ${CMAKE_SOURCE_DIR}/3rdparty)
