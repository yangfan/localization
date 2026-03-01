add_library(gui
    ui/pangolin_window.cc
    ui/pangolin_window_impl.cc
    ui/ui_car.cc
    ui/ui_trajectory.cc
    ui/ui_cloud.cc

    convert.cpp
)

target_link_libraries(gui
    ${PCL_LIBRARIES}
    Sophus::Sophus
    Eigen3::Eigen
    rclcpp::rclcpp
    ${sensor_msgs_TARGETS}
    ${Pangolin_LIBRARIES}
)

target_include_directories(gui PUBLIC
    ${CMAKE_SOURCE_DIR}/3rdparty/
    ${CMAKE_SOURCE_DIR}/include/localization
)
