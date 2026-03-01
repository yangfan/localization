include(FetchContent)
FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG main
)
FetchContent_MakeAvailable(googletest)

add_executable(trans_test trans_test.cpp)
target_link_libraries(trans_test gtest glog::glog)

add_executable(map_test map_test.cpp KeyFrame.cpp)
target_link_libraries(map_test sensors localizer localizer_mt gtest glog::glog Sophus::Sophus Eigen3::Eigen ${GFLAGS_LIBRARIES})
target_include_directories(map_test PRIVATE ${CMAKE_SOURCE_DIR}/include/localization)

add_executable(pangolin_test pangolin_test.cpp)
target_link_libraries(pangolin_test gui ${Pangolin_LIBRARIES} ${PCL_LIBRARIES} glog::glog ${GFLAGS_LIBRARIES})
target_include_directories(pangolin_test PRIVATE ${Pangolin_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR}/3rdparty)

# add_executable(pcl_test pcl_test.cpp)
# target_link_libraries(pcl_test mypcl NDT Sophus::Sophus Eigen3::Eigen glog::glog ${PCL_LIBRARIES})
# target_include_directories(pcl_test PRIVATE ${CMAKE_SOURCE_DIR}/include/localization)
include(GoogleTest)