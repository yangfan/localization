add_executable(main_localizer main_localizer.cpp)
target_link_libraries(main_localizer localizer glog::glog ${GFLAGS_LIBRARIES})
target_include_directories(main_localizer PRIVATE ${CMAKE_SOURCE_DIR}/include/localization)

# add_executable(main_localizer_mt main_localizer_mt.cpp)
# target_link_libraries(main_localizer_mt localizer_mt glog::glog ${GFLAGS_LIBRARIES})
# target_include_directories(main_localizer_mt PRIVATE ${CMAKE_SOURCE_DIR}/include/localization)