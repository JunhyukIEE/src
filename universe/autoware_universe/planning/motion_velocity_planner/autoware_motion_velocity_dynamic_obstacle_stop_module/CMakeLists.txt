cmake_minimum_required(VERSION 3.14)
project(autoware_motion_velocity_dynamic_obstacle_stop_module)

find_package(autoware_cmake REQUIRED)
autoware_package()
pluginlib_export_plugin_description_file(autoware_motion_velocity_planner plugins.xml)

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)

if(BUILD_TESTING)
  find_package(rosbag2_cpp REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  ament_add_ros_isolated_gtest(test_${PROJECT_NAME}
    test/test_object_filtering.cpp
    test/test_crossing.cpp
    test/test_crossing_bag.cpp
  )
  target_link_libraries(test_${PROJECT_NAME}
    ${PROJECT_NAME}
  )
  ament_target_dependencies(test_${PROJECT_NAME} rosbag2_cpp)
endif()

ament_auto_package(INSTALL_TO_SHARE config)
