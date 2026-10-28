cmake_minimum_required(VERSION 3.14)
project(autoware_localization_rviz_plugin)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(Qt5 REQUIRED Core Widgets)
set(QT_LIBRARIES Qt5::Widgets)
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
add_definitions(-DQT_NO_KEYWORDS)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/pose_history/pose_history_display.hpp
  src/pose_history/pose_history_display.cpp
  src/pose_history_footprint/display.hpp
  src/pose_history_footprint/display.cpp
  src/pose_with_covariance_history/pose_with_covariance_history_display.hpp
  src/pose_with_covariance_history/pose_with_covariance_history_display.cpp
  src/colored_pose_with_covariance_history/colored_pose_with_covariance_history_display.hpp
  src/colored_pose_with_covariance_history/colored_pose_with_covariance_history_display.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${QT_LIBRARIES}
  autoware_overlay_rviz_plugin::autoware_overlay_rviz_plugin
)

# Export the plugin to be imported by rviz2
pluginlib_export_plugin_description_file(rviz_common plugins/plugin_description.xml)

if(BUILD_TESTING)
  ament_add_gtest(test_value_overlay test/test_value_overlay.cpp)
  target_include_directories(test_value_overlay PRIVATE src)
  target_link_libraries(test_value_overlay Qt5::Widgets)
  set_tests_properties(test_value_overlay PROPERTIES ENVIRONMENT "QT_QPA_PLATFORM=offscreen")
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  icons
  plugins
)
