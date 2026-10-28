cmake_minimum_required(VERSION 3.14)
project(tier4_adapi_rviz_plugin)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(Qt5 REQUIRED Core Widgets)
set(QT_LIBRARIES Qt5::Widgets)
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/route_tool.cpp
  src/route_panel.cpp
  src/goal_setter_panel.cpp
  src/operation_mode_debug_panel.cpp
  src/door_panel.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${QT_LIBRARIES}
)

pluginlib_export_plugin_description_file(rviz_common plugins/plugin_description.xml)

if(BUILD_TESTING)
  ament_add_gtest(test_kcity_course test/test_kcity_course.cpp)
  target_include_directories(test_kcity_course PRIVATE src)
  ament_target_dependencies(test_kcity_course autoware_adapi_v1_msgs geometry_msgs)
  ament_add_gtest(test_kcity_speed_limit test/test_kcity_speed_limit.cpp)
  target_include_directories(test_kcity_speed_limit PRIVATE src)
endif()

ament_auto_package(INSTALL_TO_SHARE icons)
