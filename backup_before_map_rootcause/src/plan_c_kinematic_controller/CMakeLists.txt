cmake_minimum_required(VERSION 3.8)
project(plan_c_kinematic_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)

add_library(kinematic_law_lib SHARED
  src/kinematic_law.cpp
)
target_include_directories(kinematic_law_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(kinematic_law_lib
  nav2_costmap_2d
  geometry_msgs
)

# Nav2 Controller plugin library
add_library(${PROJECT_NAME} SHARED
  src/kinematic_controller.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}
  kinematic_law_lib
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_lifecycle
  nav2_core
  nav2_costmap_2d
  nav2_util
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pluginlib
)

pluginlib_export_plugin_description_file(nav2_core plugin_description.xml)

# Runtime Nav2 integration diagnostic executable
add_executable(runtime_nav2_integration_test
  src/runtime_nav2_integration_test.cpp
)
target_link_libraries(runtime_nav2_integration_test
  ${PROJECT_NAME}
  kinematic_law_lib
)
ament_target_dependencies(runtime_nav2_integration_test
  rclcpp
  nav2_core
  nav2_costmap_2d
  geometry_msgs
  nav_msgs
  tf2
  pluginlib
)

install(TARGETS
  kinematic_law_lib
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_kinematic_law test/test_kinematic_law.cpp)
  target_link_libraries(test_kinematic_law kinematic_law_lib)
endif()

ament_export_include_directories(include)
ament_export_libraries(kinematic_law_lib ${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  nav2_core
  nav2_costmap_2d
  nav2_util
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pluginlib
)
ament_package()
