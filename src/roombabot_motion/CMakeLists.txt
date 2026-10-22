cmake_minimum_required(VERSION 3.8)
project(roombabot_motion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(include)

ament_python_install_package(${PROJECT_NAME})

add_executable(docking_controller src/docking_controller.cpp)
ament_target_dependencies(docking_controller rclcpp std_msgs)

add_executable(pure_pursuit src/pure_pursuit.cpp)
ament_target_dependencies(pure_pursuit rclcpp geometry_msgs nav_msgs tf2_ros tf2_geometry_msgs)

add_library(pd_motion_planner SHARED src/pd_motion_planner.cpp)
ament_target_dependencies(pd_motion_planner rclcpp geometry_msgs nav_msgs std_msgs tf2_ros tf2_geometry_msgs nav2_core nav2_util nav2_costmap_2d pluginlib)

add_library(pure_pursuit_controller SHARED src/pure_pursuit_controller.cpp)
ament_target_dependencies(pure_pursuit_controller rclcpp geometry_msgs nav_msgs tf2_ros tf2_geometry_msgs nav2_core nav2_util nav2_costmap_2d pluginlib)

pluginlib_export_plugin_description_file(nav2_core controller_plugins.xml)

install(
  DIRECTORY include
  DESTINATION include
)

install(TARGETS
  docking_controller
  pure_pursuit
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  pd_motion_planner
  pure_pursuit_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  ${PROJECT_NAME}/mission_commander.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(pd_motion_planner pure_pursuit_controller)
ament_export_dependencies(rclcpp geometry_msgs nav_msgs std_msgs tf2_ros tf2_geometry_msgs nav2_core nav2_util nav2_costmap_2d pluginlib)

ament_package()
