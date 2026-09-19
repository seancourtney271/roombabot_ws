cmake_minimum_required(VERSION 3.8)
project(roombabot_firmware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# This package is the robot's hot path -- the ros2_control read/write pair runs at
# 100 Hz and the two serial reader threads run continuously. Default to Release so a
# plain `colcon build` with no --cmake-args still produces a binary that keeps up on
# a Pi.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(Threads REQUIRED)

ament_python_install_package(${PROJECT_NAME})

# --- Shared serial transport -------------------------------------------------
# Built once and linked into both the hardware interface plugin and the IMU bridge,
# so the framing/CRC/termios logic has exactly one implementation.
add_library(roombabot_serial_link SHARED
  src/serial_link.cpp
)
target_include_directories(roombabot_serial_link PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(roombabot_serial_link Threads::Threads)

# --- ros2_control hardware interface plugin ----------------------------------
add_library(${PROJECT_NAME} SHARED
  src/roombabot_interface.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} roombabot_serial_link)
ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

pluginlib_export_plugin_description_file(hardware_interface roombabot_interface.xml)

# --- esp32_imu bridge node ---------------------------------------------------
add_executable(imu_bridge src/imu_bridge.cpp)
target_link_libraries(imu_bridge roombabot_serial_link)
ament_target_dependencies(imu_bridge rclcpp sensor_msgs)

install(
  TARGETS roombabot_serial_link ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS imu_bridge
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch firmware
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(roombabot_serial_link ${PROJECT_NAME})
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
)

ament_package()
