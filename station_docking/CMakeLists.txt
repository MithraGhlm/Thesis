cmake_minimum_required(VERSION 3.8)
project(station_docking)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(PCL 1.3 REQUIRED)
#find_package(PCL 1.3 REQUIRED COMPONENTS common filters)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(Boost REQUIRED COMPONENTS date_time)
find_package(visualization_msgs REQUIRED)



include_directories(${PCL_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Add the executable
add_executable(detect_docking src/detect_docking_station.cpp)
add_executable(attach_docking src/attach_to_docking_station.cpp)
add_executable(cSection src/detect_line_cross_section.cpp)

# Link dependencies
ament_target_dependencies(detect_docking
  rclcpp
  std_msgs
  sensor_msgs
  laser_geometry
  visualization_msgs
  pcl_conversions
  pcl_ros
  PCL
)

ament_target_dependencies(attach_docking
  rclcpp
  std_msgs
  sensor_msgs
  laser_geometry
  visualization_msgs
  pcl_conversions
  pcl_ros
  PCL
)

ament_target_dependencies(cSection
  rclcpp
  std_msgs
  sensor_msgs
  laser_geometry
  visualization_msgs
  pcl_conversions
  pcl_ros
  PCL
)

# Install the target executable
install(TARGETS
  detect_docking
  attach_docking
  cSection
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

ament_package()
