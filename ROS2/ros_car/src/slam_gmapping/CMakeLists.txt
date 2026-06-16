cmake_minimum_required(VERSION 3.5)
project(slam_gmapping)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(openslam_gmapping QUIET)

if(NOT openslam_gmapping_FOUND)
  find_path(OPENSLAM_GMAPPING_INCLUDE_DIR
    NAMES gmapping/gridfastslam/gridslamprocessor.h
    PATHS
      /opt/ros/$ENV{ROS_DISTRO}/include
      /usr/local/include
      /usr/include
  )
  find_library(OPENSLAM_GMAPPING_LIBRARY
    NAMES openslam_gmapping
    PATHS
      /opt/ros/$ENV{ROS_DISTRO}/lib
      /usr/local/lib
      /usr/lib
      /usr/lib/aarch64-linux-gnu
      /usr/lib/x86_64-linux-gnu
  )

  if(NOT OPENSLAM_GMAPPING_INCLUDE_DIR OR NOT OPENSLAM_GMAPPING_LIBRARY)
    message(FATAL_ERROR
      "Could not find OpenSlam GMapping headers or library. "
      "Install it with: sudo apt install ros-$ENV{ROS_DISTRO}-openslam-gmapping")
  endif()
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # remove the line when a copyright and license is present in all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # remove the line when this package is a git repo
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(include)

add_executable(slam_gmapping src/slam_gmapping.cpp)

ament_target_dependencies(slam_gmapping
        rclcpp
        tf2
        tf2_ros
        message_filters
        sensor_msgs
        nav_msgs
        geometry_msgs
        tf2_geometry_msgs)

if(openslam_gmapping_FOUND)
  ament_target_dependencies(slam_gmapping openslam_gmapping)
else()
  target_include_directories(slam_gmapping PRIVATE ${OPENSLAM_GMAPPING_INCLUDE_DIR})
  target_link_libraries(slam_gmapping ${OPENSLAM_GMAPPING_LIBRARY})
endif()

install(TARGETS slam_gmapping
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch params rviz
	    DESTINATION share/${PROJECT_NAME})

ament_package()
