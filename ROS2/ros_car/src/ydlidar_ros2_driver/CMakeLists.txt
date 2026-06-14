cmake_minimum_required(VERSION 3.8)
project(ydlidar_ros2_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(YDLIDAR_SDK QUIET)

if(NOT YDLIDAR_SDK_FOUND)
  find_path(YDLIDAR_SDK_ROOT_INCLUDE_DIR
    NAMES core/common/ydlidar_def.h
    PATHS /usr/local/include /usr/include
  )
  find_path(YDLIDAR_SDK_SRC_INCLUDE_DIR
    NAMES CYdLidar.h
    PATHS /usr/local/include/src /usr/include/src
  )
  find_library(YDLIDAR_SDK_LIBRARY
    NAMES ydlidar_sdk
    PATHS /usr/local/lib /usr/lib
  )

  if(NOT YDLIDAR_SDK_ROOT_INCLUDE_DIR
      OR NOT YDLIDAR_SDK_SRC_INCLUDE_DIR
      OR NOT YDLIDAR_SDK_LIBRARY)
    message(FATAL_ERROR
      "Could not find YDLidar SDK. Run "
      "`cmake .. -DBUILD_TEST=OFF && make -j4 && sudo make install` "
      "inside src/my_pkg/third_party/ydlidar_sdk/build first."
    )
  endif()

  set(YDLIDAR_SDK_INCLUDE_DIRS
    ${YDLIDAR_SDK_ROOT_INCLUDE_DIR}
    ${YDLIDAR_SDK_SRC_INCLUDE_DIR}
  )
  set(YDLIDAR_SDK_LIBRARIES ${YDLIDAR_SDK_LIBRARY})
endif()

add_executable(ydlidar_ros2_driver_node
  src/ydlidar_ros2_driver_node.cpp
)

target_include_directories(ydlidar_ros2_driver_node PRIVATE
  ${YDLIDAR_SDK_INCLUDE_DIRS}
)

target_link_directories(ydlidar_ros2_driver_node PRIVATE
  ${YDLIDAR_SDK_LIBRARY_DIRS}
)

ament_target_dependencies(ydlidar_ros2_driver_node
  rclcpp
  sensor_msgs
)

target_link_libraries(ydlidar_ros2_driver_node
  ${YDLIDAR_SDK_LIBRARIES}
)

install(TARGETS
  ydlidar_ros2_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  params
  rviz
  startup
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
