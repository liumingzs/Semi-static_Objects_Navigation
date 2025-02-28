cmake_minimum_required(VERSION 3.5)
project(ram_detect)


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

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(message_filters REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(image_transport REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
# Include cnpy library path
include_directories(
  /usr/local/include
  /home/lm/Desktop/SOFTWARE/cnpy
  /opt/ros/foxy/include/tf2/LinearMath)

link_directories(
  /home/lm/Desktop/SOFTWARE/cnpy/build
)

add_executable(virtual_point_to_tracking src/virtual_point_to_tracking.cpp)

# 添加 ROS 2 库依赖
ament_target_dependencies(virtual_point_to_tracking 
  rclcpp 
  geometry_msgs
  nav_msgs
  tf2_ros
  tf2_geometry_msgs
  )

# 安装可执行文件
install(TARGETS
  virtual_point_to_tracking
  DESTINATION lib/${PROJECT_NAME}
)

# Add executables and link libraries
add_executable(two_three src/2D_to3D.cpp)
ament_target_dependencies(two_three rclcpp sensor_msgs cv_bridge image_transport OpenCV)
target_link_libraries(two_three  yaml-cpp)
install(TARGETS two_three DESTINATION lib/${PROJECT_NAME})

add_executable(stereo_motion src/stereo_motion.cpp)
ament_target_dependencies(stereo_motion rclcpp sensor_msgs cv_bridge OpenCV message_filters yaml-cpp)
target_link_libraries(stereo_motion yaml-cpp)
install(TARGETS stereo_motion DESTINATION lib/${PROJECT_NAME})

add_executable(orb_diff src/orb_diff.cpp)
ament_target_dependencies(orb_diff rclcpp sensor_msgs cv_bridge OpenCV message_filters yaml-cpp)
target_link_libraries(orb_diff yaml-cpp)
install(TARGETS orb_diff DESTINATION lib/${PROJECT_NAME})

add_executable(detect_circle src/detect_circle.cpp)
ament_target_dependencies(detect_circle rclcpp sensor_msgs cv_bridge OpenCV message_filters yaml-cpp)
target_link_libraries(detect_circle yaml-cpp)
install(TARGETS detect_circle DESTINATION lib/${PROJECT_NAME})

add_executable(keyframe_pose src/keyframe_pose.cpp)
ament_target_dependencies(keyframe_pose rclcpp sensor_msgs cv_bridge OpenCV message_filters yaml-cpp)
target_link_libraries(keyframe_pose yaml-cpp)
install(TARGETS keyframe_pose DESTINATION lib/${PROJECT_NAME})

add_executable(image_viewer src/image_viewer.cpp)
ament_target_dependencies(image_viewer rclcpp sensor_msgs cv_bridge image_transport OpenCV)
install(TARGETS image_viewer DESTINATION lib/${PROJECT_NAME})

add_executable(depth_info src/depth_info.cpp)
ament_target_dependencies(depth_info rclcpp sensor_msgs cv_bridge image_transport OpenCV)
install(TARGETS depth_info DESTINATION lib/${PROJECT_NAME})

add_executable(depth_subscriber src/depth_subscriber.cpp)
ament_target_dependencies(depth_subscriber rclcpp sensor_msgs cv_bridge)
install(TARGETS depth_subscriber DESTINATION lib/${PROJECT_NAME})

add_executable(object_3d_bounding_box src/object_3d_bounding_box.cpp)
ament_target_dependencies(object_3d_bounding_box rclcpp sensor_msgs cv_bridge image_transport OpenCV)
install(TARGETS object_3d_bounding_box DESTINATION lib/${PROJECT_NAME})

add_executable(object_depth_detect src/object_depth_detect.cpp)
target_link_libraries(object_depth_detect yaml-cpp cnpy)  # 链接 cnpy 库
ament_target_dependencies(object_depth_detect rclcpp sensor_msgs cv_bridge)
install(TARGETS object_depth_detect DESTINATION lib/${PROJECT_NAME})


install(DIRECTORY launch/
        DESTINATION share/${PROJECT_NAME}/launch)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
