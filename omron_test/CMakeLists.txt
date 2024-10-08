cmake_minimum_required(VERSION 3.8)
project(omron_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(CMAKE_BUILD_TYPE Debug)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(omron_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

###############
## Test Node ##
###############
add_executable(omron_test_node src/omron_test/test_move.cpp)
ament_target_dependencies(omron_test_node
  rclcpp
  rclcpp_action
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav2_msgs
  omron_msgs
  )
install(TARGETS omron_test_node
    DESTINATION lib/${PROJECT_NAME})


##############
## Bag Node ##
##############
add_executable(omron_test_bag_node src/omron_test/save_bag.cpp)
ament_target_dependencies(omron_test_bag_node
  rclcpp
  geometry_msgs
  rosbag2_cpp
  nav_msgs
  )
install(TARGETS omron_test_bag_node
    DESTINATION lib/${PROJECT_NAME})

############
## Others ##
############

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

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
