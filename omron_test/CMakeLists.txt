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
find_package(geometry_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)

###############
## Test Node ##
###############
add_executable(omron_test_node src/omron_test/test_move.cpp)
ament_target_dependencies(omron_test_node
  rclcpp
  geometry_msgs
  rosbag2_cpp
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