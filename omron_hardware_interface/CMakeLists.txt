cmake_minimum_required(VERSION 3.8)
project(omron_hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_FLAGS "-Wno-undef")

set(CMAKE_BUILD_TYPE Debug)

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(PACKAGES_REQUIRED_HERE
  rclcpp
  ros2_control
  hardware_interface
  pluginlib
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
)

set(SUPPORT_NODES_REQUIRED
  nav_msgs
  sensor_msgs
  omron_msgs
  geometry_msgs)

foreach(package IN ITEMS ${PACKAGES_REQUIRED_HERE})
  find_package(${package} REQUIRED)
endforeach()

foreach(package IN ITEMS ${SUPPORT_NODES_REQUIRED})
  find_package(${package} REQUIRED)
endforeach()

##############
#find_package(std_msgs REQUIRED)
#find_package(geometry_msgs REQUIRED)
##############

set(ADD_CPP_FILE
  src/${PROJECT_NAME}/omron_hardware_interface.cpp
)

set(ADD_HPP_FILE # only for IDE visualization
  include/${PROJECT_NAME}/omron_hardware_interface.hpp
)

add_library(omron_aria_hw_interface SHARED
  ${ADD_HPP_FILE}
  ${ADD_CPP_FILE})

if(${CMAKE_BUILD_TYPE} MATCHES "Debug")
  target_compile_definitions(omron_aria_hw_interface PRIVATE DEBUG_ON)
endif()

target_include_directories(omron_aria_hw_interface PUBLIC
  "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
  /usr/local/include/Aria/
  /usr/local/include/ArNetworking/)

target_link_libraries(omron_aria_hw_interface
  Aria
  ArNetworking)
ament_target_dependencies(omron_aria_hw_interface
  ${PACKAGES_REQUIRED_HERE}
  )


target_compile_definitions(omron_aria_hw_interface PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
pluginlib_export_plugin_description_file(hardware_interface plugin.xml)


###################
## Map and Laser ##
###################

add_executable(omron_support_nodes
  src/support_nodes.cpp
  src/${PROJECT_NAME}/omron_map_client.cpp
  src/${PROJECT_NAME}/omron_laser_client.cpp
  src/${PROJECT_NAME}/omron_goto_goal_client.cpp
  # Only for IDE
  include/${PROJECT_NAME}/omron_map_client.hpp
  include/${PROJECT_NAME}/omron_laser_client.hpp
  include/${PROJECT_NAME}/omron_goto_goal_client.hpp
  )
ament_target_dependencies(omron_support_nodes
  ${SUPPORT_NODES_REQUIRED}
  )
target_link_libraries(omron_support_nodes
  omron_aria_hw_interface
  Aria
  ArNetworking
  )
target_include_directories(omron_support_nodes PUBLIC
  "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
  /usr/local/include/Aria/
  /usr/local/include/ArNetworking/)

install(TARGETS omron_support_nodes
  DESTINATION lib/${PROJECT_NAME})


#############
## INSTALL ##
#############

install(
  TARGETS omron_aria_hw_interface
  EXPORT export_omron_aria_hw_interface
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION bin/${PROJECT_NAME}
  INCLUDES DESTINATION include/${PROJECT_NAME}
  )

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
  )

install(
  DIRECTORY urdf launch rviz
  DESTINATION share/${PROJECT_NAME}
  )

ament_export_targets(export_omron_aria_hw_interface HAS_LIBRARY_TARGET)
ament_export_dependencies(${PACKAGES_REQUIRED_HERE})
ament_export_include_directories(include)

ament_package()
