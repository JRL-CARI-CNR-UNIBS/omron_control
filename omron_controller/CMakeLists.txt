cmake_minimum_required(VERSION 3.8)
project(omron_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(CMAKE_BUILD_TYPE Debug)

find_package(ament_cmake REQUIRED)

set(PACKAGES_REQUIRED_HERE
  pluginlib
  rclcpp
#  ros2_control
  controller_interface
  realtime_tools
  geometry_msgs
  nav_msgs
  std_msgs
  generate_parameter_library
  tf2
  tf2_ros
  tf2_geometry_msgs)

foreach(package IN ITEMS ${PACKAGES_REQUIRED_HERE})
  find_package(${package} REQUIRED)
endforeach()

set(ADD_CPP_FILE
  src/${PROJECT_NAME}/omron_forward_controller.cpp
)

set(ADD_HPP_FILE # only for IDE visualization
  include/${PROJECT_NAME}/omron_forward_controller.hpp
)

generate_parameter_library(omron_forward_controller_parameters
  config/omron_controller_parameters.yaml
)
generate_parameter_library(omron_position_controller_parameters
  config/omron_position_parameters.yaml
)

#########################
## Forward Controller ##
#########################

add_library(omron_forward_controller SHARED
  ${ADD_HPP_FILE}
  ${ADD_CPP_FILE})

if(${CMAKE_BUILD_TYPE} MATCHES "Debug")
  target_compile_definitions(omron_forward_controller PRIVATE DEBUG_ON)
endif()

target_include_directories(omron_forward_controller PUBLIC
  "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
  )

ament_target_dependencies(omron_forward_controller
  ${PACKAGES_REQUIRED_HERE}
  )
target_link_libraries(omron_forward_controller
  omron_forward_controller_parameters)

target_compile_definitions(omron_forward_controller PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

#########################
## Position Controller ##
#########################

#add_library(omron_position_controller SHARED
#  include/${PROJECT_NAME}/omron_position_controller.hpp
#  src/${PROJECT_NAME}/omron_position_controller.cpp
#  )

#if(${CMAKE_BUILD_TYPE} MATCHES "Debug")
#  target_compile_definitions(omron_position_controller PRIVATE DEBUG_ON)
#endif()

#target_include_directories(omron_position_controller PUBLIC
#  "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
#  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
#  )

#ament_target_dependencies(omron_position_controller
#  ${PACKAGES_REQUIRED_HERE}
#  )
#target_link_libraries(omron_position_controller
#  omron_position_controller_parameters)

#target_compile_definitions(omron_position_controller PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")


## Export Plugin ##
pluginlib_export_plugin_description_file(controller_interface forward_controller_plugin.xml)


#############
## INSTALL ##
#############
install(
  TARGETS omron_forward_controller
          omron_forward_controller_parameters
  EXPORT export_omron_forward_controller
  RUNTIME DESTINATION bin/${PROJECT_NAME}
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  )

#install(
#  TARGETS omron_position_controller
#          omron_position_controller_parameters
#  EXPORT export_omron_position_controller
#  RUNTIME DESTINATION bin/${PROJECT_NAME}
#  ARCHIVE DESTINATION lib/${PROJECT_NAME}
#  LIBRARY DESTINATION lib/${PROJECT_NAME}
#  )

install(
  DIRECTORY include
  DESTINATION include/${PROJECT_NAME})

install(
  DIRECTORY config launch
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

ament_export_targets(export_omron_forward_controller
#                     export_omron_position_controller
                     HAS_LIBRARY_TARGET)
ament_export_dependencies(${PACKAGES_REQUIRED_HERE})

ament_package()
