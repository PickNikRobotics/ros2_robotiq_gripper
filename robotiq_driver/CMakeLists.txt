cmake_minimum_required(VERSION 3.8)
project(robotiq_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(serial REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  serial
)

add_library(
  ${PROJECT_NAME}
  SHARED
  src/crc.cpp
  src/hardware_interface.cpp
  src/default_driver.cpp
  src/default_driver_factory.cpp
)
target_include_directories(
  ${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} atomic)
ament_target_dependencies(
  ${PROJECT_NAME}
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

###############################################################################
# PLUGINS

pluginlib_export_plugin_description_file(hardware_interface hardware_interface_plugin.xml)

###############################################################################
# ADDITIONAL OPTIONAL EXECUTABLES

add_executable(gripper_interface_test src/gripper_interface_test.cpp)
target_include_directories(gripper_interface_test PRIVATE include)
ament_target_dependencies(gripper_interface_test serial)
target_link_libraries(gripper_interface_test ${PROJECT_NAME})

###############################################################################
# EXPORTS

# This is necessary to allow this library’s clients to use the syntax
# target_link_libraries(client <pkg>::<target_name>)
# Without this a client cannot find this library.
# It can take an arbitrary list of targets named as EXPORT in an install call.
ament_export_targets(
  export_${PROJECT_NAME}
)
# Help downstream packages to find transitive dependencies i.e. export all
# dependencies required by a package to use this library.
# When a package calls find_package(epick_driver), CMake looks for a file
# called epick_driverConfig.cmake which sets up everything another project
# would need to depend on this one.
ament_export_dependencies(
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
# Tell downstream packages where to find our headers.
ament_export_include_directories(
  include
)
# Tell downstream packages our libraries to link against.
ament_export_libraries(
  ${PROJECT_NAME}
)

###############################################################################
# INSTALL

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  TARGETS gripper_interface_test
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY include/
  DESTINATION include
)
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

###############################################################################
# TESTS

# CTest module automatically creates a BUILD_TESTING option that selects
# whether to enable testing support (ON by default).
include(CTest)
if(BUILD_TESTING)
  add_subdirectory(tests)
endif()

ament_package()
