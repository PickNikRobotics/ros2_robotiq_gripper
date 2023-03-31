cmake_minimum_required(VERSION 3.8)
project(robotiq_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
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

# # COMPILE
add_library(
  ${PROJECT_NAME}
  SHARED
  src/crc.cpp
  src/hardware_interface.cpp
  src/robotiq_gripper_interface.cpp
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

pluginlib_export_plugin_description_file(hardware_interface hardware_interface_plugin.xml)

add_executable(gripper_interface_test src/gripper_interface_test.cpp)
target_include_directories(gripper_interface_test PRIVATE include)
ament_target_dependencies(gripper_interface_test serial)
target_link_libraries(gripper_interface_test ${PROJECT_NAME})

# # INSTALL
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

## EXPORTS
ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_targets(
  export_${PROJECT_NAME}
)
ament_export_dependencies(
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

ament_package()
