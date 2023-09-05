# Add support for GTest.
find_package(ament_cmake_gtest REQUIRED)
find_package(ament_cmake_gmock REQUIRED)
find_package(ament_lint_auto REQUIRED)
find_package(ros2_control_test_assets REQUIRED)

ament_lint_auto_find_test_dependencies()

# GMock throws an error if we don't switch off this option in tests.
if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
    add_compile_options(-Wno-sign-conversion)
endif()

###############################################################################
# test_robotiq_gripper_hardware_interface

ament_add_gmock(test_robotiq_gripper_hardware_interface
    test_robotiq_gripper_hardware_interface.cpp
)
target_include_directories(test_robotiq_gripper_hardware_interface
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(test_robotiq_gripper_hardware_interface robotiq_driver)
