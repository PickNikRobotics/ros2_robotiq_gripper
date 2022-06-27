#include <iostream>

#include <robotiq_driver/robotiq_gripper_interface.hpp>

constexpr auto kComPort = "/dev/ttyUSB0";
constexpr auto kSlaveID = 0x09;

int main() {
    RobotiqGripperInterface gripper(kComPort, kSlaveID);

    std::cout << "Deactivating gripper...\n";
    gripper.deactivateGripper();

    std::cout << "Activating gripper...\n";
    std::cout << "Gripper activated? " << (gripper.activateGripper() ? "Yes.\n" : "No.\n");

    return 0;
}
