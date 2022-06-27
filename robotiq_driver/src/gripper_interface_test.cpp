#include <iostream>

#include <robotiq_driver/robotiq_gripper_interface.hpp>

constexpr auto kComPort = "/dev/ttyUSB0";
constexpr auto kSlaveID = 0x09;

int main() {
    RobotiqGripperInterface gripper(kComPort, kSlaveID);

    std::cout << "Deactivating gripper...\n";
    gripper.deactivateGripper();

    std::cout << "Activating gripper...\n";
    if (!gripper.activateGripper()) {
        std::cout << "Failed to activate gripper.\n";
        return 1;
    }
    std::cout << "Gripper successfully activated.\n";

    std::cout << "Closing gripper...\n";
    gripper.setGripperPosition(0xFF);

    return 0;
}
