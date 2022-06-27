#include <iostream>
#include <thread>

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
    while (gripper.gripperIsMoving()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    std::cout << "Opening gripper...\n";
    gripper.setGripperPosition(0x00);
    while (gripper.gripperIsMoving()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    std::cout << "Half closing gripper...\n";
    gripper.setGripperPosition(0x80);
    while (gripper.gripperIsMoving()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    std::cout << "Opening gripper...\n";
    gripper.setGripperPosition(0x00);
    while (gripper.gripperIsMoving()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    std::cout << "Decreasing gripper speed...\n";
    gripper.setSpeed(0x0F);

    std::cout << "Closing gripper...\n";
    gripper.setGripperPosition(0xFF);
    while (gripper.gripperIsMoving()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    std::cout << "Increasing gripper speed...\n";
    gripper.setSpeed(0xFF);

    std::cout << "Opening gripper...\n";
    gripper.setGripperPosition(0x00);
    while (gripper.gripperIsMoving()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    return 0;
}
