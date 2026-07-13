#include "SerialManager.h"
#include "NetworkManager.h"
#include "PathFollower.h"
#include <iostream>
#include <unistd.h>

int main() {
    // 1. Initialize modular components
    SerialManager robot_comm("/dev/ttyAMA0", 115200);
    NetworkManager net_manager("192.168.0.10", 8080); // Default server IP and Port (stub)
    PathFollower path_follower;

    // 2. Initialize serial communications
    if (!robot_comm.Init()) {
        std::cerr << "[MAIN] Error: Failed to build communication bridge. Exiting." << std::endl;
        return 1;
    }

    // 3. Initialize network communications (Optional stub link for now)
    net_manager.Init();

    std::cout << "[MAIN] Main Controller Sequence Active." << std::endl;

    // TODO: Switch to this modular loop when integrating the Vision Server
    /*
    while (true) {
        // Run network loop to check sockets
        net_manager.Process();

        // Check if there is an updated pose from the Vision Server
        Pose_t current_pose;
        if (net_manager.GetLatestPose(current_pose)) {
            Msg_SetSpeed_t target_speed;
            uint8_t nozzle_on = 0;

            // Compute speed targets via PathFollower kinematics
            path_follower.Update(current_pose, target_speed, nozzle_on);

            // Transmit controls down to the STM32 driver
            robot_comm.SendSetSpeed(target_speed.left_sps, target_speed.right_sps);
            robot_comm.SendControlNozzle(nozzle_on);
        }

        // Delay loop to maintain 10-15Hz execution
        usleep(80000); // 80ms delay (~12Hz)
    }
    */

    // Active serial pipeline testing sequence (keeps testing original setup)
    while (true) {
        // Scenario 1: Transmit speed targets
        std::cout << "\n[MAIN] Sending speed commands L: 500, R: -500..." << std::endl;
        robot_comm.SendSetSpeed(500, -500);
        sleep(2);

        // Scenario 2: Control Spray Nozzle
        std::cout << "\n[MAIN] Sending Nozzle ON command..." << std::endl;
        robot_comm.SendControlNozzle(1);
        sleep(2);

        // Scenario 3: Trigger emergency stop test
        std::cout << "\n[MAIN] Sending Emergency Stop command..." << std::endl;
        robot_comm.SendEmergencyStop(0x01);
        sleep(5);
    }

    robot_comm.Close();
    net_manager.Close();
    return 0;
}