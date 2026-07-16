#include "NetworkManager.h"
#include "PathFollower.h"
#include "SerialManager.h"
#include <iostream>
#include <string>
#include <unistd.h>
#include <chrono>
#include <vector>

int main(int argc, char **argv) {
  // Parse target server IP (default: 192.168.0.8)
  std::string server_ip = (argc > 1) ? argv[1] : "192.168.0.8";
  const uint16_t server_port = 9000; // Target TLS port defined in ICD

  // 1. Initialize modular components
  SerialManager robot_comm("/dev/serial0", 115200);
  NetworkManager net_manager(server_ip, server_port);
  PathFollower path_follower;

  // 2. Initialize serial communications
  if (!robot_comm.Init()) {
    std::cerr << "[MAIN] Error: Failed to build communication bridge. Exiting."
              << std::endl;
    return 1;
  }

  // 3. Initialize network communications
  std::cout << "[MAIN] Starting TLS network link to " << server_ip << ":"
            << server_port << "..." << std::endl;
  if (!net_manager.Init()) {
    std::cerr << "[MAIN] Warning: Network link failed. Starting in local "
                 "test-only mode."
              << std::endl;
  }

  std::cout << "[MAIN] Main Controller Sequence Active." << std::endl;

  auto last_status_time = std::chrono::steady_clock::now();

  while (true) {
      // 1. Run network loop to check sockets and reconnect if needed
      net_manager.Process();

      // 2. Handle incoming CMD (ESTOP / RESUME) relay to STM32
      std::string cmd;
      if (net_manager.GetLatestCommand(cmd)) {
          std::cout << "[MAIN] Relaying command to STM32: " << cmd << std::endl;
          if (cmd == "ESTOP") {
              robot_comm.SendEmergencyStop(0x01);
          } else if (cmd == "RESUME") {
              robot_comm.SendClearEStop();
          }
      }

      // 3. Handle incoming PATH (segments sequence)
      std::vector<Segment_t> path;
      if (net_manager.GetPath(path)) {
          path_follower.SetPath(path);
      }

      // 4. Update kinematics based on latest pose
      Pose_t current_pose;
      Msg_SetSpeed_t target_speed = {0, 0};
      uint8_t nozzle_on = 0;

      if (net_manager.GetLatestPose(current_pose)) {
          path_follower.Update(current_pose, target_speed, nozzle_on);
      }

      // 5. Periodic UART heartbeat: Transmit controls to STM32 (every 80ms loop iteration)
      // This frequency (~12.5Hz) is sufficient to prevent the 300ms STM32 watchdog timeout.
      robot_comm.SendSetSpeed(target_speed.left_sps, target_speed.right_sps);
      robot_comm.SendControlNozzle(nozzle_on);

      // 6. Periodic STATUS forwarding (every 500ms) from STM32 back to the TLS server
      auto now = std::chrono::steady_clock::now();
      if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_status_time).count() >= 500) {
          last_status_time = now;
          Msg_Status_t status;
          if (robot_comm.GetLatestStatus(status)) {
              net_manager.SendStatus(status);
              std::cout << "[MAIN] STATUS sent to Server -> L: " << status.left_steps 
                        << " | R: " << status.right_steps 
                        << " | Flags: 0x" << std::hex << (int)status.flags << std::dec << std::endl;
          }
      }

      // Delay loop to maintain 10-15Hz execution
      usleep(80000); // 80ms delay (~12Hz)
  }

  robot_comm.Close();
  net_manager.Close();
  return 0;
}