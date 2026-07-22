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

  std::cout << "[MAIN] Main Controller Sequence Active (v0.3 Protocol)." << std::endl;

  auto last_status_time = std::chrono::steady_clock::now();
  bool waiting_for_go = false;
  uint32_t ready_seg_sent = 0xFFFFFFFF; // Track last segment index READY was sent for

  while (true) {
      // 1. Run network loop to check sockets and reconnect if needed
      net_manager.Process();

      // 2. Handle incoming CMD (ESTOP / RESUME / Manual Controls) relay to STM32
      std::string cmd;
      if (net_manager.GetLatestCommand(cmd)) {
          std::cout << "[MAIN] Relaying command to STM32: " << cmd << std::endl;
          if (cmd == "ESTOP") {
              robot_comm.SendEmergencyStop(0x01);
          } else if (cmd == "RESUME") {
              robot_comm.SendClearEStop();
          } else if (cmd == "FORWARD") {
              robot_comm.SendSetSpeed(500, 500);
          } else if (cmd == "BACKWARD") {
              robot_comm.SendSetSpeed(-500, -500);
          } else if (cmd == "TURN_LEFT") {
              robot_comm.SendSetSpeed(-300, 300);
          } else if (cmd == "TURN_RIGHT") {
              robot_comm.SendSetSpeed(300, -300);
          } else if (cmd == "STOP") {
              robot_comm.SendSetSpeed(0, 0);
              robot_comm.SendControlNozzle(0);
          }
      }

      // 3. Handle incoming PATH (segments sequence)
      std::vector<Segment_t> path;
      if (net_manager.GetPath(path)) {
          path_follower.SetPath(path);
          waiting_for_go = false;
          ready_seg_sent = 0xFFFFFFFF;
          std::string phase = net_manager.GetPathPhase();
          std::cout << "[MAIN] New PATH received with phase=" << phase << std::endl;
      }

      // 4. Check DRIFT feedback from server (~5Hz)
      float drift_angle = 0.0f;
      if (net_manager.GetDriftCorrection(drift_angle)) {
          path_follower.SetDriftOffset(drift_angle);
      }

      // 5. Segment Execution Handshake State Machine
      Msg_SetSpeed_t target_speed = {0, 0};
      uint8_t nozzle_on = 0;

      if (!path_follower.IsPathFinished()) {
          Segment_t current_seg;
          if (path_follower.GetCurrentSegment(current_seg)) {
              if (current_seg.op == "MOVE") {
                  uint32_t seg_idx = static_cast<uint32_t>(path_follower.GetCurrentSegmentIndex());
                  
                  if (ready_seg_sent != seg_idx) {
                      // Stop and send READY before executing MOVE
                      robot_comm.SendSetSpeed(0, 0);
                      net_manager.SendReady(seg_idx);
                      ready_seg_sent = seg_idx;
                      waiting_for_go = true;
                      std::cout << "[MAIN] Sent READY for MOVE segment " << seg_idx << ", waiting for GO/ALIGN..." << std::endl;
                  }

                  if (waiting_for_go) {
                      // Check for ALIGN micro-rotation request
                      float align_deg = 0.0f;
                      if (net_manager.GetAlignCommand(align_deg)) {
                          std::cout << "[MAIN] Executing ALIGN micro-turn: " << align_deg << " deg" << std::endl;
                          // Micro-turn: positive = left turn
                          int16_t turn_sps = (align_deg > 0.0f) ? -200 : 200;
                          robot_comm.SendSetSpeed(-turn_sps, turn_sps);
                          usleep(200000); // 200ms spot adjustment
                          robot_comm.SendSetSpeed(0, 0);
                          
                          // Re-send READY after ALIGN adjustment
                          net_manager.SendReady(seg_idx);
                      }

                      // Check for GO signal
                      if (net_manager.CheckAndClearGoSignal()) {
                          std::cout << "[MAIN] GO signal received! Starting MOVE segment " << seg_idx << std::endl;
                          waiting_for_go = false;
                      }
                  }

                  if (!waiting_for_go) {
                      Pose_t dummy_pose{};
                      path_follower.Update(dummy_pose, target_speed, nozzle_on);
                  }
              } else {
                  // "TURN" segment runs directly without READY handshake
                  Pose_t dummy_pose{};
                  path_follower.Update(dummy_pose, target_speed, nozzle_on);
              }
          }
      }

      // 6. Periodic UART heartbeat: Transmit controls to STM32 (every 80ms loop iteration)
      robot_comm.SendSetSpeed(target_speed.left_sps, target_speed.right_sps);
      robot_comm.SendControlNozzle(nozzle_on);

      // 7. Periodic STATUS forwarding (every 500ms) from STM32 back to the TLS server
      auto now = std::chrono::steady_clock::now();
      if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_status_time).count() >= 500) {
          last_status_time = now;
          Msg_Status_t status{};
          bool stm32_ready = robot_comm.GetLatestStatus(status);
          if (!stm32_ready) {
              status.flags = 0x00; // Default IDLE state fallback if STM32 telemetry is offline
          }
          net_manager.SendStatus(status);
          if (stm32_ready) {
              std::cout << "[MAIN] STATUS sent to Server -> L: " << status.left_steps 
                        << " | R: " << status.right_steps 
                        << " | Flags: 0x" << std::hex << (int)status.flags << std::dec << std::endl;
          }
      }

      // Delay loop to maintain ~12Hz execution
      usleep(80000); // 80ms delay
  }

  robot_comm.Close();
  net_manager.Close();
  return 0;
}