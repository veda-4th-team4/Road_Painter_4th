#include "NetworkManager.h"
#include "PathFollower.h"
#include "SerialManager.h"
#include "ImuManager.h"
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
  ImuManager imu_manager;

  // 2. Initialize serial communications
  if (!robot_comm.Init()) {
    std::cerr << "[MAIN] Error: Failed to build communication bridge. Exiting."
              << std::endl;
    return 1;
  }

  // Clear startup ESTOP latch on STM32
  robot_comm.SendClearEStop();
  usleep(200000);

  // 3. Initialize IMU sensor manager (returns false gracefully if I2C hardware is offline)
  if (!imu_manager.Init()) {
      std::cout << "[MAIN] Warning: IMU not detected. Running on step odometry fallback." << std::endl;
  }

  // 4. Initialize network communications
  std::cout << "[MAIN] Starting TLS network link to " << server_ip << ":"
            << server_port << "..." << std::endl;
  if (!net_manager.Init()) {
    std::cerr << "[MAIN] Warning: Network link failed. Starting in local "
                 "test-only mode."
              << std::endl;
  }

  std::cout << "[MAIN] Main Controller Sequence Active (v0.3 Protocol with IMU Support)." << std::endl;

  auto last_status_time = std::chrono::steady_clock::now();
  bool waiting_for_go = false;
  uint32_t ready_seg_sent = 0xFFFFFFFF; // Track last segment index READY was sent for
  bool path_done_sent = false;          // Track PATH_DONE transmission per path

  bool manual_override = false;
  Msg_SetSpeed_t manual_speed = {0, 0};
  uint8_t manual_nozzle = 0;
  uint8_t auto_nozzle = 0;

  while (true) {
      // 1. Run network loop to check sockets and reconnect if needed
      net_manager.Process();

      // 2. Handle incoming CMD (ESTOP / RESUME / Manual Controls) relay to STM32
      std::string cmd;
      if (net_manager.GetLatestCommand(cmd)) {
          std::cout << "[MAIN] Relaying command to STM32: " << cmd << std::endl;
          if (cmd == "ESTOP") {
              robot_comm.SendEmergencyStop(0x01);
              manual_override = true;
              manual_speed = {0, 0};
              manual_nozzle = 0;
              robot_comm.SendControlNozzle(0);
          } else if (cmd == "RESUME") {
              robot_comm.SendClearEStop();
              manual_override = false;
              manual_speed = {0, 0};
              manual_nozzle = 0;
          } else if (cmd == "FORWARD") {
              manual_override = true;
              manual_speed = {500, 500};
          } else if (cmd == "BACKWARD") {
              manual_override = true;
              manual_speed = {-500, -500};
          } else if (cmd == "TURN_LEFT") {
              manual_override = true;
              manual_speed = {-300, 300};
          } else if (cmd == "TURN_RIGHT") {
              manual_override = true;
              manual_speed = {300, -300};
          } else if (cmd == "STOP") {
              manual_override = true;
              manual_speed = {0, 0};
              manual_nozzle = 0;
              robot_comm.SendControlNozzle(0);
          } else if (cmd == "NOZZLE_DOWN" || cmd == "PAINT_ON") {
              manual_nozzle = 1;
              robot_comm.SendControlNozzle(1);
          } else if (cmd == "NOZZLE_UP" || cmd == "PAINT_OFF") {
              manual_nozzle = 0;
              robot_comm.SendControlNozzle(0);
          }
      }

      // 3. Handle incoming PATH (segments sequence)
      std::vector<Segment_t> path;
      if (net_manager.GetPath(path)) {
          path_follower.SetPath(path);
          waiting_for_go = false;
          ready_seg_sent = 0xFFFFFFFF;
          path_done_sent = false;  // Reset PATH_DONE tracker for new path
          manual_override = false; // Reset manual override upon receiving autonomous path
          manual_nozzle = 0;
          std::string phase = net_manager.GetPathPhase();
          std::cout << "[MAIN] New PATH received with phase=" << phase << std::endl;
          if (phase == "draw") {
              imu_manager.ResetYaw(0.0f); // Protocol requirement: Reset IMU Yaw to 0 deg when entering draw phase
          }
      }

      // 4. Check DRIFT feedback from server (~5Hz)
      float drift_angle = 0.0f;
      if (net_manager.GetDriftCorrection(drift_angle)) {
          // Protocol requirement: Ignore DRIFT feedback while executing TURN segment!
          if (!path_follower.IsTurning()) {
              path_follower.SetDriftOffset(drift_angle);
          } else {
              std::cout << "[MAIN] [DRIFT IGNORED] Robot is currently turning." << std::endl;
          }
      }

      // 5. Segment Execution Handshake State Machine
      Msg_SetSpeed_t target_speed = {0, 0};
      uint8_t nozzle_on = manual_override ? manual_nozzle : auto_nozzle;

      if (manual_override) {
          target_speed = manual_speed;
      } else if (!path_follower.IsPathFinished()) {
          Segment_t current_seg;
          if (path_follower.GetCurrentSegment(current_seg)) {
              if (current_seg.op == "MOVE") {
                  uint32_t seg_idx = static_cast<uint32_t>(path_follower.GetCurrentSegmentIndex());
                  bool bypass_server_go = true; // Set to true to bypass server GO/ALIGN wait for continuous drive
                  
                  if (ready_seg_sent != seg_idx) {
                      // Send READY to server for logging, then proceed directly if bypass enabled
                      net_manager.SendReady(seg_idx);
                      ready_seg_sent = seg_idx;
                      waiting_for_go = !bypass_server_go;
                      if (waiting_for_go) {
                          robot_comm.SendSetSpeed(0, 0);
                          std::cout << "[MAIN] Sent READY for MOVE segment " << seg_idx << ", waiting for GO/ALIGN..." << std::endl;
                      } else {
                          std::cout << "[MAIN] [BYPASS GO] Starting MOVE segment " << seg_idx << " directly." << std::endl;
                      }
                  }

                  if (waiting_for_go) {
                      // Check for ALIGN micro-rotation request
                      float align_deg = 0.0f;
                      if (net_manager.GetAlignCommand(align_deg)) {
                          std::cout << "[MAIN] Executing ALIGN micro-turn: " << align_deg << " deg" << std::endl;
                          Msg_Status_t status_snap{};
                          if (robot_comm.GetLatestStatus(status_snap)) {
                              path_follower.StartTurn(align_deg, static_cast<int32_t>(status_snap.left_steps), static_cast<int32_t>(status_snap.right_steps));
                          }
                      }

                      if (path_follower.IsTurning()) {
                          Msg_Status_t status_snap{};
                          if (robot_comm.GetLatestStatus(status_snap)) {
                              if (path_follower.UpdateTurn(static_cast<int32_t>(status_snap.left_steps), static_cast<int32_t>(status_snap.right_steps), target_speed)) {
                                  // Micro-turn completed: re-send READY
                                  net_manager.SendReady(seg_idx);
                              }
                          }
                      }

                      // Check for GO signal
                      if (net_manager.CheckAndClearGoSignal()) {
                          std::cout << "[MAIN] GO signal received! Starting MOVE segment " << seg_idx << std::endl;
                          waiting_for_go = false;
                      }
                  }

                  if (!waiting_for_go) {
                      Msg_Status_t status_snap{};
                      if (robot_comm.GetLatestStatus(status_snap)) {
                          int32_t l_steps = static_cast<int32_t>(status_snap.left_steps);
                          int32_t r_steps = static_cast<int32_t>(status_snap.right_steps);

                          if (!path_follower.IsMovingStraight()) {
                              path_follower.StartMove(current_seg.dist_m, l_steps, r_steps);
                          }

                          float imu_yaw = imu_manager.GetYaw();
                          if (path_follower.UpdateMove(l_steps, r_steps, target_speed, nozzle_on, imu_yaw)) {
                              // Move segment completed -> advance to next segment
                              path_follower.AdvanceSegment();
                          }
                      }
                  }
              } else if (current_seg.op == "TURN") {
                  // "TURN" segment runs step-based precision turning
                  Msg_Status_t status_snap{};
                  if (robot_comm.GetLatestStatus(status_snap)) {
                      int32_t l_steps = static_cast<int32_t>(status_snap.left_steps);
                      int32_t r_steps = static_cast<int32_t>(status_snap.right_steps);

                      if (!path_follower.IsTurning()) {
                          path_follower.StartTurn(current_seg.angle_deg, l_steps, r_steps);
                      }
                      
                      if (path_follower.UpdateTurn(l_steps, r_steps, target_speed)) {
                          // Turn segment completed -> advance to next segment
                          path_follower.AdvanceSegment();
                      }
                  }
              } else if (current_seg.op == "NOZZLE") {
                  // Protocol v0.3: NOZZLE op explicitly controls nozzle down/up state
                  auto_nozzle = current_seg.paint ? 1 : 0;
                  robot_comm.SendControlNozzle(auto_nozzle);
                  std::cout << "[MAIN] Executed NOZZLE op (down=" << (auto_nozzle ? "true" : "false") << ")" << std::endl;
                  path_follower.AdvanceSegment();
              }
          }
      } else if (path_follower.IsPathFinished() && !path_done_sent) {
          std::string phase = net_manager.GetPathPhase();
          net_manager.SendPathDone(phase);
          path_done_sent = true;
          std::cout << "[MAIN] All path segments completed! Transmitted PATH_DONE (phase=" << phase << ")" << std::endl;
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
              std::cout << "[MAIN] STATUS sent to Server -> L: " << static_cast<int32_t>(status.left_steps) 
                        << " | R: " << static_cast<int32_t>(status.right_steps) 
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