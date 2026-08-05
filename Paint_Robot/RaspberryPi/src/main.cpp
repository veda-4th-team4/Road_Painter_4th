#include "NetworkManager.h"
#include "PathFollower.h"
#include "SerialManager.h"
#include "ImuManager.h"
#include <algorithm>
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

  enum class NozzleSubSeq { OFFSET_MOVE, WAIT_DELAY };

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
       // Defense logic: Buffer incoming new PATH and defer loading until current active segment movement completes!
       static std::vector<Segment_t> pending_path;
       static bool has_pending_path = false;

       std::vector<Segment_t> new_path;
       if (net_manager.GetPath(new_path)) {
           pending_path = new_path;
           has_pending_path = true;
           std::cout << "[MAIN] New PATH received from server (buffered until current segment completes)" << std::endl;
       }

        if (has_pending_path) {
           // Apply new PATH only when robot is at a standstill (not in middle of active straight move or turn)
           if (path_follower.IsPathFinished() || (!path_follower.IsMovingStraight() && !path_follower.IsTurning())) {
               path_follower.SetPath(pending_path);
               waiting_for_go = false;
               ready_seg_sent = 0xFFFFFFFF;
               path_done_sent = false;  // Reset PATH_DONE tracker for new path
               manual_override = false; // Reset manual override upon receiving autonomous path
               manual_nozzle = 0;
               has_pending_path = false;
               net_manager.ClearLatches(); // R-8: Clear any stale command latches from previous path
               std::string phase = net_manager.GetPathPhase();
               std::cout << "[MAIN] Applying new PATH (phase=" << phase << ")" << std::endl;
               if (phase == "draw") {
                   imu_manager.ResetYaw(0.0f); // Reset IMU Yaw to 0 deg when entering draw phase from standstill
               }
           }
       }

      // 4. Check HOLD state and DRIFT feedback from server (~2.5Hz)
      bool hold_active = net_manager.IsHoldActive();
      if (hold_active) {
          static bool hold_logged = false;
          if (!hold_logged) {
              std::cout << "[MAIN] HOLD active (pos lost). Pausing robot movement..." << std::endl;
              hold_logged = true;
          }
      } else {
          static bool hold_logged = false;
          if (hold_logged) {
              std::cout << "[MAIN] HOLD released. Resuming autonomous path execution." << std::endl;
              hold_logged = false;
          }
      }

      // 5. Server-Master Segment Execution Handshake State Machine
      Msg_SetSpeed_t target_speed = {0, 0};
      uint8_t nozzle_on = manual_override ? manual_nozzle : auto_nozzle;

      if (manual_override || hold_active) {
          target_speed = hold_active ? Msg_SetSpeed_t{0, 0} : manual_speed;
      } else if (!path_follower.IsPathFinished()) {
          Segment_t current_seg;
          if (path_follower.GetCurrentSegment(current_seg)) {
              uint32_t active_op_index = current_seg.op_index;

              // Check DRIFT feedback matching active op_index (R-6: Pass to path_follower for continuous steering)
              float drift_angle = 0.0f;
              if (net_manager.GetDriftCorrection(active_op_index, drift_angle)) {
                  std::cout << "[MAIN] [DRIFT] Server drift correction received for op " 
                            << active_op_index << ": " << drift_angle << " deg" << std::endl;
                  path_follower.SetDriftOffset(drift_angle);
              }

              // R-5: Unified READY -> GO handshake for ALL ops ("nozzle", "move", "turn", "arc")
              if (ready_seg_sent != active_op_index) {
                  net_manager.SendReady(active_op_index);
                  ready_seg_sent = active_op_index;
                  waiting_for_go = true;
                  robot_comm.SendSetSpeed(0, 0);
                  std::cout << "[MAIN] Sent READY for op " << active_op_index << " (" << current_seg.op 
                            << "), waiting for GO/ALIGN/MORE..." << std::endl;
              }

              if (waiting_for_go) {
                  // Check for ALIGN micro-rotation request
                  float align_deg = 0.0f;
                  if (net_manager.GetAlignCommand(active_op_index, align_deg)) {
                      if (!path_follower.IsTurning()) {
                          // Protocol rule: positive angle = turn right (CW) -> StartTurn(-align_deg)
                          float robot_turn_deg = -align_deg;
                          std::cout << "[MAIN ALIGN] Executing ALIGN micro-turn for op " << active_op_index
                                    << ": server=" << align_deg << " deg -> robot=" << robot_turn_deg << " deg" << std::endl;
                          Msg_Status_t status_snap{};
                          if (robot_comm.GetLatestStatus(status_snap)) {
                              path_follower.StartTurn(robot_turn_deg, static_cast<int32_t>(status_snap.left_steps), static_cast<int32_t>(status_snap.right_steps));
                          }
                      }
                  }

                  // Check for MORE micro-distance request
                  float more_dist = 0.0f;
                  if (net_manager.GetMoreCommand(active_op_index, more_dist)) {
                      if (!path_follower.IsMovingStraight() && !path_follower.IsTurning()) {
                          std::cout << "[MAIN MORE] Executing MORE micro-move for op " << active_op_index
                                    << ": " << more_dist << " m" << std::endl;
                          Msg_Status_t status_snap{};
                          if (robot_comm.GetLatestStatus(status_snap)) {
                              path_follower.StartMove(more_dist, static_cast<int32_t>(status_snap.left_steps), static_cast<int32_t>(status_snap.right_steps));
                          }
                      }
                  }

                  if (path_follower.IsTurning()) {
                      Msg_Status_t status_snap{};
                      if (robot_comm.GetLatestStatus(status_snap)) {
                          if (path_follower.UpdateTurn(static_cast<int32_t>(status_snap.left_steps), static_cast<int32_t>(status_snap.right_steps), target_speed)) {
                              // Micro-turn completed: Send stop & wait 500ms for camera settling before re-sending READY for same op
                              robot_comm.SendSetSpeed(0, 0);
                              std::cout << "[MAIN ALIGN] Turn complete -> Waiting 500ms for camera settling..." << std::endl;
                              
                              auto start_wait = std::chrono::steady_clock::now();
                              while (std::chrono::duration_cast<std::chrono::milliseconds>(
                                         std::chrono::steady_clock::now() - start_wait).count() < 500) {
                                  robot_comm.SendSetSpeed(0, 0);
                                  std::this_thread::sleep_for(std::chrono::milliseconds(20));
                              }

                              net_manager.SendReady(active_op_index);
                          }
                      }
                  }

                  if (path_follower.IsMovingStraight()) {
                      Msg_Status_t status_snap{};
                      if (robot_comm.GetLatestStatus(status_snap)) {
                          uint8_t dummy_nozzle = 0;
                          float imu_yaw = imu_manager.GetYaw();
                          if (path_follower.UpdateMove(static_cast<int32_t>(status_snap.left_steps), static_cast<int32_t>(status_snap.right_steps), target_speed, dummy_nozzle, imu_yaw)) {
                              // Micro-move completed: Send stop & wait 500ms before re-sending READY for same op
                              robot_comm.SendSetSpeed(0, 0);
                              std::cout << "[MAIN MORE] Move complete -> Waiting 500ms for camera settling..." << std::endl;
                              
                              auto start_wait = std::chrono::steady_clock::now();
                              while (std::chrono::duration_cast<std::chrono::milliseconds>(
                                         std::chrono::steady_clock::now() - start_wait).count() < 500) {
                                  robot_comm.SendSetSpeed(0, 0);
                                  std::this_thread::sleep_for(std::chrono::milliseconds(20));
                              }

                              net_manager.SendReady(active_op_index);
                          }
                      }
                  }

                  // Check for GO signal matching active_op_index
                  if (net_manager.CheckAndClearGoSignal(active_op_index)) {
                      std::cout << "[MAIN] GO signal received for op " << active_op_index 
                                << " (" << current_seg.op << ")" << std::endl;
                      waiting_for_go = false;
                  }
              }

              if (!waiting_for_go) {
                  Msg_Status_t status_snap{};
                  if (robot_comm.GetLatestStatus(status_snap)) {
                      int32_t l_steps = static_cast<int32_t>(status_snap.left_steps);
                      int32_t r_steps = static_cast<int32_t>(status_snap.right_steps);

                      if (current_seg.op == "nozzle") {
                          auto_nozzle = current_seg.down ? 1 : 0;
                          robot_comm.SendControlNozzle(auto_nozzle);
                          std::cout << "[MAIN NOZZLE] Op " << active_op_index 
                                    << " set (down=" << (current_seg.down ? "true" : "false") 
                                    << ", 1000ms delay)..." << std::endl;
                          std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // R-9: 1000ms actuator delay
                          // R-4: Do NOT send SendReady here! AdvanceSegment() lets next loop iteration send READY(active_op_index + 1)
                          path_follower.AdvanceSegment();
                      } else if (current_seg.op == "move") {
                          if (!path_follower.IsMovingStraight()) {
                              path_follower.StartMove(current_seg.dist_m, l_steps, r_steps);
                          }

                          float imu_yaw = imu_manager.GetYaw();
                          if (path_follower.UpdateMove(l_steps, r_steps, target_speed, nozzle_on, imu_yaw)) {
                              std::cout << "[MAIN MOVE] Op " << active_op_index << " complete." << std::endl;
                              // R-4: Do NOT send SendReady here! AdvanceSegment() lets next loop iteration send READY(active_op_index + 1)
                              path_follower.AdvanceSegment();
                          }
                      } else if (current_seg.op == "turn") {
                          if (!path_follower.IsTurning()) {
                              // Protocol rule: positive angle = turn right (CW) -> StartTurn(-angle_deg)
                              float robot_turn_deg = -current_seg.angle_deg;
                              path_follower.StartTurn(robot_turn_deg, l_steps, r_steps);
                          }

                          if (path_follower.UpdateTurn(l_steps, r_steps, target_speed)) {
                              std::cout << "[MAIN TURN] Op " << active_op_index << " in-place turn (" 
                                        << current_seg.angle_deg << " deg) complete." << std::endl;
                              // R-4: Do NOT send SendReady here! AdvanceSegment() lets next loop iteration send READY(active_op_index + 1)
                              path_follower.AdvanceSegment();
                          }
                      } else if (current_seg.op == "arc") {
                          if (!path_follower.IsMovingStraight()) {
                              path_follower.StartArc(current_seg.radius_m, current_seg.angle_deg, current_seg.direction, l_steps, r_steps);
                          }

                          if (path_follower.UpdateArc(l_steps, r_steps, target_speed)) {
                              std::cout << "[MAIN ARC] Op " << active_op_index << " arc complete." << std::endl;
                              // R-4: Do NOT send SendReady here! AdvanceSegment() lets next loop iteration send READY(active_op_index + 1)
                              path_follower.AdvanceSegment();
                          }
                      }
                  }
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
          static auto last_status_log_time = std::chrono::steady_clock::now();
          if (stm32_ready && std::chrono::duration_cast<std::chrono::milliseconds>(now - last_status_log_time).count() >= 2000) {
              last_status_log_time = now;
              std::cout << "[MAIN] STATUS sent to Server -> L: " << static_cast<int32_t>(status.left_steps) 
                        << " | R: " << static_cast<int32_t>(status.right_steps) 
                        << " | Flags: 0x" << std::hex << (int)status.flags << std::dec << std::endl;
          }
      }

      // Delay loop to maintain ~50Hz execution (20ms)
      usleep(20000); // 20ms delay for high-precision turn stopping
  }

  robot_comm.Close();
  net_manager.Close();
  return 0;
}