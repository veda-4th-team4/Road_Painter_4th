#include "NetworkManager.h"
#include "PathFollower.h"
#include "SerialManager.h"
#include "ImuManager.h"
#include "LedStripManager.h"
#include "AudioStripManager.h"
#include <algorithm>
#include <iostream>
#include <string>
#include <unistd.h>
#include <chrono>
#include <vector>
#include <cmath>

int main(int argc, char **argv) {
  // Parse target server IP (default: 192.168.0.8)
  std::string server_ip = (argc > 1) ? argv[1] : "192.168.0.8";
  const uint16_t server_port = 9000; // Target TLS port defined in ICD

  // 1. Initialize modular components
  SerialManager robot_comm("/dev/serial0", 115200);
  NetworkManager net_manager(server_ip, server_port);
  PathFollower path_follower;
  ImuManager imu_manager;
  LedStripManager tower_lamp(5); // Changed from 7 to 5 LEDs
  if (!tower_lamp.Open()) {
      std::cerr << "[MAIN] Warning: Tower lamp failed to open." << std::endl;
  }
  tower_lamp.SetState(LedState::IDLE);

  AudioStripManager audio;
  const bool audio_ready = audio.Open();
  if (!audio_ready) {
      std::cerr << "[MAIN] Warning: Audio strip failed to open." << std::endl;
  } else {
      audio.Play(SoundId::POWER_ON);
  }

  // 2. Initialize serial communications
  if (!robot_comm.Init()) {
    std::cerr << "[MAIN] Error: Failed to build communication bridge. Exiting."
              << std::endl;
    return 1;
  }

  // Clear startup ESTOP latch on STM32
  robot_comm.SendClearEStop();
  usleep(50000);

  // Transmit dynamic RPi servo PWM configuration (RPI_SERVO_OFF_US, RPI_SERVO_ON_US) to STM32
  robot_comm.SendSetServoConfig(RPI_SERVO_OFF_US, RPI_SERVO_ON_US);
  std::cout << "[MAIN] Transmitted dynamic servo config to STM32: OFF="
            << RPI_SERVO_OFF_US << "us, ON=" << RPI_SERVO_ON_US << "us" << std::endl;
  usleep(150000);

  // 3. Initialize IMU sensor manager (returns false gracefully if I2C hardware is offline)
  if (!imu_manager.Init()) {
      std::cout << "[MAIN] Warning: IMU not detected. Running on step odometry fallback." << std::endl;
  }

  // =========================================================================
  // IMU Closed-Loop Feature Flags (Toggle ON/OFF easily)
  // =========================================================================
  bool g_use_imu_turn = false; // Disabled per user request (pure step odometry)
  bool g_use_imu_move = false; // Disabled per user request

  // 4. Initialize network communications
  std::cout << "[MAIN] Starting TLS network link to " << server_ip << ":"
            << server_port << "..." << std::endl;
  if (!net_manager.Init()) {
    std::cerr << "[MAIN] Warning: Network link failed. Starting in local "
                 "test-only mode."
              << std::endl;
    audio.Play(SoundId::SIGNAL_LOST);
  }

  std::cout << "[MAIN] Main Controller Sequence Active (v0.3 Protocol with IMU Support)." << std::endl;
  std::cout << "[MAIN] IMU Configuration: TURN_IMU=" << (g_use_imu_turn ? "ENABLED" : "DISABLED")
            << " | MOVE_IMU=" << (g_use_imu_move ? "ENABLED" : "DISABLED") << std::endl;

  auto last_status_time = std::chrono::steady_clock::now();
  bool waiting_for_go = false;
  uint32_t ready_seg_sent = 0xFFFFFFFF; // Track last segment index READY was sent for
  bool path_done_sent = false;          // Track PATH_DONE transmission per path
  static std::vector<Segment_t> pending_path;
  static bool has_pending_path = false;

  enum class NozzleSubSeq { OFFSET_MOVE, WAIT_DELAY };

  bool manual_override = false;
  Msg_SetSpeed_t manual_speed = {0, 0};
  uint8_t manual_nozzle = 0;
  uint8_t auto_nozzle = 0;

  while (true) {
      // 1. Run network loop to check sockets and reconnect if needed
      net_manager.Process();

      // 1b. Check for intrusion alarm from CCTV zone enter event
      if (net_manager.CheckAndClearZoneEnterEvent()) {
          std::cout << GetTimestampStr() << "[MAIN] Zone enter event triggered -> Playing PERSON_IN_ZONE alarm." << std::endl;
          audio.Play(SoundId::PERSON_IN_ZONE);
      }

      // 2. Handle incoming CMD (ESTOP / RESUME / ABORT_DRAW / Manual Controls) relay to STM32
      std::string cmd;
      if (net_manager.GetLatestCommand(cmd)) {
          std::cout << GetTimestampStr() << "[MAIN] Processing server CMD: " << cmd << std::endl;
          if (cmd == "ESTOP") {
              tower_lamp.SetState(LedState::ESTOP);
              audio.Play(SoundId::EMERGENCY_STOP);
              robot_comm.SendEmergencyStop(0x01);
              manual_override = true;
              manual_speed = {0, 0};
              manual_nozzle = 0;
              auto_nozzle = 0;
              robot_comm.SendControlNozzle(0);
          } else if (cmd == "ABORT_DRAW" || cmd == "CANCEL_DRAW") {
              tower_lamp.SetState(LedState::IDLE);
              audio.Play(SoundId::SYSTEM_ERROR);
              std::cout << GetTimestampStr() << "[MAIN] [ABORT] Aborting active path execution and stopping robot." << std::endl;
              manual_override = false;
              manual_speed = {0, 0};
              manual_nozzle = 0;
              auto_nozzle = 0;
              robot_comm.SendSetSpeed(0, 0);
              robot_comm.SendControlNozzle(0);
              path_follower.SetPath({});
              has_pending_path = false;
              pending_path.clear();
              waiting_for_go = false;
              ready_seg_sent = 0xFFFFFFFF;
              path_done_sent = true; // A-3: Prevent false PATH_DONE reporting
              net_manager.ClearLatches();
          } else if (cmd == "STOP") { // A-4: STOP only clears manual speed
              if (tower_lamp.GetState() != LedState::ESTOP) tower_lamp.SetState(LedState::IDLE);
              manual_speed = {0, 0};
              robot_comm.SendSetSpeed(0, 0);
          } else if (cmd == "RESUME") {
              tower_lamp.SetState(LedState::IDLE);
              robot_comm.SendClearEStop();
              manual_override = false;
              manual_speed = {0, 0};
              manual_nozzle = 0;
          } else if (cmd == "FORWARD") {
              tower_lamp.SetState(LedState::MANUAL);
              manual_override = true;
              manual_speed = {500, 500};
          } else if (cmd == "BACKWARD") {
              tower_lamp.SetState(LedState::MANUAL);
              manual_override = true;
              manual_speed = {-500, -500};
          } else if (cmd == "TURN_LEFT") {
              tower_lamp.SetState(LedState::MANUAL);
              manual_override = true;
              manual_speed = {-300, 300};
          } else if (cmd == "TURN_RIGHT") {
              tower_lamp.SetState(LedState::MANUAL);
              manual_override = true;
              manual_speed = {300, -300};
          } else if (cmd == "NOZZLE_DOWN" || cmd == "PAINT_ON") {
              manual_override = true; // §4.1 Fix: Enable manual override so IDLE manual nozzle persists
              manual_nozzle = 1;
              robot_comm.SendControlNozzle(1);
          } else if (cmd == "NOZZLE_UP" || cmd == "PAINT_OFF") {
              manual_override = true; // §4.1 Fix: Enable manual override so IDLE manual nozzle persists
              manual_nozzle = 0;
              robot_comm.SendControlNozzle(0);
          } else if (cmd == "CALIB_START") { // R-1: Homography calibration motion start request
              std::cout << GetTimestampStr() << "[MAIN] [CALIB] CALIB_START received. Initializing calibration state (nozzle UP)." << std::endl;
              manual_override = false;
              manual_speed = {0, 0};
              manual_nozzle = 0;
              auto_nozzle = 0;
              robot_comm.SendControlNozzle(0); // Ensure nozzle is strictly UP during calibration
          } else if (cmd == "CALIB_CANCEL" || cmd == "CANCEL_CALIB") { // R-2: Homography calibration cancellation ACK request
              std::cout << GetTimestampStr() << "[MAIN] [CALIB] CALIB_CANCEL received. Stopping robot, raising nozzle, and sending CALIB_STOPPED." << std::endl;
              manual_override = false;
              manual_speed = {0, 0};
              manual_nozzle = 0;
              auto_nozzle = 0;
              robot_comm.SendSetSpeed(0, 0);
              robot_comm.SendControlNozzle(0);
              path_follower.SetPath({});
              has_pending_path = false;
              pending_path.clear();
              waiting_for_go = false;
              ready_seg_sent = 0xFFFFFFFF;
              path_done_sent = true;
              net_manager.ClearLatches();
              // Settling delay: Ensure robot is fully still for 100ms before transmitting CALIB_STOPPED ACK
              std::this_thread::sleep_for(std::chrono::milliseconds(100));
              net_manager.SendCalibStopped();
          } else if (cmd == "ALARM") {
              // Server-triggered intrusion alarm: Play PERSON_IN_ZONE alarm without interrupting driving
              std::cout << GetTimestampStr() << "[MAIN] [ALARM] Intrusion alarm received from server -> Playing PERSON_IN_ZONE." << std::endl;
              audio.Play(SoundId::PERSON_IN_ZONE);
          }
      }

       // 3. Handle incoming PATH (segments sequence)
       // Defense logic: Buffer incoming new PATH and defer loading until current active segment movement completes!
       std::vector<Segment_t> new_path;
       if (net_manager.GetPath(new_path)) {
           pending_path = new_path;
           has_pending_path = true;
           std::cout << GetTimestampStr() << "[MAIN] New PATH received from server (buffered until current segment completes)" << std::endl;
       }

        if (has_pending_path) {
           // Apply new PATH only when robot is at a standstill (not in middle of active straight move, turn, or arc)
           if (path_follower.IsPathFinished() || (!path_follower.IsMovingStraight() && !path_follower.IsTurning() && !path_follower.IsArc())) {
               path_follower.SetPath(pending_path);
               waiting_for_go = false;
               ready_seg_sent = 0xFFFFFFFF;
               path_done_sent = false;  // Reset PATH_DONE tracker for new path
               manual_override = false; // Reset manual override upon receiving autonomous path
               manual_nozzle = 0;
               has_pending_path = false;
               net_manager.ClearLatches(); // R-8: Clear any stale command latches from previous path
               tower_lamp.SetState(LedState::DRIVING);
               robot_comm.SendClearEStop(); // Clear startup/idle ESTOP latch when applying new autonomous path
               audio.Play(SoundId::TASK_START);
               std::string phase = net_manager.GetPathPhase();
               std::cout << GetTimestampStr() << "[MAIN] Applying new PATH (phase=" << phase << ") -> Waiting 2500ms for camera settling..." << std::endl;

               // Settling delay: Ensure robot is fully still for 2500ms (2.5s) before sending initial READY for op 0
               auto start_wait = std::chrono::steady_clock::now();
               while (std::chrono::duration_cast<std::chrono::milliseconds>(
                          std::chrono::steady_clock::now() - start_wait).count() < 2500) {
                   robot_comm.SendSetSpeed(0, 0);
                   std::this_thread::sleep_for(std::chrono::milliseconds(20));
               }
           }
       }

      // 4. Check HOLD state and DRIFT feedback from server (~2.5Hz)
      bool hold_active = net_manager.IsHoldActive();
      if (hold_active) {
          static bool hold_logged = false;
          if (!hold_logged) {
              std::cout << GetTimestampStr() << "[MAIN] HOLD active (pos lost). Pausing robot movement..." << std::endl;
              hold_logged = true;
          }
      } else {
          static bool hold_logged = false;
          if (hold_logged) {
              std::cout << GetTimestampStr() << "[MAIN] HOLD released. Resuming autonomous path execution." << std::endl;
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
                  std::cout << GetTimestampStr() << "[MAIN] [DRIFT] Server drift correction received for op " 
                            << active_op_index << ": " << drift_angle << " deg" << std::endl;
                  path_follower.SetDriftOffset(drift_angle);
              }

              // R-5: Unified READY -> GO handshake for ALL ops ("nozzle", "move", "turn", "arc")
              if (ready_seg_sent != active_op_index) {
                  robot_comm.SendSetSpeed(0, 0);

                  // Mechanical settling delay before sending READY to server
                  // (skip for op 0 since we already did a 2.5s wait at the start of the path)
                  if (active_op_index > 0) {
                      // std::cout << GetTimestampStr() << "[MAIN] Waiting 500ms for mechanical settling..." << std::endl;
                      auto start_wait = std::chrono::steady_clock::now();
                      while (std::chrono::duration_cast<std::chrono::milliseconds>(
                                 std::chrono::steady_clock::now() - start_wait).count() < 500) {
                          robot_comm.SendSetSpeed(0, 0);
                          std::this_thread::sleep_for(std::chrono::milliseconds(20));
                      }
                  }

                  net_manager.SendReady(active_op_index);
                  ready_seg_sent = active_op_index;
                  waiting_for_go = true;
                  std::cout << GetTimestampStr() << "[MAIN] Sent READY for op " << active_op_index << " (" << current_seg.op 
                            << "), waiting for GO/ALIGN/MORE..." << std::endl;
              }

              if (waiting_for_go) {
                  // Check for ALIGN micro-rotation request
                  float align_deg = 0.0f;
                  if (net_manager.GetAlignCommand(active_op_index, align_deg)) {
                      if (!path_follower.IsTurning()) {
                          // Protocol rule: positive angle = turn right (CW) -> StartTurn(-align_deg)
                          float robot_turn_deg = -align_deg;
                          std::cout << GetTimestampStr() << "[MAIN ALIGN] Executing ALIGN micro-turn for op " << active_op_index
                                    << ": server=" << align_deg << " deg -> robot=" << robot_turn_deg << " deg" << std::endl;
                          Msg_Status_t status_snap{};
                          if (robot_comm.GetLatestStatus(status_snap)) {
                              float cur_yaw = g_use_imu_turn ? imu_manager.GetYaw() : 0.0f;
                              path_follower.StartTurn(robot_turn_deg, static_cast<int32_t>(status_snap.left_steps), static_cast<int32_t>(status_snap.right_steps), cur_yaw);
                          }
                      }
                  }

                  // Check for MORE micro-distance request
                  float more_dist = 0.0f;
                  if (net_manager.GetMoreCommand(active_op_index, more_dist)) {
                      if (!path_follower.IsMovingStraight() && !path_follower.IsTurning()) {
                          std::cout << GetTimestampStr() << "[MAIN MORE] Executing MORE micro-move for op " << active_op_index
                                    << ": " << more_dist << " m" << std::endl;
                          Msg_Status_t status_snap{};
                          if (robot_comm.GetLatestStatus(status_snap)) {
                              // MORE micro-distance correction (0.8cm~2.0cm): Use 0.013 m/s (~200 sps) micro-creeping speed
                              path_follower.StartMove(more_dist, static_cast<int32_t>(status_snap.left_steps), static_cast<int32_t>(status_snap.right_steps), 0.013f);
                          }
                      }
                  }

                  if (path_follower.IsTurning()) {
                      Msg_Status_t status_snap{};
                      if (robot_comm.GetLatestStatus(status_snap)) {
                          bool has_turn_imu = g_use_imu_turn && imu_manager.IsHealthy();
                          float cur_yaw = has_turn_imu ? imu_manager.GetYaw() : 0.0f;
                          if (path_follower.UpdateTurn(static_cast<int32_t>(status_snap.left_steps), static_cast<int32_t>(status_snap.right_steps), target_speed, cur_yaw, has_turn_imu)) {
                              // Micro-turn completed: Send stop & immediately send READY (Server handles 1s camera settling)
                              robot_comm.SendSetSpeed(0, 0);
                              std::cout << GetTimestampStr() << "[MAIN ALIGN] Turn complete -> Sending READY immediately (Server will wait 1s)" << std::endl;
                              
                              net_manager.SendReady(active_op_index);
                          }
                      }
                  }

                  if (path_follower.IsMovingStraight()) {
                      Msg_Status_t status_snap{};
                      if (robot_comm.GetLatestStatus(status_snap)) {
                          bool has_move_imu = g_use_imu_move && imu_manager.IsHealthy();
                          float imu_yaw = has_move_imu ? imu_manager.GetYaw() : 0.0f;
                          if (path_follower.UpdateMove(static_cast<int32_t>(status_snap.left_steps), static_cast<int32_t>(status_snap.right_steps), target_speed, imu_yaw)) {
                              // Micro-move completed: Send stop & immediately send READY (Server handles 1s camera settling)
                              robot_comm.SendSetSpeed(0, 0);
                              std::cout << GetTimestampStr() << "[MAIN MORE] Move complete -> Sending READY immediately (Server will wait 1s)" << std::endl;

                              net_manager.SendReady(active_op_index);
                          }
                      }
                  }

                  // Check for GO signal matching active_op_index
                  if (net_manager.CheckAndClearGoSignal(active_op_index)) {
                      waiting_for_go = false;
                      std::cout << GetTimestampStr() << "[MAIN] GO signal received for op " << active_op_index << " (" << current_seg.op << ")" << std::endl;
                      
                      // Fix: Reset IMU Yaw exactly at the moment of the first GO signal to eliminate wait-time drift
                      if (active_op_index == 0 || path_follower.GetCurrentSegmentIndex() == 0) {
                          imu_manager.ResetYaw(0.0f);
                          std::this_thread::sleep_for(std::chrono::milliseconds(5));
                      }

                      Msg_Status_t status_snap{};
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
                          std::cout << GetTimestampStr() << "[MAIN NOZZLE] Op " << active_op_index 
                                    << " set (down=" << (current_seg.down ? "true" : "false") 
                                    << ", 2500ms delay)..." << std::endl;
                          std::this_thread::sleep_for(std::chrono::milliseconds(2500)); // 2500ms (2.5s) actuator & camera settling delay
                          // R-4: Do NOT send SendReady here! AdvanceSegment() lets next loop iteration send READY(active_op_index + 1)
                          path_follower.AdvanceSegment();
                      } else if (current_seg.op == "move") {
                          if (!path_follower.IsMovingStraight()) {
                              path_follower.StartMove(current_seg.dist_m, l_steps, r_steps);
                          }

                          std::string phase = net_manager.GetPathPhase();
                          bool has_move_imu = g_use_imu_move && imu_manager.IsHealthy() && (phase != "calib");
                          float imu_yaw = has_move_imu ? imu_manager.GetYaw() : 0.0f;
                          if (path_follower.UpdateMove(l_steps, r_steps, target_speed, imu_yaw)) {
                              std::cout << GetTimestampStr() << "[MAIN MOVE] Op " << active_op_index << " complete." << std::endl;
                              
                              // User request: Recalibrate IMU after segment to guarantee stability (ONLY in calib phase)
                              if ((g_use_imu_turn || g_use_imu_move) && phase == "calib") {
                                  std::cout << GetTimestampStr() << "[MAIN MOVE] [CALIB PHASE] Waiting 3000ms for mechanical settling..." << std::endl;
                                  robot_comm.SendSetSpeed(0, 0);
                                  
                                  // 1. Mechanical settling delay (3000ms)
                                  auto start_mech = std::chrono::steady_clock::now();
                                  while (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start_mech).count() < 3000) {
                                      robot_comm.SendSetSpeed(0, 0);
                                      std::this_thread::sleep_for(std::chrono::milliseconds(20));
                                  }
                                  
                                  // 2. Dynamic IMU recalibration
                                  imu_manager.Calibrate();
                                  auto start_wait = std::chrono::steady_clock::now();
                                  while (std::chrono::duration_cast<std::chrono::milliseconds>(
                                             std::chrono::steady_clock::now() - start_wait).count() < 2000) {
                                      robot_comm.SendSetSpeed(0, 0);
                                      std::this_thread::sleep_for(std::chrono::milliseconds(20));
                                  }
                                  
                                  // 3. Post-calibration breath delay (1000ms)
                                  std::cout << GetTimestampStr() << "[MAIN MOVE] Calibration done. Waiting 1000ms before READY..." << std::endl;
                                  auto start_breath = std::chrono::steady_clock::now();
                                  while (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start_breath).count() < 1000) {
                                      robot_comm.SendSetSpeed(0, 0);
                                      std::this_thread::sleep_for(std::chrono::milliseconds(20));
                                  }
                              } else {
                                  robot_comm.SendSetSpeed(0, 0); // draw/approach: stop instantly without 6s delay
                              }

                              // R-4: Do NOT send SendReady here! AdvanceSegment() lets next loop iteration send READY(active_op_index + 1)
                              path_follower.AdvanceSegment();
                          }
                      } else if (current_seg.op == "turn") {
                          std::string phase = net_manager.GetPathPhase();
                          bool has_turn_imu = g_use_imu_turn && imu_manager.IsHealthy() && (phase != "calib");
                          if (!path_follower.IsTurning()) {
                              // Protocol rule: positive angle = turn right (CW) -> StartTurn(-angle_deg)
                              float robot_turn_deg = -current_seg.angle_deg;
                              float start_yaw = has_turn_imu ? imu_manager.GetYaw() : 0.0f;
                              path_follower.StartTurn(robot_turn_deg, l_steps, r_steps, start_yaw);
                          }

                          float cur_yaw = has_turn_imu ? imu_manager.GetYaw() : 0.0f;
                          if (path_follower.UpdateTurn(l_steps, r_steps, target_speed, cur_yaw, has_turn_imu)) {
                              std::cout << GetTimestampStr() << "[MAIN TURN] Op " << active_op_index << " in-place turn (" 
                                        << current_seg.angle_deg << " deg) complete." << std::endl;
                                        
                              // User request: Recalibrate IMU after 90-degree turn to guarantee stability (ONLY in calib phase)
                              if ((g_use_imu_turn || g_use_imu_move) && phase == "calib") {
                                  std::cout << GetTimestampStr() << "[MAIN TURN] [CALIB PHASE] Waiting 3000ms for mechanical settling..." << std::endl;
                                  robot_comm.SendSetSpeed(0, 0);
                                  
                                  // 1. Mechanical settling delay (3000ms)
                                  auto start_mech = std::chrono::steady_clock::now();
                                  while (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start_mech).count() < 3000) {
                                      robot_comm.SendSetSpeed(0, 0);
                                      std::this_thread::sleep_for(std::chrono::milliseconds(20));
                                  }
                                  
                                  // 2. Dynamic IMU recalibration
                                  imu_manager.Calibrate();
                                  auto start_wait = std::chrono::steady_clock::now();
                                  while (std::chrono::duration_cast<std::chrono::milliseconds>(
                                             std::chrono::steady_clock::now() - start_wait).count() < 2000) {
                                      robot_comm.SendSetSpeed(0, 0);
                                      std::this_thread::sleep_for(std::chrono::milliseconds(20));
                                  }
                                  
                                  // 3. Post-calibration breath delay (1000ms)
                                  std::cout << GetTimestampStr() << "[MAIN TURN] Calibration done. Waiting 1000ms before READY..." << std::endl;
                                  auto start_breath = std::chrono::steady_clock::now();
                                  while (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start_breath).count() < 1000) {
                                      robot_comm.SendSetSpeed(0, 0);
                                      std::this_thread::sleep_for(std::chrono::milliseconds(20));
                                  }
                              } else {
                                  robot_comm.SendSetSpeed(0, 0); // draw/approach: stop instantly without 6s delay
                              }

                              // R-4: Do NOT send SendReady here! AdvanceSegment() lets next loop iteration send READY(active_op_index + 1)
                              path_follower.AdvanceSegment();
                          }
                      } else if (current_seg.op == "arc") {
                          if (!path_follower.IsArc()) {
                              path_follower.StartArc(current_seg.radius_m, current_seg.angle_deg, current_seg.direction, l_steps, r_steps);
                          }

                          if (path_follower.UpdateArc(l_steps, r_steps, target_speed)) {
                              std::cout << GetTimestampStr() << "[MAIN ARC] Op " << active_op_index << " arc complete." << std::endl;
                              // R-4: Do NOT send SendReady here! AdvanceSegment() lets next loop iteration send READY(active_op_index + 1)
                              path_follower.AdvanceSegment();
                          }
                      }
                  }
              }
          }
      } else if (path_follower.IsPathFinished() && !path_done_sent) {
          tower_lamp.SetState(LedState::IDLE);
          std::string phase = net_manager.GetPathPhase();
          net_manager.SendPathDone(phase);
          path_done_sent = true;
          audio.Play(SoundId::TASK_COMPLETE);
          std::cout << GetTimestampStr() << "[MAIN] All path segments completed! Transmitted PATH_DONE (phase=" << phase << ")" << std::endl;
      }

      // 6. Periodic UART heartbeat: Transmit controls to STM32 (every 20ms loop iteration)
      robot_comm.SendSetSpeed(target_speed.left_sps, target_speed.right_sps);
      
      // Only transmit SendControlNozzle when nozzle state changes to prevent continuous 20ms overwrite of IR remote commands
      static uint8_t last_sent_nozzle = 0xFF;
      nozzle_on = manual_override ? manual_nozzle : auto_nozzle;
      if (nozzle_on != last_sent_nozzle) {
          robot_comm.SendControlNozzle(nozzle_on);
          last_sent_nozzle = nozzle_on;
      }

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
          if (stm32_ready && std::chrono::duration_cast<std::chrono::milliseconds>(now - last_status_log_time).count() >= 5000) {
              last_status_log_time = now;
              std::cout << GetTimestampStr() << "[MAIN] STATUS sent to Server -> L: " << static_cast<int32_t>(status.left_steps) 
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