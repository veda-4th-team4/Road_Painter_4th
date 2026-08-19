#include "RobotTypes.h"
#include "SerialManager.h"
#include "ImuManager.h"
#include "PathFollower.h"
#include <chrono>
#include <cmath>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <unistd.h>

void PrintHelp() {
  std::cout << "\n========== [ Calibration Test CLI Helper (CLOSED-LOOP IMU) ] ==========\n";
  std::cout << "  move <dist_m>      : Straight move in meters with IMU yaw correction\n";
  std::cout << "  turn <angle_deg>   : Turn in degrees with IMU yaw correction\n";
  std::cout << "  nozzle <1|0>       : Control nozzle actuator (1=DOWN/ON, 0=UP/OFF)\n";
  std::cout << "  status             : Read latest telemetry status from STM32\n";
  std::cout << "  help               : Show this help message\n";
  std::cout << "  quit / exit        : Exit calibration tool\n";
  std::cout << "=========================================================================\n\n";
}

int main(int argc, char **argv) {
  std::string device = "/dev/serial0";
  if (argc > 1) {
    device = argv[1];
  }

  SerialManager robot_comm(device, 115200);
  if (!robot_comm.Init()) {
    std::cerr << GetTimestampStr()
              << "[CALIB] Error: Failed to open serial port " << device
              << std::endl;
    return 1;
  }

  ImuManager imu_manager;
  if (!imu_manager.Init()) {
    std::cerr << GetTimestampStr() << "[CALIB] Warning: Failed to init IMU. Yaw will be 0.0" << std::endl;
  }

  PathFollower path_follower;

  std::cout << GetTimestampStr() << "[CALIB] Successfully connected to STM32 via " << device << std::endl;
  std::cout << GetTimestampStr() << "[CALIB] NOTE: Calibration Constants are now managed inside PathFollower.cpp!\n";

  PrintHelp();

  std::string line;
  while (true) {
    std::cout << "[CALIB-CLI]> ";
    if (!std::getline(std::cin, line))
      break;
    if (line.empty())
      continue;

    std::istringstream iss(line);
    std::string cmd;
    iss >> cmd;

    if (cmd == "quit" || cmd == "exit") {
      std::cout << GetTimestampStr() << "[CALIB] Exiting calibration tool." << std::endl;
      break;
    } else if (cmd == "help") {
      PrintHelp();
    } else if (cmd == "status") {
      Msg_Status_t status{};
      if (robot_comm.GetLatestStatus(status)) {
        std::cout << GetTimestampStr() << "[STATUS] Left Steps: "
                  << static_cast<int32_t>(status.left_steps)
                  << " | Right Steps: "
                  << static_cast<int32_t>(status.right_steps) << " | Flags: 0x"
                  << std::hex << (int)status.flags << std::dec << " | Yaw: " << imu_manager.GetYaw() << " deg" << std::endl;
      } else {
        std::cout << GetTimestampStr()
                  << "[STATUS] Warning: No telemetry received from STM32."
                  << std::endl;
      }
    } else if (cmd == "nozzle") {
      int state = 0;
      if (iss >> state) {
        robot_comm.SendControlNozzle(state ? 1 : 0);
        std::cout << GetTimestampStr() << "[CALIB NOZZLE] Command sent: "
                  << (state ? "DOWN (ON)" : "UP (OFF)") << std::endl;
      } else {
        std::cout << "[CALIB] Usage: nozzle <1|0>" << std::endl;
      }
    } else if (cmd == "move") {
      float dist_m = 0.0f;
      if (!(iss >> dist_m)) {
        std::cout << "[CALIB] Usage: move <dist_m> (e.g. move 0.1)" << std::endl;
        continue;
      }

      uint32_t target_steps = path_follower.CalculateMoveSteps(dist_m);
      Msg_Status_t start_status{};
      robot_comm.GetLatestStatus(start_status);

      int32_t start_l = static_cast<int32_t>(start_status.left_steps);
      int32_t start_r = static_cast<int32_t>(start_status.right_steps);

      std::cout << GetTimestampStr() << "[CALIB MOVE] Command: " << dist_m
                << " m (" << (dist_m * 100.0f) << " cm)\n";
      std::cout << "             Target Steps: " << target_steps << std::endl;

      path_follower.StartMove(dist_m, start_l, start_r, 0.05f);

      auto start_time = std::chrono::steady_clock::now();
      bool completed = false;
      while (!completed) {
        Msg_Status_t cur_status{};
        if (robot_comm.GetLatestStatus(cur_status)) {
          int32_t cur_l = static_cast<int32_t>(cur_status.left_steps);
          int32_t cur_r = static_cast<int32_t>(cur_status.right_steps);
          float cur_yaw = imu_manager.GetYaw();

          Msg_SetSpeed_t out_speed{};
          completed = path_follower.UpdateMove(cur_l, cur_r, out_speed, cur_yaw);
          robot_comm.SendSetSpeed(out_speed.left_sps, out_speed.right_sps);
        }

        auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                              std::chrono::steady_clock::now() - start_time).count();
        if (elapsed_ms > 30000) { // 30s safety timeout
          std::cout << GetTimestampStr() << "[CALIB MOVE] Timeout (30s) reached!" << std::endl;
          break;
        }
        usleep(10000); // 10ms loop
      }

      robot_comm.SendSetSpeed(0, 0); // Stop
      std::this_thread::sleep_for(std::chrono::milliseconds(200));

      Msg_Status_t end_status{};
      robot_comm.GetLatestStatus(end_status);
      int32_t final_l = std::abs(static_cast<int32_t>(end_status.left_steps) - start_l);
      int32_t final_r = std::abs(static_cast<int32_t>(end_status.right_steps) - start_r);
      uint32_t final_avg = (final_l + final_r) / 2;

      std::cout << GetTimestampStr() << "===== [CALIB MOVE RESULT] =====\n";
      std::cout << "  Requested Dist: " << dist_m << " m (" << (dist_m * 100.0f) << " cm)\n";
      std::cout << "  Target Steps  : " << target_steps << "\n";
      std::cout << "  Actual Steps  : Left=" << final_l << ", Right=" << final_r
                << " (Avg=" << final_avg << ")\n";
      std::cout << "  Step Delta    : " << (final_avg - target_steps) << " steps\n";
      std::cout << "=================================\n";

    } else if (cmd == "turn") {
      float angle_deg = 0.0f;
      if (!(iss >> angle_deg)) {
        std::cout << "[CALIB] Usage: turn <angle_deg> (e.g. turn 20)" << std::endl;
        continue;
      }

      uint32_t target_steps = path_follower.CalculateTurnSteps(angle_deg);
      Msg_Status_t start_status{};
      robot_comm.GetLatestStatus(start_status);

      int32_t start_l = static_cast<int32_t>(start_status.left_steps);
      int32_t start_r = static_cast<int32_t>(start_status.right_steps);
      float start_yaw = imu_manager.GetYaw();

      std::cout << GetTimestampStr() << "[CALIB TURN] Command: " << angle_deg
                << " deg (" << (angle_deg > 0.0f ? "CW Right" : "CCW Left") << ")\n";
      std::cout << "             Target Steps: " << target_steps << std::endl;

      path_follower.StartTurn(angle_deg, start_l, start_r, start_yaw);

      auto start_time = std::chrono::steady_clock::now();
      bool completed = false;
      while (!completed) {
        Msg_Status_t cur_status{};
        if (robot_comm.GetLatestStatus(cur_status)) {
          int32_t cur_l = static_cast<int32_t>(cur_status.left_steps);
          int32_t cur_r = static_cast<int32_t>(cur_status.right_steps);
          float cur_yaw = imu_manager.GetYaw();

          Msg_SetSpeed_t out_speed{};
          completed = path_follower.UpdateTurn(cur_l, cur_r, out_speed, cur_yaw, true);
          robot_comm.SendSetSpeed(out_speed.left_sps, out_speed.right_sps);
        }

        auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                              std::chrono::steady_clock::now() - start_time).count();
        if (elapsed_ms > 30000) {
          std::cout << GetTimestampStr() << "[CALIB TURN] Timeout (30s) reached!" << std::endl;
          break;
        }
        usleep(10000); // 10ms loop
      }

      robot_comm.SendSetSpeed(0, 0); // Stop
      std::this_thread::sleep_for(std::chrono::milliseconds(200));

      Msg_Status_t end_status{};
      robot_comm.GetLatestStatus(end_status);
      int32_t final_l = std::abs(static_cast<int32_t>(end_status.left_steps) - start_l);
      int32_t final_r = std::abs(static_cast<int32_t>(end_status.right_steps) - start_r);
      uint32_t final_avg = (final_l + final_r) / 2;

      std::cout << GetTimestampStr() << "===== [CALIB TURN RESULT] =====\n";
      std::cout << "  Requested Angle: " << angle_deg << " deg\n";
      std::cout << "  Target Steps   : " << target_steps << "\n";
      std::cout << "  Actual Steps   : Left=" << final_l << ", Right=" << final_r << " (Avg=" << final_avg << ")\n";
      std::cout << "  Step Delta     : " << (final_avg - target_steps) << " steps\n";
      std::cout << "=================================\n";
    } else {
      std::cout << "[CALIB] Unknown command: " << cmd << ". Type 'help' for instructions." << std::endl;
    }
  }

  robot_comm.SendSetSpeed(0, 0);
  robot_comm.Close();
  return 0;
}
