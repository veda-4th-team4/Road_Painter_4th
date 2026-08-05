#include "SerialManager.h"
#include "RobotTypes.h"
#include <iostream>
#include <sstream>
#include <string>
#include <chrono>
#include <cmath>
#include <thread>
#include <unistd.h>

// =================================================================
// 🎯 하드웨어 보정 상수 (Calibration Scaling Factors)
// =================================================================
// 1. 거리 보정 계수 (Distance Scaling Factor)
//    - 예: 10cm(0.1m) 명령 시 실측이 9.5cm면 FACTOR = 0.1m / 0.095m = 1.053f 로 수정
constexpr float DISTANCE_CALIB_FACTOR = 1.000f;

// 2. 회전 각도 보정 계수 (Turn Angle Scaling Factor)
//    - 예: 90도 회전 명령 시 실측이 95도 회전하면 FACTOR = 90.0f / 95.0f = 0.947f 로 수정
constexpr float TURN_ANGLE_CALIB_FACTOR = 1.000f;

// 3. 로봇 기구학 규격 상수 (Robot Physical Parameters)
constexpr float WHEEL_DIAMETER_M = 0.066f;  // 바퀴 지름 (66mm = 0.066m)
constexpr float WHEELBASE_M      = 0.285f;  // 차축 거리 (285mm = 0.285m)
constexpr float GEAR_RATIO       = 1.0f;    // 기어비 1:1
constexpr uint16_t STEPS_PER_REV = 3200;    // 1회전당 스텝수 (3200 steps/rev)

// =================================================================
// 📐 스텝수 환산 헬퍼 함수
// =================================================================
uint32_t CalculateMoveSteps(float dist_m) {
    float wheel_circ = M_PI * WHEEL_DIAMETER_M; // 0.207345m
    float steps_per_meter = (STEPS_PER_REV * GEAR_RATIO) / wheel_circ; // ~15433.09 steps/m
    float abs_dist = std::fabs(dist_m) * DISTANCE_CALIB_FACTOR;
    return static_cast<uint32_t>(std::round(abs_dist * steps_per_meter));
}

uint32_t CalculateTurnSteps(float angle_deg) {
    float turn_circ = M_PI * WHEELBASE_M; // 0.89535m
    float arc_len = turn_circ * (std::fabs(angle_deg) * TURN_ANGLE_CALIB_FACTOR / 360.0f);
    float wheel_circ = M_PI * WHEEL_DIAMETER_M;
    float steps = (arc_len / wheel_circ) * STEPS_PER_REV * GEAR_RATIO;
    return static_cast<uint32_t>(std::round(steps));
}

void PrintHelp() {
    std::cout << "\n========== [ Calibration Test CLI Helper ] ==========\n";
    std::cout << "  move <dist_m>      : Straight move in meters (e.g. 'move 0.1' = 10cm forward, 'move -0.1' = 10cm reverse)\n";
    std::cout << "  turn <angle_deg>   : Turn in degrees (e.g. 'turn 20' = 20 deg CW right, 'turn -20' = 20 deg CCW left)\n";
    std::cout << "  nozzle <1|0>       : Control nozzle actuator (1=DOWN/ON, 0=UP/OFF)\n";
    std::cout << "  status             : Read latest telemetry status from STM32\n";
    std::cout << "  help               : Show this help message\n";
    std::cout << "  quit / exit        : Exit calibration tool\n";
    std::cout << "=====================================================\n\n";
}

int main(int argc, char** argv) {
    std::string device = "/dev/serial0";
    if (argc > 1) {
        device = argv[1];
    }

    SerialManager robot_comm(device, 115200);
    if (!robot_comm.Init()) {
        std::cerr << GetTimestampStr() << "[CALIB] Error: Failed to open serial port " << device << std::endl;
        return 1;
    }

    std::cout << GetTimestampStr() << "[CALIB] Successfully connected to STM32 via " << device << std::endl;
    std::cout << GetTimestampStr() << "[CALIB] Current Calibration Factors:\n";
    std::cout << "         - DISTANCE_CALIB_FACTOR   : " << DISTANCE_CALIB_FACTOR << "\n";
    std::cout << "         - TURN_ANGLE_CALIB_FACTOR : " << TURN_ANGLE_CALIB_FACTOR << "\n";
    std::cout << "         - Wheel Diameter          : " << WHEEL_DIAMETER_M * 1000.0f << " mm\n";
    std::cout << "         - Wheelbase               : " << WHEELBASE_M * 1000.0f << " mm\n";

    PrintHelp();

    std::string line;
    while (true) {
        std::cout << "[CALIB-CLI]> ";
        if (!std::getline(std::cin, line)) break;
        if (line.empty()) continue;

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
                std::cout << GetTimestampStr() << "[STATUS] Left Steps: " << static_cast<int32_t>(status.left_steps)
                          << " | Right Steps: " << static_cast<int32_t>(status.right_steps)
                          << " | Flags: 0x" << std::hex << (int)status.flags << std::dec << std::endl;
            } else {
                std::cout << GetTimestampStr() << "[STATUS] Warning: No telemetry received from STM32." << std::endl;
            }
        } else if (cmd == "nozzle") {
            int state = 0;
            if (iss >> state) {
                robot_comm.SendControlNozzle(state ? 1 : 0);
                std::cout << GetTimestampStr() << "[CALIB NOZZLE] Command sent: " << (state ? "DOWN (ON)" : "UP (OFF)") << std::endl;
            } else {
                std::cout << "[CALIB] Usage: nozzle <1|0>" << std::endl;
            }
        } else if (cmd == "move") {
            float dist_m = 0.0f;
            if (!(iss >> dist_m)) {
                std::cout << "[CALIB] Usage: move <dist_m> (e.g. move 0.1)" << std::endl;
                continue;
            }

            uint32_t target_steps = CalculateMoveSteps(dist_m);
            Msg_Status_t start_status{};
            if (!robot_comm.GetLatestStatus(start_status)) {
                std::cout << GetTimestampStr() << "[CALIB] Warning: STM32 telemetry offline. Using 0 step baseline." << std::endl;
            }

            int32_t start_l = static_cast<int32_t>(start_status.left_steps);
            int32_t start_r = static_cast<int32_t>(start_status.right_status_or_steps ? start_status.right_steps : start_status.right_steps);

            int16_t speed_sps = (dist_m >= 0.0f) ? 771 : -771; // 5 cm/s straight move
            std::cout << GetTimestampStr() << "[CALIB MOVE] Command: " << dist_m << " m (" << (dist_m * 100.0f) << " cm)\n";
            std::cout << "             Target Steps: " << target_steps << " | Speed SPS: " << speed_sps << std::endl;

            auto start_time = std::chrono::steady_clock::now();
            bool completed = false;
            while (!completed) {
                robot_comm.SendSetSpeed(speed_sps, speed_sps);

                Msg_Status_t cur_status{};
                if (robot_comm.GetLatestStatus(cur_status)) {
                    int32_t cur_l = static_cast<int32_t>(cur_status.left_steps);
                    int32_t cur_r = static_cast<int32_t>(cur_status.right_steps);

                    uint32_t delta_l = std::abs(cur_l - start_l);
                    uint32_t delta_r = std::abs(cur_r - start_r);
                    uint32_t progress = (delta_l + delta_r) / 2;

                    if (progress >= target_steps) {
                        completed = true;
                    }
                }

                auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start_time).count();
                if (elapsed_ms > 30000) { // 30s safety timeout
                    std::cout << GetTimestampStr() << "[CALIB MOVE] Timeout (30s) reached!" << std::endl;
                    break;
                }
                usleep(20000); // 20ms loop
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
            std::cout << "  Actual Steps  : Left=" << final_l << ", Right=" << final_r << " (Avg=" << final_avg << ")\n";
            std::cout << "  Step Delta    : " << (final_avg - target_steps) << " steps\n";
            std::cout << "=================================\n";

        } else if (cmd == "turn") {
            float angle_deg = 0.0f;
            if (!(iss >> angle_deg)) {
                std::cout << "[CALIB] Usage: turn <angle_deg> (e.g. turn 20)" << std::endl;
                continue;
            }

            uint32_t target_steps = CalculateTurnSteps(angle_deg);
            Msg_Status_t start_status{};
            robot_comm.GetLatestStatus(start_status);

            int32_t start_l = static_cast<int32_t>(start_status.left_steps);
            int32_t start_r = static_cast<int32_t>(start_status.right_steps);

            // Protocol v2 sign convention: positive angle = turn right (CW) -> Left Forward (+), Right Reverse (-)
            int16_t l_sps = (angle_deg > 0.0f) ? 500 : -500;
            int16_t r_sps = (angle_deg > 0.0f) ? -500 : 500;

            std::cout << GetTimestampStr() << "[CALIB TURN] Command: " << angle_deg << " deg (" << (angle_deg > 0.0f ? "CW Right" : "CCW Left") << ")\n";
            std::cout << "             Target Steps: " << target_steps << " | Speed SPS (L: " << l_sps << ", R: " << r_sps << ")\n";

            auto start_time = std::chrono::steady_clock::now();
            bool completed = false;
            while (!completed) {
                robot_comm.SendSetSpeed(l_sps, r_sps);

                Msg_Status_t cur_status{};
                if (robot_comm.GetLatestStatus(cur_status)) {
                    int32_t cur_l = static_cast<int32_t>(cur_status.left_steps);
                    int32_t cur_r = static_cast<int32_t>(cur_status.right_steps);

                    uint32_t delta_l = std::abs(cur_l - start_l);
                    uint32_t delta_r = std::abs(cur_r - start_r);
                    uint32_t progress = (delta_l + delta_r) / 2;

                    if (progress >= target_steps) {
                        completed = true;
                    }
                }

                auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start_time).count();
                if (elapsed_ms > 30000) {
                    std::cout << GetTimestampStr() << "[CALIB TURN] Timeout (30s) reached!" << std::endl;
                    break;
                }
                usleep(20000);
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
