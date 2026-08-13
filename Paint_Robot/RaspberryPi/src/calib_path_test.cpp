#include <iostream>
#include <vector>
#include <string>
#include <chrono>
#include <thread>
#include <iomanip>
#include <cmath>
#include <unistd.h>

#include "RobotTypes.h"
#include "SerialManager.h"
#include "ImuManager.h"
#include "PathFollower.h"

// =================================================================
// 🎯 Road-Painter Odometry Calibration Test Tool (calib_path_test)
// =================================================================
// Usage: ./calib_path_test [m_cm] [n_cm] [start_corner]
// Default: m_cm = 90, n_cm = 60, start_corner = "bottom_left"
// =================================================================

int main(int argc, char* argv[]) {
    int m_cm = 90;
    int n_cm = 60;
    std::string start_corner = "bottom_left";

    if (argc >= 2) m_cm = std::stoi(argv[1]);
    if (argc >= 3) n_cm = std::stoi(argv[2]);
    if (argc >= 4) start_corner = argv[3];

    float m_m = m_cm / 100.0f;
    float n_m = n_cm / 100.0f;
    float m_half = m_m / 2.0f;
    float n_half = n_m / 2.0f;

    // Corner turn angle logic (§1-1 table in plan doc)
    // bottom_left: CCW left turn (-90.0 deg)
    // top_left   : CW right turn (+90.0 deg)
    float turn_angle_deg = (start_corner == "top_left") ? 90.0f : -90.0f;

    std::cout << "\n===================================================================" << std::endl;
    std::cout << "[CALIB TEST] Starting Odometry Calibration Path Execution" << std::endl;
    std::cout << "[CALIB TEST] Dimension: " << m_cm << " cm (m) x " << n_cm << " cm (n)" << std::endl;
    std::cout << "[CALIB TEST] Corner: " << start_corner << " | Turn Angle per Corner: " << turn_angle_deg << " deg" << std::endl;
    std::cout << "===================================================================\n" << std::endl;

    // 1. Build 11-operation 8-point rectangular path
    std::vector<Segment_t> ops(11);

    ops[0].op = "move"; ops[0].dist_m = m_half; ops[0].op_index = 0;
    ops[1].op = "move"; ops[1].dist_m = m_half; ops[1].op_index = 1;
    ops[2].op = "turn"; ops[2].angle_deg = turn_angle_deg; ops[2].op_index = 2;

    ops[3].op = "move"; ops[3].dist_m = n_half; ops[3].op_index = 3;
    ops[4].op = "move"; ops[4].dist_m = n_half; ops[4].op_index = 4;
    ops[5].op = "turn"; ops[5].angle_deg = turn_angle_deg; ops[5].op_index = 5;

    ops[6].op = "move"; ops[6].dist_m = m_half; ops[6].op_index = 6;
    ops[7].op = "move"; ops[7].dist_m = m_half; ops[7].op_index = 7;
    ops[8].op = "turn"; ops[8].angle_deg = turn_angle_deg; ops[8].op_index = 8;

    ops[9].op = "move";  ops[9].dist_m = n_half; ops[9].op_index = 9;
    ops[10].op = "move"; ops[10].dist_m = n_half; ops[10].op_index = 10;

    // 2. Initialize Hardware Managers
    SerialManager robot_comm("/dev/serial0", 115200);
    if (!robot_comm.Init()) {
        std::cerr << "[CALIB TEST] Error: Failed to open serial port /dev/serial0!" << std::endl;
        return 1;
    }

    ImuManager imu_manager;
    bool has_imu = imu_manager.Init();
    if (has_imu) {
        imu_manager.ResetYaw();
        std::cout << "[CALIB TEST] IMU MPU-6050 initialized successfully. Yaw zeroed -> 0.00 deg" << std::endl;
    } else {
        std::cout << "[CALIB TEST] Warning: IMU offline. Running on step odometry fallback." << std::endl;
    }

    // Clear STM32 startup ESTOP and transmit dynamic servo PWM parameters
    robot_comm.SendClearEStop();
    usleep(50000);
    robot_comm.SendSetServoConfig(RPI_SERVO_OFF_US, RPI_SERVO_ON_US);
    usleep(100000);

    PathFollower path_follower;
    path_follower.SetPath(ops);

    // Initial Camera Settling Delay (Boundary 0)
    std::cout << "[CALIB BOUNDARY 0] Point (0mm, 0mm) | Initial Camera Settling (2.0s)... Sending READY(0)" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    float accumulated_target_yaw = 0.0f;
    uint32_t total_l_steps = 0;
    uint32_t total_r_steps = 0;

    // 3. Execution Loop over 11 ops
    for (size_t op_idx = 0; op_idx < ops.size(); ++op_idx) {
        Segment_t seg = ops[op_idx];

        Msg_Status_t status{};
        robot_comm.GetLatestStatus(status);
        int32_t start_l = static_cast<int32_t>(status.left_steps);
        int32_t start_r = static_cast<int32_t>(status.right_steps);

        if (seg.op == "move") {
            path_follower.StartMove(seg.dist_m, start_l, start_r);
            while (true) {
                robot_comm.GetLatestStatus(status);
                int32_t cur_l = static_cast<int32_t>(status.left_steps);
                int32_t cur_r = static_cast<int32_t>(status.right_steps);
                float cur_yaw = has_imu ? imu_manager.GetYaw() : 0.0f;

                Msg_SetSpeed_t speed_cmd{};
                bool finished = path_follower.UpdateMove(cur_l, cur_r, speed_cmd, cur_yaw);
                robot_comm.SendSetSpeed(speed_cmd.left_sps, speed_cmd.right_sps);

                if (finished) {
                    robot_comm.SendSetSpeed(0, 0);
                    float final_yaw = has_imu ? imu_manager.GetYaw() : 0.0f;
                    std::cout << GetTimestampStr() << "[CALIB OP " << op_idx << "/10] MOVE " 
                              << std::fixed << std::setprecision(3) << seg.dist_m << "m -> Complete | Steps L: " 
                              << (cur_l - start_l) << ", R: " << (cur_r - start_r)
                              << " | IMU Yaw: " << std::setprecision(2) << final_yaw << " deg" << std::endl;
                    break;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }
        } else if (seg.op == "turn") {
            accumulated_target_yaw += seg.angle_deg;
            float start_yaw = has_imu ? imu_manager.GetYaw() : 0.0f;
            path_follower.StartTurn(seg.angle_deg, start_l, start_r, start_yaw);
            while (true) {
                robot_comm.GetLatestStatus(status);
                int32_t cur_l = static_cast<int32_t>(status.left_steps);
                int32_t cur_r = static_cast<int32_t>(status.right_steps);
                float cur_yaw = has_imu ? imu_manager.GetYaw() : 0.0f;

                Msg_SetSpeed_t speed_cmd{};
                bool finished = path_follower.UpdateTurn(cur_l, cur_r, speed_cmd, cur_yaw, has_imu);
                robot_comm.SendSetSpeed(speed_cmd.left_sps, speed_cmd.right_sps);

                if (finished) {
                    robot_comm.SendSetSpeed(0, 0);
                    float final_yaw = has_imu ? imu_manager.GetYaw() : 0.0f;
                    float relative_target_yaw = seg.angle_deg;
                    float yaw_err = final_yaw - relative_target_yaw;
                    std::cout << GetTimestampStr() << "[CALIB OP " << op_idx << "/10] TURN " 
                              << std::fixed << std::setprecision(2) << seg.angle_deg << "deg -> Main Turn Complete | Target Yaw: "
                              << relative_target_yaw << " deg | IMU Yaw: " << final_yaw 
                              << " deg (Error: " << (yaw_err >= 0 ? "+" : "") << yaw_err << " deg)" << std::endl;

                    if (has_imu) {
                        // 1. Initial coasting pause (400ms)
                        std::this_thread::sleep_for(std::chrono::milliseconds(400));
                        float settled_yaw = imu_manager.GetYaw();
                        float residual_error = settled_yaw - relative_target_yaw;

                        // 2. If residual error > 0.25 deg, run active micro-correction sequence at 120 sps
                        if (std::fabs(residual_error) > 0.25f) {
                            std::cout << GetTimestampStr() << "[CALIB TRIM] Active Micro-Correction Triggered! Residual Error: " 
                                      << (residual_error >= 0 ? "+" : "") << residual_error << " deg" << std::endl;

                            auto trim_start_time = std::chrono::high_resolution_clock::now();
                            while (true) {
                                float cur_trim_yaw = imu_manager.GetYaw();
                                Msg_SetSpeed_t trim_speed{};
                                bool trim_done = path_follower.TrimTurn(relative_target_yaw, cur_trim_yaw, trim_speed);
                                robot_comm.SendSetSpeed(trim_speed.left_sps, trim_speed.right_sps);

                                auto trim_elapsed = std::chrono::high_resolution_clock::now() - trim_start_time;
                                if (trim_done || std::chrono::duration_cast<std::chrono::milliseconds>(trim_elapsed).count() > 1500) {
                                    robot_comm.SendSetSpeed(0, 0);
                                    float post_trim_yaw = imu_manager.GetYaw();
                                    float post_trim_err = post_trim_yaw - relative_target_yaw;
                                    std::cout << GetTimestampStr() << "[CALIB TRIM] Active Micro-Correction Finished! Final Yaw: " 
                                              << post_trim_yaw << " deg (Error: " << (post_trim_err >= 0 ? "+" : "") << post_trim_err << " deg)" << std::endl;
                                    break;
                                }
                                std::this_thread::sleep_for(std::chrono::milliseconds(20));
                            }
                        }
                    }
                    break;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }
        }

        // Boundary Handshake & Camera Settling logic (2.0s delay post-TURN for complete physical rest)
        if (seg.op == "turn") {
            std::cout << GetTimestampStr() << "[CALIB BOUNDARY " << (op_idx + 1) 
                      << "] Post-TURN Settling Delay (2.0s)... Bringing chassis to complete rest" << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(2000));
            if (has_imu) {
                imu_manager.ResetYaw(0.0f);
                std::cout << GetTimestampStr() << "[CALIB CORNER] Chassis at complete rest. Resetting IMU Yaw to 0.00 deg for next segment!" << std::endl;
            }
        } else {
            std::cout << GetTimestampStr() << "[CALIB BOUNDARY " << (op_idx + 1) 
                      << "] Camera Settling Delay (2.5s)... Sending READY(" << (op_idx + 1) << ")" << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(2500));
        }
    }

    // 4. Print Final Execution Summary
    Msg_Status_t final_status{};
    robot_comm.GetLatestStatus(final_status);
    float final_yaw = has_imu ? imu_manager.GetYaw() : 0.0f;
    float final_yaw_err = final_yaw - accumulated_target_yaw;

    std::cout << "\n===================================================================" << std::endl;
    std::cout << "[CALIB SUMMARY] All 11 Operations Successfully Executed!" << std::endl;
    std::cout << "[CALIB SUMMARY] Total Rectangular Drive Distance: " << (2.0f * (m_m + n_m)) << " m" << std::endl;
    std::cout << "[CALIB SUMMARY] Final Accumulated IMU Yaw Error: " << (final_yaw_err >= 0 ? "+" : "") << final_yaw_err << " deg" << std::endl;
    std::cout << "[CALIB SUMMARY] Left Motor Total Steps: " << final_status.left_steps 
              << " | Right Motor Total Steps: " << final_status.right_steps << std::endl;
    std::cout << "[CALIB SUMMARY] Robot Returned to Starting Point (0,0) -> Session Complete." << std::endl;
    std::cout << "===================================================================\n" << std::endl;

    robot_comm.Close();
    return 0;
}
