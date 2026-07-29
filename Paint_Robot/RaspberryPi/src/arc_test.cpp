#include "SerialManager.h"
#include <iostream>
#include <iomanip>
#include <cmath>
#include <unistd.h>
#include <chrono>

static constexpr float WHEEL_TRACK_M = 0.364f;     // Wheel track base width (364mm)
static constexpr float NOZZLE_OFFSET_M = 0.155f;    // Rear nozzle offset (155mm)
static constexpr float PULSES_PER_METER = 15433.09f;// Encoder pulse scale (pulses/m)

void print_telemetry(SerialManager& comm, const std::string& label) {
    Msg_Status_t status{};
    if (comm.GetLatestStatus(status)) {
        std::cout << "[" << label << "] STM32 telemetry -> L_Steps: " << status.left_steps 
                  << " | R_Steps: " << status.right_steps 
                  << " | Flags: 0x" << std::hex << std::setw(2) << std::setfill('0') << (int)status.flags << std::dec << std::endl;
    }
}

int main(int argc, char** argv) {
    float r_paint = (argc > 1) ? std::stof(argv[1]) : 0.30f;   // Default radius: 0.30m (30cm)
    float angle_deg = (argc > 2) ? std::stof(argv[2]) : 180.0f; // Default angle: 180.0 deg (semi-circle)
    bool is_left = (argc > 3) ? (std::string(argv[3]) == "left") : true;

    std::cout << "==================================================" << std::endl;
    std::cout << "=== Rear Nozzle Offset Arc Motion Test Tool ===" << std::endl;
    std::cout << "==================================================" << std::endl;
    std::cout << "  - Target Paint Radius R_paint: " << r_paint << " m" << std::endl;
    std::cout << "  - Turn Angle: " << angle_deg << " deg (" << (is_left ? "LEFT" : "RIGHT") << ")" << std::endl;
    std::cout << "  - Rear Nozzle Offset d: " << NOZZLE_OFFSET_M << " m (155mm)" << std::endl;

    // Pythagorean corrected robot wheel center radius
    float r_robot = std::sqrt(r_paint * r_paint + NOZZLE_OFFSET_M * NOZZLE_OFFSET_M);
    std::cout << "  -> Calculated Pythagorean Robot Center Radius R_robot: " << r_robot << " m" << std::endl;

    SerialManager robot_comm("/dev/serial0", 115200);
    if (!robot_comm.Init()) {
        std::cerr << "[ARC_TEST] Error: Failed to open serial port /dev/serial0." << std::endl;
        return 1;
    }

    // Clear ESTOP latch
    robot_comm.SendClearEStop();
    usleep(200000);

    // Get initial step encoders
    Msg_Status_t start_status{};
    int retry = 0;
    while (!robot_comm.GetLatestStatus(start_status) && retry++ < 20) {
        usleep(50000);
    }
    int32_t start_l = static_cast<int32_t>(start_status.left_steps);
    int32_t start_r = static_cast<int32_t>(start_status.right_steps);

    std::cout << "\n[STEP 1] +15.5cm Approach Alignment (Nozzle UP)..." << std::endl;
    int32_t align_steps = static_cast<int32_t>(NOZZLE_OFFSET_M * PULSES_PER_METER);
    while (true) {
        Msg_Status_t cur{};
        if (robot_comm.GetLatestStatus(cur)) {
            int32_t dl = static_cast<int32_t>(cur.left_steps) - start_l;
            int32_t dr = static_cast<int32_t>(cur.right_steps) - start_r;
            if (std::abs(dl) >= align_steps || std::abs(dr) >= align_steps) break;
        }
        robot_comm.SendSetSpeed(772, 772); // ~0.05 m/s
        robot_comm.SendControlNozzle(0);
        usleep(80000);
    }
    robot_comm.SendSetSpeed(0, 0);
    print_telemetry(robot_comm, "APPROACH COMPLETE");

    std::cout << "\n[STEP 2] Lowering Nozzle (1.0s delay)..." << std::endl;
    auto wait_start = std::chrono::steady_clock::now();
    while (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - wait_start).count() < 1000) {
        robot_comm.SendSetSpeed(0, 0);
        robot_comm.SendControlNozzle(1);
        usleep(80000);
    }

    std::cout << "\n[STEP 3] Tracing Arc Trajectory (Nozzle DOWN)..." << std::endl;
    float r_left = is_left ? (r_robot - WHEEL_TRACK_M / 2.0f) : (r_robot + WHEEL_TRACK_M / 2.0f);
    float r_right = is_left ? (r_robot + WHEEL_TRACK_M / 2.0f) : (r_robot - WHEEL_TRACK_M / 2.0f);

    float angle_rad = angle_deg * (3.14159265f / 180.0f);
    int32_t target_l_steps = static_cast<int32_t>(std::abs(r_left * angle_rad) * PULSES_PER_METER);
    int32_t target_r_steps = static_cast<int32_t>(std::abs(r_right * angle_rad) * PULSES_PER_METER);

    float base_sps = 771.65f; // 0.05 m/s
    int16_t sps_l = static_cast<int16_t>(base_sps * (r_left / r_robot));
    int16_t sps_r = static_cast<int16_t>(base_sps * (r_right / r_robot));

    std::cout << "  -> Target L_Steps: " << target_l_steps << " | Target R_Steps: " << target_r_steps << std::endl;
    std::cout << "  -> Calculated Wheel Speeds: SPS_L = " << sps_l << " | SPS_R = " << sps_r << std::endl;

    Msg_Status_t arc_start_status{};
    robot_comm.GetLatestStatus(arc_start_status);
    int32_t arc_start_l = static_cast<int32_t>(arc_start_status.left_steps);
    int32_t arc_start_r = static_cast<int32_t>(arc_start_status.right_steps);

    while (true) {
        Msg_Status_t cur{};
        if (robot_comm.GetLatestStatus(cur)) {
            int32_t dl = std::abs(static_cast<int32_t>(cur.left_steps) - arc_start_l);
            int32_t dr = std::abs(static_cast<int32_t>(cur.right_steps) - arc_start_r);
            if (dl >= target_l_steps || dr >= target_r_steps) break;
        }
        robot_comm.SendSetSpeed(sps_l, sps_r);
        robot_comm.SendControlNozzle(1);
        usleep(80000);
    }
    robot_comm.SendSetSpeed(0, 0);
    print_telemetry(robot_comm, "ARC TRACING COMPLETE");

    std::cout << "\n[STEP 4] Raising Nozzle (1.0s delay)..." << std::endl;
    wait_start = std::chrono::steady_clock::now();
    while (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - wait_start).count() < 1000) {
        robot_comm.SendSetSpeed(0, 0);
        robot_comm.SendControlNozzle(0);
        usleep(80000);
    }

    std::cout << "\n[STEP 5] -15.5cm Corner Reverse Alignment (Nozzle UP)..." << std::endl;
    Msg_Status_t rev_start_status{};
    robot_comm.GetLatestStatus(rev_start_status);
    int32_t rev_start_l = static_cast<int32_t>(rev_start_status.left_steps);
    int32_t rev_start_r = static_cast<int32_t>(rev_start_status.right_steps);

    while (true) {
        Msg_Status_t cur{};
        if (robot_comm.GetLatestStatus(cur)) {
            int32_t dl = static_cast<int32_t>(cur.left_steps) - rev_start_l;
            int32_t dr = static_cast<int32_t>(cur.right_steps) - rev_start_r;
            if (std::abs(dl) >= align_steps || std::abs(dr) >= align_steps) break;
        }
        robot_comm.SendSetSpeed(-772, -772); // -0.05 m/s
        robot_comm.SendControlNozzle(0);
        usleep(80000);
    }
    robot_comm.SendSetSpeed(0, 0);
    print_telemetry(robot_comm, "ARC TEST FINISHED");

    std::cout << "\n=== Arc Motion Test Successfully Completed ===" << std::endl;
    robot_comm.Close();
    return 0;
}
