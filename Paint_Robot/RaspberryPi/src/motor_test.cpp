#include "SerialManager.h"
#include <iostream>
#include <iomanip>
#include <unistd.h>
#include <chrono>

void print_status(SerialManager& comm, const std::string& step_label) {
    Msg_Status_t status{};
    if (comm.GetLatestStatus(status)) {
        std::cout << "[" << step_label << " Feedback] STM32 STATUS -> L_Steps: " << status.left_steps 
                  << " | R_Steps: " << status.right_steps 
                  << " | Flags: 0x" << std::hex << std::setw(2) << std::setfill('0') << (int)status.flags << std::dec;
        std::cout << " (";
        if (status.flags & 0x01) std::cout << "MOVING ";
        if (status.flags & 0x02) std::cout << "ESTOP ";
        if (status.flags & 0x04) std::cout << "TIMEOUT ";
        if (status.flags & 0x08) std::cout << "NOZZLE_ON ";
        if (status.flags & 0x10) std::cout << "RX_ERR ";
        if (status.flags & 0x20) std::cout << "TX_OVERFLOW ";
        if (status.flags == 0x00) std::cout << "IDLE";
        std::cout << ")" << std::endl;
    } else {
        std::cout << "[" << step_label << " Feedback] Waiting for STM32 0x81 status packet..." << std::endl;
    }
}

int main() {
    std::cout << "==================================================" << std::endl;
    std::cout << "=== STM32 <-> RPi UART All-Packet Test Suite ===" << std::endl;
    std::cout << "==================================================" << std::endl;

    SerialManager robot_comm("/dev/serial0", 115200);

    if (!robot_comm.Init()) {
        std::cerr << "[TEST] Error: Failed to open serial port /dev/serial0." << std::endl;
        return 1;
    }

    std::cout << "\n[TEST INIT] Sending 0x04 CLEAR_ESTOP (Key: 0xA55A) to un-latch startup ESTOP..." << std::endl;
    robot_comm.SendClearEStop();
    usleep(200000);
    print_status(robot_comm, "INIT CLEAR_ESTOP");

    std::cout << "\n[TEST STEP 1] Sending 0x01 SET_SPEED (L: 500 sps, R: 500 sps) for 2 seconds..." << std::endl;
    for (int i = 0; i < 25; i++) { // 25 * 80ms = 2.0 seconds
        robot_comm.SendSetSpeed(500, 500);
        usleep(80000);
        if (i % 5 == 0) print_status(robot_comm, "STEP 1 SPEED");
    }

    std::cout << "\n[TEST STEP 2] Sending 0x02 NOZZLE ON (1)..." << std::endl;
    robot_comm.SendControlNozzle(1);
    for (int i = 0; i < 15; i++) { // 1.2s
        robot_comm.SendSetSpeed(500, 500);
        robot_comm.SendControlNozzle(1);
        usleep(80000);
        if (i % 5 == 0) print_status(robot_comm, "STEP 2 NOZZLE_ON");
    }

    std::cout << "\n[TEST STEP 3] Sending 0x02 NOZZLE OFF (0)..." << std::endl;
    robot_comm.SendControlNozzle(0);
    usleep(100000);
    print_status(robot_comm, "STEP 3 NOZZLE_OFF");

    std::cout << "\n[TEST STEP 4] Sending 0x01 SET_SPEED (0, 0) to stop motors and disable watchdog..." << std::endl;
    robot_comm.SendSetSpeed(0, 0);
    robot_comm.SendControlNozzle(0);
    usleep(300000);
    print_status(robot_comm, "STEP 4 STOP");

    std::cout << "\n==================================================" << std::endl;
    std::cout << "=== All UART Packets Transmitted Successfully! ===" << std::endl;
    std::cout << "==================================================" << std::endl;

    robot_comm.Close();
    return 0;
}
