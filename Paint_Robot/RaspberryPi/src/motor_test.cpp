#include "SerialManager.h"
#include <iostream>
#include <unistd.h>
#include <csignal>

// Exit flag for clean shutdown
volatile sig_atomic_t keep_running = 1;

void handle_signal(int signal) {
    keep_running = 0;
}

int main() {
    // Register Ctrl+C handler
    std::signal(SIGINT, handle_signal);

    std::cout << "=== STM32 Stepper Motor (DRV8825) Drive Test ===" << std::endl;
    std::cout << "Target: Left Motor (PB0/PB1/PB6) at 300 sps." << std::endl;
    std::cout << "Please ensure common GND, motor external power (12V-24V), and TX/RX line cross-over are all set." << std::endl;
    std::cout << "Press Ctrl+C to stop." << std::endl;
    std::cout << "--------------------------------------------------" << std::endl;

    SerialManager robot_comm("/dev/ttyAMA0", 115200);

    if (!robot_comm.Init()) {
        std::cerr << "[TEST] Error: Failed to open serial port /dev/ttyAMA0." << std::endl;
        return 1;
    }

    std::cout << "[TEST] Serial initialized. Starting 100ms heartbeat loop..." << std::endl;

    while (keep_running) {
        // Send Left: 300 steps/s, Right: 0 steps/s
        robot_comm.SendSetSpeed(300, 0);
        
        // Print feedback to console
        std::cout << "\r[TEST Transmit] Sent SET_SPEED (Left: 300 sps, Right: 0 sps) to STM32..." << std::flush;
        
        usleep(100000); // 100ms interval to bypass STM32's 300ms watchdog timeout
    }

    std::cout << "\n[TEST] Stopping loop. Sending E-STOP to park motors safely..." << std::endl;
    // Send 0 speed targets first
    robot_comm.SendSetSpeed(0, 0);
    usleep(50000);
    // Send formal E-Stop to cut off driver outputs (releases holding torque)
    robot_comm.SendEmergencyStop(0x00);
    
    robot_comm.Close();
    std::cout << "[TEST] Done. Serial port closed. Goodbye!" << std::endl;
    return 0;
}
