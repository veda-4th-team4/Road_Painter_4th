#include "ImuManager.h"
#include "RobotTypes.h"
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <iostream>
#include <unistd.h>
#include <cmath>
#include <chrono>

#define MPU6050_ADDR         0x68
#define REG_CONFIG           0x1A
#define REG_GYRO_CONFIG      0x1B
#define REG_PWR_MGMT_1       0x6B
#define REG_GYRO_XOUT_H      0x43
#define REG_GYRO_YOUT_H      0x45
#define REG_GYRO_ZOUT_H      0x47

ImuManager::ImuManager(uint8_t i2c_addr)
    : i2c_address(i2c_addr),
      fd(-1),
      is_initialized(false),
      rx_alive(false),
      trigger_calibration(false),
      current_yaw(0.0f),
      gyro_z_offset(0.0)
{
}

ImuManager::~ImuManager() {
    Close();
}

short ImuManager::read_raw_register(int addr, bool& error) {
    if (fd < 0) {
        error = true;
        return 0;
    }
    int high = wiringPiI2CReadReg8(fd, addr);
    int low = wiringPiI2CReadReg8(fd, addr + 1);
    if (high < 0 || low < 0) {
        error = true;
        return 0;
    }
    error = false;
    return static_cast<short>((high << 8) | low);
}

bool ImuManager::Init() {
    fd = wiringPiI2CSetup(i2c_address);
    if (fd < 0) {
        std::cerr << "[ImuManager] Warning: Failed to init I2C at address 0x" 
                  << std::hex << (int)i2c_address << std::dec 
                  << ". Running in Odometry fallback mode." << std::endl;
        return false;
    }

    // 1. Wake up MPU-6050
    wiringPiI2CWriteReg8(fd, REG_PWR_MGMT_1, 0x00);
    usleep(50000);

    // 2. Configure Full Scale Range: ±250 deg/s (131.0 LSB/deg/s)
    wiringPiI2CWriteReg8(fd, REG_GYRO_CONFIG, 0x00);

    // 3. Configure Hardware DLPF: ~42Hz bandwidth
    wiringPiI2CWriteReg8(fd, REG_CONFIG, 0x03);
    usleep(100000);

    // 4. Gyro Calibration (500 samples)
    std::cout << "[ImuManager] Calibrating Gyro Z offset... Keep robot still!" << std::endl;
    double offset_z_sum = 0;
    int valid_samples = 0;
    bool err = false;

    for (int i = 0; i < 500; i++) {
        short rz = read_raw_register(REG_GYRO_ZOUT_H, err);
        if (!err) {
            offset_z_sum += rz;
            valid_samples++;
        }
        usleep(5000);
    }

    if (valid_samples == 0) {
        std::cerr << "[ImuManager] Warning: Gyro calibration failed (no valid I2C samples)." << std::endl;
        return false;
    }

    gyro_z_offset = offset_z_sum / valid_samples;
    std::cout << "[ImuManager] Calibration complete! (" << valid_samples << " samples)" << std::endl;
    std::cout << "  - Gyro Z Offset: " << gyro_z_offset << " LSB (~" 
              << (gyro_z_offset / 131.0) << " deg/s bias removed)" << std::endl;

    is_initialized = true;
    rx_alive = true;
    worker_thread = std::thread(&ImuManager::imu_loop, this);
    return true;
}

void ImuManager::Close() {
    rx_alive = false;
    if (worker_thread.joinable()) {
        worker_thread.join();
    }
    fd = -1;
    is_initialized = false;
}

void ImuManager::ResetYaw(float new_yaw) {
    std::lock_guard<std::mutex> lock(yaw_mutex);
    current_yaw = new_yaw;
    std::cout << "[ImuManager] Yaw heading reset to: " << new_yaw << " deg" << std::endl;
}

void ImuManager::Calibrate() {
    trigger_calibration = true;
    std::cout << "[ImuManager] Recalibration requested..." << std::endl;
}

float ImuManager::GetYaw() {
    std::lock_guard<std::mutex> lock(yaw_mutex);
    return current_yaw;
}

void ImuManager::imu_loop() {
    const float GYRO_DEADBAND = 0.30f; 
    const float MAX_PHYSICAL_RATE = 120.0f; 
    const float MAX_SLEW_RATE = 80.0f;     
    const float IMU_ANOMALY_LIMIT = 200.0f; 

    float prev_gyro_z = 0.0f;
    bool err = false;
    auto last_time = std::chrono::high_resolution_clock::now();

    while (rx_alive) {
        if (trigger_calibration) {
            std::cout << "[ImuManager] Recalibrating Gyro Z offset dynamically... Keep robot still!" << std::endl;
            double offset_z_sum = 0;
            int valid_samples = 0;
            bool err = false;
            for (int i = 0; i < 500; i++) {
                short rz = read_raw_register(REG_GYRO_ZOUT_H, err);
                if (!err) {
                    offset_z_sum += rz;
                    valid_samples++;
                }
                usleep(5000);
            }
            if (valid_samples > 0) {
                gyro_z_offset = offset_z_sum / valid_samples;
                std::cout << "[ImuManager] Dynamic Calibration complete! Gyro Z Offset: " << gyro_z_offset << std::endl;
                // Note: We deliberately DO NOT reset current_yaw to 0.0f here so the global heading tracking is preserved!
            }
            trigger_calibration = false;
            last_time = std::chrono::high_resolution_clock::now();
            continue;
        }

        auto current_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<float> elapsed = current_time - last_time;
        float dt = elapsed.count();
        last_time = current_time;

        short raw_gx = read_raw_register(REG_GYRO_XOUT_H, err);
        short raw_gy = read_raw_register(REG_GYRO_YOUT_H, err);
        short raw_gz = read_raw_register(REG_GYRO_ZOUT_H, err);

        float gyro_x_deg_s = 0.0f;
        float gyro_y_deg_s = 0.0f;
        float gyro_z_deg_s = 0.0f;

        if (err) {
            gyro_z_deg_s = prev_gyro_z;
        } else {
            gyro_x_deg_s = (static_cast<float>(raw_gx)) / 131.0f;
            gyro_y_deg_s = (static_cast<float>(raw_gy)) / 131.0f;
            gyro_z_deg_s = (static_cast<float>(raw_gz) - gyro_z_offset) / 131.0f;
        }

        if (std::abs(gyro_x_deg_s) > IMU_ANOMALY_LIMIT || std::abs(gyro_y_deg_s) > IMU_ANOMALY_LIMIT) {
            gyro_z_deg_s = prev_gyro_z;
        } else {
            if (std::abs(gyro_z_deg_s - prev_gyro_z) > MAX_SLEW_RATE) {
                gyro_z_deg_s = prev_gyro_z;
            }
            if (gyro_z_deg_s > MAX_PHYSICAL_RATE) gyro_z_deg_s = MAX_PHYSICAL_RATE;
            else if (gyro_z_deg_s < -MAX_PHYSICAL_RATE) gyro_z_deg_s = -MAX_PHYSICAL_RATE;

            if (std::abs(gyro_z_deg_s) < GYRO_DEADBAND) {
                gyro_z_deg_s = 0.0f;
            }
        }

        prev_gyro_z = gyro_z_deg_s;

        {
            std::lock_guard<std::mutex> lock(yaw_mutex);
            current_yaw += gyro_z_deg_s * dt;
        }

        // Continuous logging disabled to keep stdout clean.
        // Uncomment below for low-frequency debug logging if needed:
        // static int log_counter = 0;
        // if (++log_counter >= 50) { // 1Hz logging
        //     log_counter = 0;
        //     std::cout << GetTimestampStr() << "[IMU 1Hz] Yaw: " << current_yaw << " deg | Gyro Z-Rate: " << gyro_z_deg_s << " deg/s" << std::endl;
        // }

        usleep(20000); // 20ms update interval (~50Hz)
    }
}
