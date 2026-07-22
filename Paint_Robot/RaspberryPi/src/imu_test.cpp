#include <iostream>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <unistd.h>
#include <cstdio>
#include <chrono>
#include <cmath>

#define MPU6050_ADDR         0x68
#define REG_CONFIG           0x1A
#define REG_GYRO_CONFIG      0x1B
#define REG_PWR_MGMT_1       0x6B
#define REG_WHO_AM_I         0x75
#define REG_GYRO_XOUT_H      0x43
#define REG_GYRO_YOUT_H      0x45
#define REG_GYRO_ZOUT_H      0x47

// Global Yaw variable to allow external resets from server correction triggers
float yaw = 0.0f;

// External Reset Sequence Stub (Call this when Vision Server POS message arrives)
void reset_yaw_heading(float new_yaw) {
    yaw = new_yaw;
    std::cout << "\n[IMU System] External reset applied! Yaw synced to vision reference: " << new_yaw << " deg" << std::endl;
}

// 16-bit register reader with I2C bus error verification
short read_raw_data(int fd, int addr, bool& error) {
    int high = wiringPiI2CReadReg8(fd, addr);
    int low = wiringPiI2CReadReg8(fd, addr + 1);
    if (high < 0 || low < 0) {
        error = true;
        return 0;
    }
    error = false;
    return (high << 8) | low;
}

int main() {
    std::cout << "[IMU Configured Safety Tracker] Initializing wiringPi I2C..." << std::endl;
    
    int fd = wiringPiI2CSetup(MPU6050_ADDR);
    if (fd < 0) {
        std::cerr << "[IMU Test] Error: Failed to init I2C at address 0x68." << std::endl;
        return 1;
    }
    
    // 1. Wake up MPU-6050 (clear sleep mode)
    wiringPiI2CWriteReg8(fd, REG_PWR_MGMT_1, 0x00);
    usleep(50000);

    // 2. Explicitly Configure Full Scale Range & Resolution (GYRO_CONFIG 0x1B)
    // Write 0x00 to select FS_SEL = 0 (Full Scale Range: ±250 deg/s, Sensitivity: 131.0 LSB/deg/s)
    // This strictly forces the hardware resolution to match our code scaling (131.0f).
    wiringPiI2CWriteReg8(fd, REG_GYRO_CONFIG, 0x00);
    
    // 3. Configure Hardware Digital Low Pass Filter (CONFIG 0x1A)
    // Write 0x03 to enable DLPF with a ~42Hz bandwidth. 
    // This cuts off high-frequency motor vibrations and chassis resonance at the hardware level!
    wiringPiI2CWriteReg8(fd, REG_CONFIG, 0x03);
    
    usleep(100000); // Wait for sensor registers to settle

    // 4. Multi-axis Calibration
    std::cout << "[IMU Test] Calibrating Gyro with explicit registers. KEEP SENSOR STILL..." << std::endl;
    double offset_x_sum = 0, offset_y_sum = 0, offset_z_sum = 0;
    int valid_samples = 0;
    bool err = false;
    
    while (valid_samples < 500) {
        short rx = read_raw_data(fd, REG_GYRO_XOUT_H, err);
        short ry = read_raw_data(fd, REG_GYRO_YOUT_H, err);
        short rz = read_raw_data(fd, REG_GYRO_ZOUT_H, err);
        
        if (!err) {
            offset_x_sum += rx;
            offset_y_sum += ry;
            offset_z_sum += rz;
            valid_samples++;
        }
        usleep(5000);
    }
    
    double gyro_x_offset = offset_x_sum / valid_samples;
    double gyro_y_offset = offset_y_sum / valid_samples;
    double gyro_z_offset = offset_z_sum / valid_samples;
    
    std::cout << "[IMU Test] Calibration complete." << std::endl;
    std::cout << "  - Gyro Z Offset: " << gyro_z_offset << " (Configured FSR: ±250 deg/s)" << std::endl;

    std::cout << "\nExplicit FSR (±250 dps) and Hardware DLPF (~42Hz) active." << std::endl;
    std::cout << "Press Ctrl+C to exit." << std::endl;
    std::cout << "------------------------------------------------------------------" << std::endl;

    float prev_gyro_z = 0.0f;

    // --- Safety & Noise Filters Settings ---
    const float GYRO_DEADBAND = 0.15f; 
    const float MAX_PHYSICAL_RATE = 120.0f; 
    const float MAX_SLEW_RATE = 80.0f;     
    const float IMU_ANOMALY_LIMIT = 200.0f; 

    auto last_time = std::chrono::high_resolution_clock::now();
    int loop_counter = 0;

    while (true) {
        auto current_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<float> elapsed = current_time - last_time;
        float dt = elapsed.count();
        last_time = current_time;

        short raw_gx = read_raw_data(fd, REG_GYRO_XOUT_H, err);
        short raw_gy = read_raw_data(fd, REG_GYRO_YOUT_H, err);
        short raw_gz = read_raw_data(fd, REG_GYRO_ZOUT_H, err);
        
        float gyro_x_deg_s = 0.0f;
        float gyro_y_deg_s = 0.0f;
        float gyro_z_deg_s = 0.0f;

        if (err) {
            gyro_z_deg_s = prev_gyro_z;
        } else {
            gyro_x_deg_s = (static_cast<float>(raw_gx) - gyro_x_offset) / 131.0f;
            gyro_y_deg_s = (static_cast<float>(raw_gy) - gyro_y_offset) / 131.0f;
            gyro_z_deg_s = (static_cast<float>(raw_gz) - gyro_z_offset) / 131.0f;
        }

        if (std::abs(gyro_x_deg_s) > IMU_ANOMALY_LIMIT || std::abs(gyro_y_deg_s) > IMU_ANOMALY_LIMIT) {
            gyro_z_deg_s = prev_gyro_z;
            std::cout << "\n[IMU ANOMALY] Roll/Pitch spike - Skipping frame." << std::endl;
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
        yaw += gyro_z_deg_s * dt;

        loop_counter++;
        if (loop_counter >= 500) {
            reset_yaw_heading(0.0f); 
            loop_counter = 0;
        }

        printf("\r[IMU Robust] X-Rate: %6.1f | Y-Rate: %6.1f | Yaw Heading: %7.2f deg", 
               gyro_x_deg_s, gyro_y_deg_s, yaw);
        fflush(stdout);

        usleep(20000); 
    }

    return 0;
}
