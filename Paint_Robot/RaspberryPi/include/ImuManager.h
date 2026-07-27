#ifndef __IMU_MANAGER_H__
#define __IMU_MANAGER_H__

#include <thread>
#include <mutex>
#include <atomic>
#include <stdint.h>

/**
 * @brief Thread-safe MPU6050 I2C IMU Driver & Yaw Heading Estimator.
 */
class ImuManager {
public:
    ImuManager(uint8_t i2c_addr = 0x68);
    ~ImuManager();

    /**
     * @brief Initializes I2C, configures DLPF/FSR, and runs gyro calibration.
     * @return true if IMU hardware responded and initialized successfully.
     */
    bool Init();

    /**
     * @brief Stops worker thread and releases I2C resources.
     */
    void Close();

    /**
     * @brief Resets current Yaw heading angle to a target reference angle (default 0.0 deg).
     */
    void ResetYaw(float new_yaw = 0.0f);

    /**
     * @brief Gets current integrated Yaw heading angle in degrees.
     */
    float GetYaw();

    /**
     * @brief Checks if IMU I2C communication is active and healthy.
     */
    bool IsHealthy() const { return is_initialized && rx_alive; }

private:
    uint8_t i2c_address;
    int fd;
    std::atomic<bool> is_initialized;
    std::atomic<bool> rx_alive;
    std::thread worker_thread;
    std::mutex yaw_mutex;

    float current_yaw;
    double gyro_z_offset;

    short read_raw_register(int addr, bool& error);
    void imu_loop();
};

#endif /* __IMU_MANAGER_H__ */
