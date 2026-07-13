#ifndef __SERIAL_MANAGER_H__
#define __SERIAL_MANAGER_H__

#include <string>
#include "RobotTypes.h"

/**
 * @brief Handles low-level UART serial communications with the STM32 controller.
 */
class SerialManager {
public:
    SerialManager(const std::string& dev = "/dev/ttyAMA0", uint32_t baud = 115200);
    ~SerialManager();

    /**
     * @brief Initializes wiringPi and opens the serial port.
     * @return true if successful, false otherwise.
     */
    bool Init();

    /**
     * @brief Closes the serial port file descriptor.
     */
    void Close();

    /**
     * @brief Sends speed control commands to the STM32.
     * @param left Target steps-per-second for left motor.
     * @param right Target steps-per-second for right motor.
     * @return true if transmission succeeded.
     */
    bool SendSetSpeed(int16_t left, int16_t right);

    /**
     * @brief Controls the spray nozzle state.
     * @param on 1 to turn on the nozzle, 0 to turn off.
     * @return true if transmission succeeded.
     */
    bool SendControlNozzle(uint8_t on);

    /**
     * @brief Sends an emergency stop request with a fault reason.
     * @param reason The safety fault identifier.
     * @return true if transmission succeeded.
     */
    bool SendEmergencyStop(uint8_t reason);

private:
    int fd;
    std::string device_path;
    uint32_t baudrate;

    /**
     * @brief Computes a CRC-8 checksum for outgoing packets.
     */
    uint8_t calculate_crc8(const uint8_t *data, uint8_t len);

    /**
     * @brief Builds the serial frame and transmits it.
     */
    bool send_packet(uint8_t cmd, const uint8_t *payload, uint8_t payload_len);
};

#endif // __SERIAL_MANAGER_H__