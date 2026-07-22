#ifndef __SERIAL_MANAGER_H__
#define __SERIAL_MANAGER_H__

#include <string>
#include <thread>
#include <mutex>
#include <atomic>
#include "RobotTypes.h"

/**
 * @brief Handles low-level UART serial communications with the STM32 controller.
 */
class SerialManager {
public:
    SerialManager(const std::string& dev = "/dev/serial0", uint32_t baud = 115200);
    ~SerialManager();

    /**
     * @brief Initializes wiringPi, opens the serial port, and spins up the RX thread.
     * @return true if successful, false otherwise.
     */
    bool Init();

    /**
     * @brief Closes the serial port file descriptor and stops the RX thread.
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
    
    /**
     * @brief Sends a command to clear the emergency stop latch.
     * @return true if transmission succeeded.
     */
    bool SendClearEStop();

    /**
     * @brief Thread-safely fetches the latest received status packet.
     * @param out_status Buffer to copy the fetched telemetry metrics.
     * @return true if status was ever received.
     */
    bool GetLatestStatus(Msg_Status_t& out_status);

private:
    int fd;
    std::string device_path;
    uint32_t baudrate;

    // Receive thread parameters
    std::thread rx_thread;
    std::atomic<bool> rx_alive;
    std::mutex status_mutex;
    Msg_Status_t latest_status;
    bool status_received_once;

    // Parser State Machine Variables
    ParserState_t rx_state;
    uint8_t rx_len;
    uint8_t rx_cmd;
    uint8_t rx_payload[16];
    uint8_t rx_payload_idx;
    uint8_t rx_calculated_crc;
    uint8_t rx_received_crc;

    /**
     * @brief Computes a CRC-8 checksum for outgoing/incoming packets.
     */
    uint8_t calculate_crc8(const uint8_t *data, uint8_t len);

    /**
     * @brief Helper function to update incremental CRC8 byte by byte.
     */
    uint8_t crc8_update(uint8_t crc, uint8_t data);

    /**
     * @brief Builds the serial frame and transmits it.
     */
    bool send_packet(uint8_t cmd, const uint8_t *payload, uint8_t payload_len);

    /**
     * @brief Thread execution entry function for background serial reading.
     */
    void rx_loop();

    /**
     * @brief Evaluates a single incoming byte into the parser state machine.
     */
    bool parse_byte(uint8_t byte, Msg_Status_t& out_status);
};

#endif // __SERIAL_MANAGER_H__