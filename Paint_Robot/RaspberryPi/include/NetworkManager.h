#ifndef __NETWORK_MANAGER_H__
#define __NETWORK_MANAGER_H__

#include <string>
#include <vector>
#include <mutex>
#include <thread>
#include <atomic>
#include <openssl/ssl.h>
#include <openssl/err.h>
#include "RobotTypes.h"

#define DEFAULT_SERVER_IP   "192.168.0.8"
#define DEFAULT_SERVER_PORT 9000

/**
 * @brief Manages TCP socket connection to the Vision Server and handles packets.
 */
class NetworkManager {
public:
    NetworkManager(const std::string& ip = DEFAULT_SERVER_IP, uint16_t port = DEFAULT_SERVER_PORT);
    ~NetworkManager();

    /**
     * @brief Establishes a socket connection to the server.
     * @return true if successful, false otherwise.
     */
    bool Init();

    /**
     * @brief Disconnects the socket.
     */
    void Close();

    /**
     * @brief Serves as a placeholder for low-level connection state checks.
     */
    void Process();

    /**
     * @brief Sends status packets back to the Vision Server.
     * @param status Struct containing steps count and flag state.
     * @return true if successfully sent.
     */
    bool SendStatus(const Msg_Status_t& status);

    /**
     * @brief Retrieves the latest pose data received from the server.
     * @param out_pose Reference to store the retrieved pose.
     * @return true if a valid pose exists.
     */
    bool GetLatestPose(Pose_t& out_pose);

    /**
     * @brief Retrieves the path data received from the server.
     * @param out_path Vector to populate with segments.
     * @return true if a path is loaded.
     */
    bool GetPath(std::vector<Segment_t>& out_path);

    /**
     * @brief Thread-safely fetches the latest received command from the server.
     * @param out_cmd Reference to store the retrieved command.
     * @return true if a new command is available.
     */
    bool GetLatestCommand(std::string& out_cmd);

private:
    std::string server_ip;
    uint16_t server_port;
    int client_fd;
    bool is_connected;

    // OpenSSL variables
    SSL_CTX* ssl_ctx;
    SSL* ssl_connection;
    std::mutex write_mutex;

    // Background thread configuration
    std::thread rx_thread;
    std::atomic<bool> rx_alive;

    // Mutex protectors for shared data
    std::mutex pose_mutex;
    std::mutex path_mutex;

    Pose_t latest_pose;
    bool has_new_pose;

    std::vector<Segment_t> current_path;
    bool has_new_path;
    std::atomic<uint32_t> msg_seq;

    std::mutex cmd_mutex;
    std::string latest_cmd;
    bool has_new_cmd;

    /**
     * @brief Background worker loop to read incoming data from socket.
     */
    void rx_loop();

    /**
     * @brief Reads a full line (terminated by '\n') from the SSL stream.
     */
    bool ssl_read_line(std::string& buf, std::string& line);

    /**
     * @brief Writes a raw JSON message over the SSL stream.
     */
    bool ssl_send_line(const std::string& raw_json_message);

    /**
     * @brief Internal helper to parse raw incoming buffers.
     */
    void parse_incoming_data(const std::string& line);
};

#endif /* __NETWORK_MANAGER_H__ */
