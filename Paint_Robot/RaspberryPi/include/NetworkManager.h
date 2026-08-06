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
     * @brief Sends READY handshake message to server before starting an op.
     * @param op_index The global operation index (0-based) about to be executed.
     * @return true if successfully sent.
     */
    bool SendReady(uint32_t op_index);

    /**
     * @brief Transmits PATH_DONE completion message to vision server when a path is finished.
     * @param phase The path phase that was completed ("approach" or "draw").
     * @return true if successfully sent.
     */
    bool SendPathDone(const std::string& phase);

    /**
     * @brief Thread-safely fetches the latest received ALIGN angle correction matching active op_index.
     */
    bool GetAlignCommand(uint32_t active_op_index, float& out_angle_deg);

    /**
     * @brief Thread-safely fetches the latest received MORE distance correction matching active op_index.
     */
    bool GetMoreCommand(uint32_t active_op_index, float& out_dist_m);

    /**
     * @brief Checks if a GO signal matching active op_index was received and clears it.
     */
    bool CheckAndClearGoSignal(uint32_t active_op_index);

    /**
     * @brief Thread-safely fetches the latest DRIFT angle correction feedback matching active op_index.
     */
    bool GetDriftCorrection(uint32_t active_op_index, float& out_angle_deg);

    /**
     * @brief Thread-safely clears all pending command latches upon receiving a new PATH.
     */
    void ClearLatches();

    /**
     * @brief Checks if HOLD emergency pause from server is currently active.
     */
    bool IsHoldActive();

    /**
     * @brief Gets the phase of the current path ("approach" or "draw").
     */
    std::string GetPathPhase();

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
    std::string current_path_phase;
    bool has_new_path;
    std::atomic<uint32_t> msg_seq;

    std::mutex cmd_mutex;
    std::string latest_cmd;
    bool has_new_cmd;

    // Protocol v2 state variables with op_index transaction locking
    std::mutex align_mutex;
    AlignCmd_t latest_align_cmd;
    bool has_align_cmd{false};

    std::mutex more_mutex;
    MoreCmd_t latest_more_cmd;
    bool has_more_cmd{false};

    std::mutex go_mutex;
    uint32_t go_op_index{0xFFFFFFFF};
    bool has_go_signal{false};

    std::mutex drift_mutex;
    DriftCmd_t latest_drift_cmd;
    bool has_drift_cmd{false};

    std::atomic<bool> is_hold_active{false};

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
