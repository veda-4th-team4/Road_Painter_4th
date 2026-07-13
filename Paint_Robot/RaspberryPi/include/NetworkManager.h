#ifndef __NETWORK_MANAGER_H__
#define __NETWORK_MANAGER_H__

#include <string>
#include <vector>
#include <mutex>
#include "RobotTypes.h"

/**
 * @brief Manages TCP socket connection to the Vision Server and handles packets.
 */
class NetworkManager {
public:
    NetworkManager(const std::string& ip, uint16_t port);
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
     * @brief Non-blocking worker function to receive and parse incoming server packets.
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
     * @param out_path Vector to populate with waypoints.
     * @return true if a path is loaded.
     */
    bool GetPath(std::vector<Waypoint_t>& out_path);

private:
    std::string server_ip;
    uint16_t server_port;
    int client_fd;
    bool is_connected;

    // Mutex protectors for shared data
    std::mutex pose_mutex;
    std::mutex path_mutex;

    Pose_t latest_pose;
    bool has_new_pose;

    std::vector<Waypoint_t> current_path;
    bool has_new_path;

    /**
     * @brief Internal helper to parse raw incoming buffers.
     */
    void parse_incoming_data(const uint8_t* buffer, size_t size);
};

#endif /* __NETWORK_MANAGER_H__ */
