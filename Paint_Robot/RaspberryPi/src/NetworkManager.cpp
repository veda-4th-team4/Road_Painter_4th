#include "NetworkManager.h"
#include <iostream>
#include <cstring>

NetworkManager::NetworkManager(const std::string& ip, uint16_t port)
    : server_ip(ip), server_port(port), client_fd(-1), is_connected(false), has_new_pose(false), has_new_path(false) {
    std::memset(&latest_pose, 0, sizeof(Pose_t));
}

NetworkManager::~NetworkManager() {
    Close();
}

bool NetworkManager::Init() {
    // TODO: Implement actual socket connect & TLS handshake setup
    std::cout << "[NetworkManager] Connecting to server " << server_ip << ":" << server_port << " (stub)..." << std::endl;
    is_connected = true; // Temporary mock
    return true;
}

void NetworkManager::Close() {
    if (is_connected) {
        std::cout << "[NetworkManager] Socket disconnected (stub)." << std::endl;
        is_connected = false;
    }
}

void NetworkManager::Process() {
    if (!is_connected) return;

    // TODO: Perform non-blocking socket reads
    // parse_incoming_data(buf, bytes_read);
}

bool NetworkManager::SendStatus(const Msg_Status_t& status) {
    if (!is_connected) return false;
    
    // TODO: Serialize and transmit status
    return true;
}

bool NetworkManager::GetLatestPose(Pose_t& out_pose) {
    std::lock_guard<std::mutex> lock(pose_mutex);
    if (!has_new_pose) return false;
    
    out_pose = latest_pose;
    has_new_pose = false;
    return true;
}

bool NetworkManager::GetPath(std::vector<Waypoint_t>& out_path) {
    std::lock_guard<std::mutex> lock(path_mutex);
    if (!has_new_path) return false;
    
    out_path = current_path;
    has_new_path = false;
    return true;
}

void NetworkManager::parse_incoming_data(const uint8_t* buffer, size_t size) {
    // TODO: Implement protocol deserialization (JSON or Binary parsing)
}
