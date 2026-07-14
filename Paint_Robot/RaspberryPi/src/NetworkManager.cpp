#include "NetworkManager.h"
#include <nlohmann/json.hpp>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <iostream>
#include <cstring>

using json = nlohmann::json;

NetworkManager::NetworkManager(const std::string& ip, uint16_t port)
    : server_ip(ip),
      server_port(port),
      client_fd(-1),
      is_connected(false),
      ssl_ctx(nullptr),
      ssl_connection(nullptr),
      rx_alive(false),
      has_new_pose(false),
      has_new_path(false),
      msg_seq(0)
{
    std::memset(&latest_pose, 0, sizeof(Pose_t));
}

NetworkManager::~NetworkManager() {
    Close();
}

bool NetworkManager::Init() {
    if (is_connected) return true;

    // 1. Initialize OpenSSL client context
    ssl_ctx = SSL_CTX_new(TLS_client_method());
    if (!ssl_ctx) {
        std::cerr << "[NetworkManager] Error: Failed to create SSL context." << std::endl;
        return false;
    }
    SSL_CTX_set_min_proto_version(ssl_ctx, TLS1_2_VERSION);

    // 2. Load the Vision Server self-signed certificate (server.crt Pinning)
    std::string cert_path = "server.crt"; // Must be in the execution path
    if (SSL_CTX_load_verify_locations(ssl_ctx, cert_path.c_str(), nullptr) != 1) {
        std::cerr << "[NetworkManager] Error: Failed to load server.crt certificate." << std::endl;
        SSL_CTX_free(ssl_ctx);
        ssl_ctx = nullptr;
        return false;
    }
    SSL_CTX_set_verify(ssl_ctx, SSL_VERIFY_PEER, nullptr);

    // 3. Create standard TCP socket
    client_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (client_fd < 0) {
        std::cerr << "[NetworkManager] Error: Failed to create TCP socket." << std::endl;
        SSL_CTX_free(ssl_ctx);
        ssl_ctx = nullptr;
        return false;
    }

    // 4. Configure Server destination address
    sockaddr_in server_addr{};
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(server_port);
    if (inet_pton(AF_INET, server_ip.c_str(), &server_addr.sin_addr) <= 0) {
        std::cerr << "[NetworkManager] Error: Invalid IP address: " << server_ip << std::endl;
        close(client_fd);
        client_fd = -1;
        SSL_CTX_free(ssl_ctx);
        ssl_ctx = nullptr;
        return false;
    }

    // 5. Connect to TCP server
    std::cout << "[NetworkManager] Connecting to server at " << server_ip << ":" << server_port << "..." << std::endl;
    if (connect(client_fd, (sockaddr*)&server_addr, sizeof(server_addr)) != 0) {
        std::cerr << "[NetworkManager] Error: TCP connection failed." << std::endl;
        close(client_fd);
        client_fd = -1;
        SSL_CTX_free(ssl_ctx);
        ssl_ctx = nullptr;
        return false;
    }

    // 6. Bind socket to SSL connection and perform TLS Handshake
    ssl_connection = SSL_new(ssl_ctx);
    SSL_set_fd(ssl_connection, client_fd);
    if (SSL_connect(ssl_connection) != 1) {
        std::cerr << "[NetworkManager] Error: TLS handshake failed." << std::endl;
        ERR_print_errors_fp(stderr);
        SSL_free(ssl_connection);
        ssl_connection = nullptr;
        close(client_fd);
        client_fd = -1;
        SSL_CTX_free(ssl_ctx);
        ssl_ctx = nullptr;
        return false;
    }

    is_connected = true;
    std::cout << "[NetworkManager] Secure TLS connection established." << std::endl;

    // 7. Transmit HELLO registration message to server
    json hello_payload = {{"role", "ROBOT"}};
    json hello_msg = {
        {"type", "HELLO"},
        {"seq", ++msg_seq},
        {"payload", hello_payload}
    };
    if (!ssl_send_line("HELLO", hello_payload.dump())) {
        std::cerr << "[NetworkManager] Error: Failed to transmit HELLO frame." << std::endl;
        Close();
        return false;
    }

    // 8. Spin off background listener thread
    rx_alive = true;
    rx_thread = std::thread(&NetworkManager::rx_loop, this);

    return true;
}

void NetworkManager::Close() {
    rx_alive = false;
    
    // Shut down TCP socket to break out of blocking SSL_read
    if (client_fd >= 0) {
        shutdown(client_fd, SHUT_RDWR);
    }

    // Await background thread death
    if (rx_thread.joinable()) {
        rx_thread.join();
    }

    if (ssl_connection) {
        SSL_shutdown(ssl_connection);
        SSL_free(ssl_connection);
        ssl_connection = nullptr;
    }

    if (client_fd >= 0) {
        close(client_fd);
        client_fd = -1;
    }

    if (ssl_ctx) {
        SSL_CTX_free(ssl_ctx);
        ssl_ctx = nullptr;
    }

    is_connected = false;
    std::cout << "[NetworkManager] Closed connection and released SSL resources." << std::endl;
}

void NetworkManager::Process() {
    // Left empty since receiver is handled by the background thread.
}

bool NetworkManager::SendStatus(const Msg_Status_t& status) {
    if (!is_connected || !ssl_connection) return false;

    // Parse STM32 metrics to match server STATUS schema
    json status_payload = {
        {"state", (status.flags & 0x01) ? "ESTOP" : "MOVING"},
        {"x", static_cast<double>(status.left_steps) * 0.001},  // Dummy scale
        {"y", static_cast<double>(status.right_steps) * 0.001}, // Dummy scale
        {"battery", 100}                                       // Default
    };

    return ssl_send_line("STATUS", status_payload.dump());
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

void NetworkManager::rx_loop() {
    std::string buffer;
    std::string line;

    while (rx_alive) {
        if (ssl_read_line(buffer, line)) {
            if (line.empty()) continue;
            parse_incoming_data(line);
        } else {
            std::cerr << "[NetworkManager] SSL socket connection closed or error occurred." << std::endl;
            rx_alive = false;
        }
    }
}

bool NetworkManager::ssl_read_line(std::string& buf, std::string& line) {
    for (;;) {
        auto pos = buf.find('\n');
        if (pos != std::string::npos) {
            line = buf.substr(0, pos);
            buf.erase(0, pos + 1);
            return true;
        }
        
        char temp_buf[1024];
        int bytes_read = SSL_read(ssl_connection, temp_buf, sizeof(temp_buf) - 1);
        if (bytes_read <= 0) return false;

        temp_buf[bytes_read] = '\0';
        buf.append(temp_buf, bytes_read);
    }
}

bool NetworkManager::ssl_send_line(const std::string& type, const std::string& payload_json) {
    if (!ssl_connection) return false;

    // Serialize packet wrapper
    json msg = {
        {"type", type},
        {"seq", ++msg_seq},
        {"payload", json::parse(payload_json)}
    };

    std::string raw_data = msg.dump() + "\n";

    std::lock_guard<std::mutex> lock(write_mutex);
    int bytes_written = SSL_write(ssl_connection, raw_data.data(), static_cast<int>(raw_data.size()));
    return bytes_written > 0;
}

void NetworkManager::parse_incoming_data(const std::string& line) {
    try {
        json msg = json::parse(line);
        std::string type = msg.value("type", "");
        json payload = msg.value("payload", json::object());

        if (type == "PATH") {
            std::lock_guard<std::mutex> lock(path_mutex);
            current_path.clear();
            
            // Read coordinate array
            auto points = payload.value("points", json::array());
            for (const auto& pt : points) {
                if (pt.is_array() && pt.size() >= 2) {
                    Waypoint_t wp;
                    wp.x = pt[0].get<float>();
                    wp.y = pt[1].get<float>();
                    wp.nozzle_on = 1; // Default trigger (can be refined later)
                    wp.speed = 0.05f; // Default target
                    current_path.push_back(wp);
                }
            }
            has_new_path = true;
            std::cout << "[NetworkManager] Path data update: " << current_path.size() << " waypoints received." << std::endl;

            // [Test Echo] Send status response back to server upon receiving path
            Msg_Status_t path_ack_status = {0, 0, 0};
            SendStatus(path_ack_status);
            std::cout << "[NetworkManager] [TEST] PATH received -> Sent status response to server." << std::endl;

        } else if (type == "POS") {
            std::lock_guard<std::mutex> lock(pose_mutex);
            latest_pose.x = payload.value("x", 0.0f);
            latest_pose.y = payload.value("y", 0.0f);
            latest_pose.theta = payload.value("theta", 0.0f);
            latest_pose.timestamp_ms = payload.value("ts", 0);
            latest_pose.confidence = 100; // Set confidence high on valid packet
            has_new_pose = true;

            // [Test Echo] Send status response back to server upon receiving absolute position
            Msg_Status_t pose_ack_status = {1000, 2000, 0x00}; // Dummy steps: 1000, 2000
            SendStatus(pose_ack_status);
            std::cout << "[NetworkManager] [TEST] POS received -> Sent status response to server." << std::endl;

        } else if (type == "ACK") {
            std::cout << "[NetworkManager] Received ACK from server: " << payload.value("msg", "") << std::endl;
        } else {
            std::cout << "[NetworkManager] Unknown message type: " << type << std::endl;
        }
    } catch (const std::exception& e) {
        std::cerr << "[NetworkManager] Parsing exception: " << e.what() << " on data: " << line << std::endl;
    }
}
