#include "NetworkManager.h"
#include <nlohmann/json.hpp>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <iostream>
#include <cstring>
#include <chrono>

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
      msg_seq(0),
      has_new_cmd(false),
      align_angle_deg(0.0f),
      has_align_cmd(false),
      go_signal_received(false),
      drift_angle_deg(0.0f),
      has_drift_cmd(false),
      current_path_phase("")
{
    std::memset(&latest_pose, 0, sizeof(Pose_t));
}

NetworkManager::~NetworkManager() {
    Close();
}

bool NetworkManager::Init() {
    if (is_connected) return true;

    // Clean up any previous state/threads safely
    Close();

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
    if (!ssl_send_line(hello_msg.dump())) {
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
    if (!is_connected) {
        static auto last_retry = std::chrono::steady_clock::now();
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(now - last_retry).count() >= 5) {
            last_retry = now;
            std::cout << "[NetworkManager] Retrying connection to Vision Server..." << std::endl;
            Init();
        }
    }
}

bool NetworkManager::SendStatus(const Msg_Status_t& status) {
    if (!is_connected || !ssl_connection) return false;

    std::string state_str = "IDLE";
    if (status.flags & 0x02) {          // STATUS_FLAG_ESTOP (1U << 1)
        state_str = "ESTOPPED";
    } else if (status.flags & 0x01) {   // STATUS_FLAG_MOVING (1U << 0)
        state_str = "MOVING";
    }

    bool painting_bool = (status.flags & 0x08) ? true : false; // STATUS_FLAG_NOZZLE (1U << 3)

    json status_payload = {
        {"state", state_str},
        {"painting", painting_bool}
    };

    json status_msg = {
        {"type", "STATUS"},
        {"seq", ++msg_seq},
        {"payload", status_payload}
    };

    return ssl_send_line(status_msg.dump());
}

bool NetworkManager::SendReady(uint32_t seg_index) {
    if (!is_connected || !ssl_connection) return false;

    json ready_payload = {{"seg", seg_index}};
    json ready_msg = {
        {"type", "READY"},
        {"seq", ++msg_seq},
        {"payload", ready_payload}
    };

    return ssl_send_line(ready_msg.dump());
}

bool NetworkManager::GetAlignCommand(float& out_angle_deg) {
    std::lock_guard<std::mutex> lock(align_mutex);
    if (!has_align_cmd) return false;
    out_angle_deg = align_angle_deg;
    has_align_cmd = false;
    return true;
}

bool NetworkManager::CheckAndClearGoSignal() {
    return go_signal_received.exchange(false);
}

bool NetworkManager::GetDriftCorrection(float& out_angle_deg) {
    std::lock_guard<std::mutex> lock(drift_mutex);
    if (!has_drift_cmd) return false;
    out_angle_deg = drift_angle_deg;
    has_drift_cmd = false;
    return true;
}

std::string NetworkManager::GetPathPhase() {
    std::lock_guard<std::mutex> lock(path_mutex);
    return current_path_phase;
}

bool NetworkManager::GetLatestPose(Pose_t& out_pose) {
    std::lock_guard<std::mutex> lock(pose_mutex);
    if (!has_new_pose) return false;
    
    out_pose = latest_pose;
    has_new_pose = false;
    return true;
}

bool NetworkManager::GetPath(std::vector<Segment_t>& out_path) {
    std::lock_guard<std::mutex> lock(path_mutex);
    if (!has_new_path) return false;
    
    out_path = current_path;
    has_new_path = false;
    return true;
}

bool NetworkManager::GetLatestCommand(std::string& out_cmd) {
    std::lock_guard<std::mutex> lock(cmd_mutex);
    if (!has_new_cmd) return false;
    out_cmd = latest_cmd;
    has_new_cmd = false;
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
            is_connected = false;
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

bool NetworkManager::ssl_send_line(const std::string& raw_json_message) {
    if (!ssl_connection) return false;

    std::string raw_data = raw_json_message + "\n";

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
            current_path_phase = payload.value("phase", "draw");
            
            // Read segment array from nested "segments" key
            auto segments = payload.value("segments", json::array());
            for (const auto& seg : segments) {
                if (seg.is_object()) {
                    Segment_t segment;
                    segment.op = seg.value("op", "");
                    segment.dist_m = seg.value("dist_m", 0.0f);
                    segment.angle_deg = seg.value("angle_deg", 0.0f);
                    segment.paint = seg.value("paint", false);
                    segment.heading_deg = seg.value("heading_deg", 0.0f);
                    current_path.push_back(segment);
                }
            }
            has_new_path = true;
            std::cout << "[NetworkManager] Path update (phase=" << current_path_phase 
                      << "): " << current_path.size() << " segments received." << std::endl;

        } else if (type == "ALIGN") {
            float angle = payload.value("angle_deg", 0.0f);
            std::cout << "[NetworkManager] Received ALIGN: " << angle << " deg" << std::endl;
            std::lock_guard<std::mutex> lock(align_mutex);
            align_angle_deg = angle;
            has_align_cmd = true;

        } else if (type == "GO") {
            std::cout << "[NetworkManager] Received GO signal from server." << std::endl;
            go_signal_received = true;

        } else if (type == "DRIFT") {
            float angle = payload.value("angle_deg", 0.0f);
            std::cout << "[NetworkManager] Received DRIFT feedback: " << angle << " deg" << std::endl;
            std::lock_guard<std::mutex> lock(drift_mutex);
            drift_angle_deg = angle;
            has_drift_cmd = true;

        } else if (type == "ACK") {
            std::cout << "[NetworkManager] Received ACK from server: " << payload.value("msg", "") << std::endl;
        } else if (type == "CMD") {
            std::string cmd = payload.value("cmd", "");
            std::cout << "[NetworkManager] Received CMD from server: " << cmd << std::endl;
            std::lock_guard<std::mutex> lock(cmd_mutex);
            latest_cmd = cmd;
            has_new_cmd = true;
        } else {
            std::cout << "[NetworkManager] Unknown message type: " << type << std::endl;
        }
    } catch (const std::exception& e) {
        std::cerr << "[NetworkManager] Parsing exception: " << e.what() << " on data: " << line << std::endl;
    }
}
