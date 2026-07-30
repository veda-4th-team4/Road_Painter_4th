#ifndef CENTRAL_TLS_SENDER_H
#define CENTRAL_TLS_SENDER_H

// Non-blocking TLS client for the central server's role=CCTV protocol. This
// stays separate from pose_sender, which continues serving web_gui on 7000.
int central_tls_sender_init(const char* server_ip, int server_port,
                            const char* ca_file);
int central_tls_sender_send_pos(const float corners[4][2]);

// Send one already-formed message body. The envelope's "seq" is filled in
// here, so callers pass only type + payload:
//   central_tls_sender_send_typed("H_MATRIX", "{\"calib\":{...}}")
// Returns 0 when the line reached the socket, -1 while offline or if the
// message exceeds CENTRAL_TLS_MAX_LINE.
int central_tls_sender_send_typed(const char* type, const char* payload_json);

int central_tls_sender_poll_command(char* out, int out_len);

// Operator kill switch for the link itself. Disabling drops the session and
// stops all reconnect attempts; re-enabling retries immediately rather than
// waiting out the backoff. Survives across init(). Enabled at boot.
void central_tls_sender_set_enabled(int on);
int  central_tls_sender_enabled(void);

// Link state for status reporting: "disabled", "offline", "connecting",
// "handshaking" or "online". Never NULL.
const char* central_tls_sender_state(void);

void central_tls_sender_close(void);

// Largest single line (including the newline) the sender will emit. An
// H_MATRIX carrying K, D and two 3x3 matrices lands near 700 bytes; the
// central protocol's own ceiling is 16 KiB.
#define CENTRAL_TLS_MAX_LINE 2048

#endif // CENTRAL_TLS_SENDER_H
