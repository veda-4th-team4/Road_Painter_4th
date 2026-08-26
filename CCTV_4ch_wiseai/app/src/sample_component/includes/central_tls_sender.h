#ifndef CENTRAL_TLS_SENDER_H
#define CENTRAL_TLS_SENDER_H

/**
 * Non-blocking TLS client for the central server's role=CCTV protocol
 * (HELLO/POS/H_MATRIX/CMD), ported from cctv_app/src/central_tls_sender.cpp.
 *
 * Stays separate from pose_sender, which continues serving the RPi
 * dashboard (CAM_POSE + calibration commands) — different server, different
 * port, different message set.
 *
 * One difference from the cctv_app original: this app is multi-channel from
 * day one, so send_pos() takes a channel and poll_command() understands
 * SELECT_CHANNEL — see docs/08.06/CCTV_ACTION_ITEMS_20260806.md C-1/C-2/C-3.
 * cctv_app had to retrofit those; here they are in from the start.
 */

int central_tls_sender_init(const char* server_ip, int server_port,
                            const char* ca_file);

// Second address for the same server (same port as server_ip above), tried
// when server_ip's candidate fails to come ONLINE -- e.g. one NIC's address
// works and the other's doesn't, whether from reachability or (as it is for
// this app right now) a certificate whose SAN only lists one of them. Call
// after init(); a call before init() or with a bad IP leaves no fallback
// configured (not an error the caller needs to act on). Returns 0/-1.
int central_tls_sender_set_fallback(const char* fallback_ip);

// Which candidate IP the link is currently on (or would dial next, if
// OFFLINE) -- "" if init() has not been called. For /status: with a
// fallback configured, this is the only way to tell which one is in use.
const char* central_tls_sender_active_ip(void);

// ch is carried as payload.ch (see CCTV_ACTION_ITEMS_20260806.md C-1) so the
// server does not silently attribute every channel's robot marker to
// channel 1. ch is 1-based here (CH1..CH4, docs/PROTOCOL.md's "채널 규약")
// -- callers pass this app's own 0-based channel index straight through
// unconverted for a while (2026-08-11: fixed at the one call site, in
// ProcessRawVideo/SendPosePackets) and channel 1 (index 0)'s POS silently
// never reached the server, since its active-channel default is 1 and 0
// never matched.
int central_tls_sender_send_pos(int ch, const float corners[4][2]);

// Send one already-formed message body. The envelope's "seq" is filled in
// here, so callers pass only type + payload:
//   central_tls_sender_send_typed("H_MATRIX", "{\"ch\":2,\"calib\":{...}}")
// Returns 0 when the line reached the socket, -1 while offline or if the
// message exceeds CENTRAL_TLS_MAX_LINE (app_config.h).
int central_tls_sender_send_typed(const char* type, const char* payload_json);

// Poll for one command from the server. Returns 1 and fills `out` with one
// of, 0 when nothing is available:
//   "CALIB_START"
//   "SELECT_CHANNEL <n>"   (1 <= n <= 8)
//   "SELECT_CHANNEL_BAD"   ("ch" missing or out of range)
// Call in a loop each frame until it returns 0.
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

#endif // CENTRAL_TLS_SENDER_H
