#ifndef POSE_SENDER_H
#define POSE_SENDER_H

/**
 * Minimal TCP client for streaming ArUco pose packets (IF-TCP-003) to the
 * vision server (RPi).
 *
 * Design constraints (see SRS / session notes):
 *  - persistent connection: connect once, reuse for every frame
 *  - non-blocking socket: send() must NEVER stall the raw-video callback,
 *    the same way serial debug logging used to (a frame that cannot be sent
 *    right now is simply dropped — the next frame carries fresher data.
 *    Low-rate calibration/control replies use a bounded retry queue instead)
 *  - TCP_NODELAY: disable Nagle so small JSON lines go out immediately
 *  - auto-reconnect: if the server drops (reboot, cable pull), retry with a
 *    rate limit instead of blocking or giving up
 *
 * Message framing: caller passes one complete JSON object per call; a '\n'
 * terminator is appended here (newline-delimited JSON).
 */

// Set the destination once at startup (idempotent).
void pose_sender_init(const char* server_ip, int server_port);

// Send one JSON line (without trailing newline; it is appended internally).
// Returns 0 on success, -1 if dropped (not connected / would block).
// Never blocks; handles reconnection internally.
int pose_sender_send_line(const char* json);

// Queue a control/status response that must survive temporary EAGAIN or a
// reconnect (calibration ACK/progress/result). The fixed queue is flushed
// before fresh pose packets and never blocks the frame callback.
// Returns 0 when sent or queued, -1 only if invalid/queue full.
int pose_sender_send_control_line(const char* json);

// Poll for one newline-terminated command line FROM the server (the TCP
// connection is bidirectional). Non-blocking: returns 1 and copies the line
// (without '\n') into out when a complete line is available, 0 otherwise.
// Call in a loop each frame until it returns 0.
/**
 * The longest command or control line this transport carries, including the
 * caller's terminating NUL.
 *
 * Exported because FOUR places have to agree on it and three of them are in
 * another translation unit: the queue that holds outgoing control lines, the
 * buffer PollDashboardCommands() reads into, the length POST /cmd refuses
 * above, and the reply buffers ReportAnchors()/ReportMarkerPlane() format into.
 * They were literals, and they were raised together once by hand — 1024 is
 * what a 24-marker ANCHOR_SET_ALL (~620 B) and its ANCHORS reply (~880 B)
 * need. Getting three of four right next time means either a 413 on a body the
 * transport would have carried, or a reply the queue silently refuses: the
 * exact silent-truncation class the count argument on ANCHOR_SET_ALL exists to
 * rule out.
 */
#define POSE_SENDER_MAX_LINE 1024

int pose_sender_poll_command(char* out, int out_len);

// True while the TCP link to the vision server is up. A pending non-blocking
// connect counts as NOT connected — the point of this is "are packets actually
// leaving the camera", and during the handshake they are not.
//
// Added for the on-camera /status page: without it, "no poses on the dashboard"
// gives no way to tell a dead link apart from a detector that finds nothing.
bool pose_sender_is_connected(void);

// Counters since start-up, for the same reason. `sent` and `dropped` count
// LINES handed to pose_sender_send_line / _send_control_line, not bytes; a
// drop is a line the socket could not take (down, or kernel buffer full).
// `queued` is the current depth of the control-retry queue. Any pointer may be
// NULL.
void pose_sender_get_stats(unsigned long* sent, unsigned long* dropped, int* queued);

// Close the socket (app shutdown).
void pose_sender_close(void);

#endif // POSE_SENDER_H
