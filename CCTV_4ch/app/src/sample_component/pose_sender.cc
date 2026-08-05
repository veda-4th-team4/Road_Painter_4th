#include "pose_sender.h"
#include "app_config.h"           // CENTRAL_TLS_MAX_LINE lives here in this port

#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>

// Reconnect attempts are rate-limited so a dead server does not add a
// syscall storm on every frame. Interval configured in app_config.h.
static const long kReconnectIntervalMs = POSE_RECONNECT_MS;

enum SenderState { ST_DISCONNECTED, ST_CONNECTING, ST_CONNECTED };

static int         g_fd    = -1;
static SenderState g_state = ST_DISCONNECTED;
static long        g_lastAttemptMs = 0;
static struct sockaddr_in g_addr;
static bool        g_haveAddr = false;

static const int kControlQueueSize = 32;
// 1024, raised from 512 on 2026-08-05: an ANCHORS reply carrying a full list of
// 24 markers is ~880 bytes. At 512 the guard in
// pose_sender_send_control_line() would have DROPPED that reply rather than
// corrupting it, which is the right failure but still leaves the RPi with no
// answer to ANCHOR_QUERY on the only lens configurations worth querying.
// Costs 16 KB of static queue.
static const int kControlLineSize = 1024;
static char g_controlQueue[kControlQueueSize][kControlLineSize];
static int  g_controlHead = 0;
static int  g_controlCount = 0;

// Reported by pose_sender_get_stats() to the /status page. Counted in LINES:
// a pose that never left the camera is the symptom someone is looking for, and
// bytes would not show it any more clearly.
static unsigned long g_sentLines = 0;
static unsigned long g_droppedLines = 0;

static long now_ms(void)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (long) tv.tv_sec * 1000 + tv.tv_usec / 1000;
}

static void close_sock(void)
{
    if (g_fd >= 0) {
        close(g_fd);
        g_fd = -1;
    }
    g_state = ST_DISCONNECTED;
}

// Kick off a non-blocking connect. Completion is checked on later calls.
static void try_connect(void)
{
    long now = now_ms();
    if (now - g_lastAttemptMs < kReconnectIntervalMs)
        return;
    g_lastAttemptMs = now;

    close_sock();

    g_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (g_fd < 0)
        return;

    // Non-blocking BEFORE connect: connect() returns EINPROGRESS instead of
    // stalling the raw-video callback while the TCP handshake runs.
    fcntl(g_fd, F_SETFL, fcntl(g_fd, F_GETFL, 0) | O_NONBLOCK);

    // Small JSON lines at 5..30 Hz: Nagle would batch them for up to ~40 ms,
    // which is real latency for the robot control loop. Disable it.
    int one = 1;
    setsockopt(g_fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));

    int rc = connect(g_fd, (struct sockaddr*) &g_addr, sizeof(g_addr));
    if (rc == 0) {
        g_state = ST_CONNECTED;
    } else if (errno == EINPROGRESS) {
        g_state = ST_CONNECTING;
    } else {
        close_sock();
    }
}

// While a non-blocking connect is pending, poll its outcome without waiting.
static void check_connecting(void)
{
    fd_set wfds;
    FD_ZERO(&wfds);
    FD_SET(g_fd, &wfds);
    struct timeval zero = {0, 0};

    int rc = select(g_fd + 1, NULL, &wfds, NULL, &zero);
    if (rc < 0) {
        close_sock();
        return;
    }
    if (rc == 0)
        return; // handshake still in flight

    int err = 0;
    socklen_t len = sizeof(err);
    if (getsockopt(g_fd, SOL_SOCKET, SO_ERROR, &err, &len) != 0 || err != 0) {
        close_sock();
        return;
    }
    g_state = ST_CONNECTED;
}

void pose_sender_init(const char* server_ip, int server_port)
{
    memset(&g_addr, 0, sizeof(g_addr));
    g_addr.sin_family = AF_INET;
    g_addr.sin_port   = htons((unsigned short) server_port);
    if (inet_pton(AF_INET, server_ip, &g_addr.sin_addr) != 1)
        return;
    g_haveAddr = true;

    // Allow the first attempt immediately.
    g_lastAttemptMs = now_ms() - kReconnectIntervalMs;
    try_connect();
}

static int send_line_now(const char* json)
{
    if (!g_haveAddr || json == NULL)
        return -1;

    if (g_state == ST_DISCONNECTED)
        try_connect();
    if (g_state == ST_CONNECTING)
        check_connecting();
    if (g_state != ST_CONNECTED)
        return -1; // drop this frame; next frame carries fresher data anyway

    char buf[1024];
    int n = snprintf(buf, sizeof(buf), "%s\n", json);
    if (n <= 0 || n >= (int) sizeof(buf))
        return -1;

    // MSG_NOSIGNAL: a dead peer must produce EPIPE, not kill the app.
    ssize_t sent = send(g_fd, buf, (size_t) n, MSG_NOSIGNAL);
    if (sent == n) {
        ++g_sentLines;
        return 0;
    }

    if (sent < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
        return -1; // kernel buffer full (peer slow) — drop, do not wait

    // Partial write or hard error (EPIPE/ECONNRESET/...): a partial JSON line
    // would corrupt the newline framing, so drop the connection and let the
    // reconnect path start a clean stream.
    close_sock();
    return -1;
}

static void flush_control_queue(void)
{
    while (g_controlCount > 0) {
        if (send_line_now(g_controlQueue[g_controlHead]) != 0)
            return;
        g_controlHead = (g_controlHead + 1) % kControlQueueSize;
        --g_controlCount;
    }
}

int pose_sender_send_line(const char* json)
{
    // Control replies are ordered ahead of drop-tolerant realtime poses.
    flush_control_queue();
    // Drops are tallied HERE rather than in send_line_now(): a control line
    // that fails there is queued for retry, not lost, and counting it at the
    // socket would report a drop for every line that later goes out fine.
    if (g_controlCount > 0) {
        ++g_droppedLines;
        return -1;
    }
    int rc = send_line_now(json);
    if (rc != 0)
        ++g_droppedLines;
    return rc;
}

int pose_sender_send_control_line(const char* json)
{
    if (json == NULL || strlen(json) >= (size_t) kControlLineSize) {
        ++g_droppedLines;
        return -1;
    }

    flush_control_queue();
    if (g_controlCount == 0 && send_line_now(json) == 0)
        return 0;
    if (g_controlCount >= kControlQueueSize) {
        ++g_droppedLines;
        return -1;
    }

    int tail = (g_controlHead + g_controlCount) % kControlQueueSize;
    strncpy(g_controlQueue[tail], json, kControlLineSize - 1);
    g_controlQueue[tail][kControlLineSize - 1] = '\0';
    ++g_controlCount;
    return 0;
}

int pose_sender_poll_command(char* out, int out_len)
{
    // Must hold the longest command line the dashboard can send, which is
    // CENTRAL_HMATRIX plus a full calibration bundle.
    static char rbuf[CENTRAL_TLS_MAX_LINE];
    static int  rlen = 0;

    if (out == NULL || out_len <= 1)
        return 0;
    flush_control_queue();
    if (g_state != ST_CONNECTED) {
        rlen = 0; // stale bytes from a previous connection are meaningless
        return 0;
    }

    // Drain whatever is available right now (socket is non-blocking).
    if (rlen < (int) sizeof(rbuf) - 1) {
        ssize_t n = recv(g_fd, rbuf + rlen, sizeof(rbuf) - 1 - rlen, 0);
        if (n > 0) {
            rlen += (int) n;
        } else if (n == 0) {
            close_sock(); // orderly shutdown by the server
            rlen = 0;
            return 0;
        } else if (errno != EAGAIN && errno != EWOULDBLOCK) {
            close_sock();
            rlen = 0;
            return 0;
        }
    }

    rbuf[rlen] = '\0';
    char* nl = strchr(rbuf, '\n');
    if (nl == NULL) {
        if (rlen >= (int) sizeof(rbuf) - 1)
            rlen = 0; // oversized garbage line: discard, resync on next '\n'
        return 0;
    }

    int lineLen = (int) (nl - rbuf);
    int consumed = (int) (nl - rbuf) + 1;

    // A line that does not fit is DISCARDED, not trimmed.
    //
    // Trimming was silent, and silent trimming is worse here than losing the
    // command: ANCHOR_SET_ALL is a list of numbers, so a cut in the middle of
    // one still parses as a complete, plausible marker — the camera would
    // accept a calibration input that is not what anybody sent. The count
    // argument on that command catches this too; this is the layer that makes
    // it impossible rather than merely detectable.
    if (lineLen > out_len - 1) {
        printf("[ArucoPosePNM] control line of %d bytes exceeds the %d-byte "
               "command budget — discarded\n", lineLen, out_len - 1);
        fflush(stdout);
        memmove(rbuf, rbuf + consumed, rlen - consumed);
        rlen -= consumed;
        return 0;
    }

    memcpy(out, rbuf, lineLen);
    out[lineLen] = '\0';

    memmove(rbuf, rbuf + consumed, rlen - consumed);
    rlen -= consumed;
    return 1;
}

bool pose_sender_is_connected(void)
{
    return g_state == ST_CONNECTED;
}

void pose_sender_get_stats(unsigned long* sent, unsigned long* dropped, int* queued)
{
    if (sent)    *sent    = g_sentLines;
    if (dropped) *dropped = g_droppedLines;
    if (queued)  *queued  = g_controlCount;
}

void pose_sender_close(void)
{
    close_sock();
    g_controlHead = 0;
    g_controlCount = 0;
}
