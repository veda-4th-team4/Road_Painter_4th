#include "snapshot_sender.h"

#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <sys/time.h>

static struct sockaddr_in g_addr;
static bool g_haveAddr = false;

// A snapshot is rare and must land whole, so the socket is a plain blocking
// one (unlike pose_sender's non-blocking realtime socket) — but bounded by a
// timeout so a wedged link cannot stall the frame callback indefinitely.
static const int kIoTimeoutSec = 3;

void snapshot_sender_init(const char* server_ip, int server_port)
{
    memset(&g_addr, 0, sizeof(g_addr));
    g_addr.sin_family = AF_INET;
    g_addr.sin_port   = htons((unsigned short) server_port);
    if (inet_pton(AF_INET, server_ip, &g_addr.sin_addr) != 1)
        return;
    g_haveAddr = true;
}

// send() the full buffer, looping over partial writes. Returns 0 on success,
// -1 on error/timeout.
static int send_all(int fd, const void* buf, size_t len)
{
    const char* p = (const char*) buf;
    size_t sent = 0;
    while (sent < len) {
        ssize_t n = send(fd, p + sent, len - sent, MSG_NOSIGNAL);
        if (n > 0) {
            sent += (size_t) n;
            continue;
        }
        // EINTR is the only retryable case: a signal arrived before anything
        // moved, and the transfer is still viable.
        //
        // EAGAIN/EWOULDBLOCK must NOT be retried here, even though the socket is
        // a blocking one -- that pair is exactly how SO_SNDTIMEO reports that it
        // fired. Retrying re-arms another full timeout, so a server that has
        // stopped draining wedges this loop forever, and it runs on the frame
        // callback thread (a raw snapshot is 1920*1080*3 = 6.2 MB, and the RPi
        // reads snapshots on one serial thread). Failing here is what makes the
        // "bounded by a timeout" promise at the top of this file real: the
        // snapshot is lost, the caller closes the socket, and detection
        // continues.
        if (n < 0 && errno == EINTR)
            continue;
        return -1;
    }
    return 0;
}

// Shared framing: [json_len][json][width][height][payload_len][payload].
// Identical wire layout for raw-RGB and pre-encoded-JPEG snapshots; the
// server distinguishes them by a "format" field inside the JSON. Blocking
// with a short timeout; opens and closes one connection per call.
static int send_framed(const char* json, int json_len,
                       int width, int height,
                       const uint8_t* payload, int payload_len)
{
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0)
        return -1;

    struct timeval tv;
    tv.tv_sec = kIoTimeoutSec;
    tv.tv_usec = 0;
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    int one = 1;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));

    if (connect(fd, (struct sockaddr*) &g_addr, sizeof(g_addr)) != 0) {
        close(fd);
        return -1;
    }

    uint32_t jsonLenNet   = htonl((uint32_t) json_len);
    uint32_t widthNet     = htonl((uint32_t) width);
    uint32_t heightNet    = htonl((uint32_t) height);
    uint32_t payloadLenNet = htonl((uint32_t) payload_len);

    int ok = send_all(fd, &jsonLenNet, sizeof(jsonLenNet)) == 0 &&
             send_all(fd, json, (size_t) json_len) == 0 &&
             send_all(fd, &widthNet, sizeof(widthNet)) == 0 &&
             send_all(fd, &heightNet, sizeof(heightNet)) == 0 &&
             send_all(fd, &payloadLenNet, sizeof(payloadLenNet)) == 0 &&
             send_all(fd, payload, (size_t) payload_len) == 0;

    close(fd);
    return ok ? 0 : -1;
}

int snapshot_sender_send(const char* json, int json_len,
                         int width, int height,
                         const uint8_t* rgb, int rgb_len)
{
    if (!g_haveAddr || json == NULL || json_len <= 0 ||
        width <= 0 || height <= 0 || rgb == NULL ||
        rgb_len != width * height * 3)
        return -1;
    return send_framed(json, json_len, width, height, rgb, rgb_len);
}

int snapshot_sender_send_jpeg(const char* json, int json_len,
                              int width, int height,
                              const uint8_t* jpeg, int jpeg_len)
{
    if (!g_haveAddr || json == NULL || json_len <= 0 ||
        width <= 0 || height <= 0 || jpeg == NULL || jpeg_len <= 0)
        return -1;
    return send_framed(json, json_len, width, height, jpeg, jpeg_len);
}
