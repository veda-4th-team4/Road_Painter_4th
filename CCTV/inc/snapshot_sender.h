#ifndef SNAPSHOT_SENDER_H
#define SNAPSHOT_SENDER_H

#include <stdint.h>

/**
 * One-shot, reliable upload of an LDC_SNAPSHOT (metrics JSON + raw RGB image)
 * to the vision server, on a DEDICATED connection/port separate from the
 * realtime pose_sender channel.
 *
 * Why separate: pose_sender is intentionally non-blocking and drop-tolerant
 * (a lost pose packet is harmless — the next frame carries fresher data).
 * A snapshot is the opposite: rare (only on the LDC_SNAPSHOT command), and
 * must arrive whole or not at all. Sharing one channel would force either the
 * realtime path to block waiting for a big image, or the snapshot to be
 * silently dropped like a pose packet — neither is acceptable. A short-lived
 * blocking connection, opened only when a snapshot is requested, keeps both
 * concerns isolated at the cost of one brief (sub-second, on a LAN) stall in
 * the frame callback — the same trade-off already accepted for
 * CALIB_K_START's calibrateCamera() call.
 *
 * Why raw RGB, not JPEG: the on-camera OpenCV build intentionally excludes
 * imgcodecs to keep the binary small (see opencv_cross/build_opencv.sh), so
 * cv::imencode is not linkable here. The RPi side, a plain Python script,
 * writes the raw bytes out as a .ppm (Netpbm) file — no codec needed there
 * either; convert to .jpg/.png with any image tool if desired.
 *
 * Wire format (all integers big-endian / network byte order):
 *   uint32  json_len
 *   bytes   json_len bytes of UTF-8 JSON (the metrics)
 *   uint32  width
 *   uint32  height
 *   uint32  pixel_len         (must equal width * height * 3)
 *   bytes   pixel_len bytes, row-major, 3 bytes/pixel, RGB order
 * One connection carries exactly one snapshot, then closes.
 */

// Set destination once at startup (idempotent), mirrors pose_sender_init.
void snapshot_sender_init(const char* server_ip, int server_port);

// Connect, send [json][width][height][rgb pixels] per the wire format above,
// close. Blocking with a short timeout — call only from the rare
// LDC_SNAPSHOT command path, never from the per-frame pose path.
// `rgb` must be `width * height * 3` bytes, row-major, RGB order (use
// cv::cvtColor(..., COLOR_BGR2RGB) first — OpenCV Mats are BGR by default).
// Returns 0 on success, -1 on any failure; caller should just log and move on.
int snapshot_sender_send(const char* json, int json_len,
                         int width, int height,
                         const uint8_t* rgb, int rgb_len);

// Same as snapshot_sender_send but for an already-compressed JPEG payload:
// the same wire framing is used, `pixel_len` carries the JPEG byte count
// (NOT width*height*3), and the server must be told it is JPEG via a
// "format":"jpeg" field in `json`. width/height are informational (the JPEG
// carries its own dimensions). Used to upload stored calibration views
// cheaply — a JPEG is ~10-30x smaller than the raw RGB frame, so the transfer
// no longer stalls the frame path the way a raw snapshot does.
int snapshot_sender_send_jpeg(const char* json, int json_len,
                              int width, int height,
                              const uint8_t* jpeg, int jpeg_len);

#endif // SNAPSHOT_SENDER_H
