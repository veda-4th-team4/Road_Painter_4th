#include "sample_component.h"

#include <dirent.h>
#include <limits.h>
#include <stdarg.h>
#include <stdio.h>    // popen/pclose/fgets — SHELL command (ENABLE_SHELL_CMD)
#include <math.h>     // atan2 — CAM_POSE heading
#include <stdlib.h>   // strtol/strtod — ANCHOR_SET_ALL parses by hand
#include <string.h>
#include <time.h>     // gmtime_r/strftime — 캘리 번들의 created_at (UTC)
#include <sys/stat.h>
#include <sys/wait.h> // WEXITSTATUS/WIFEXITED — SHELL command
#include <malloc.h>          // mallinfo2, mallopt, malloc_trim — see heap_bytes()
#include <sys/resource.h>
#include <sys/time.h>
#include <unistd.h>

#include <algorithm>  // std::min/std::sort — IVA_SYNC hull

#include <opencv2/core.hpp>

#include "app_config.h"
#include "dispatcher_serialize.h"
#include "i_app_dispatcher.h"
#include "i_log_manager.h"
#include "i_p_metadata_manager.h"   // IPMetadataManager::eMetadataRequest — WiseAI bbox
#include "i_p_open_platform_manager.h"
#include "i_p_stream_provider_manager_video_raw.h"
#include "i_p_video_frame_raw.h"   // IPVideoFrameRaw, RawImage, RAW_FMT_*, RAW_PLANE_*
#include "i_pl_video_frame_raw.h"  // IPLVideoFrameRaw (concrete)
#include "pose_sender.h"
#include "central_tls_sender.h"
#include "wiseai_metadata.h"

namespace {

// Marker id streamed to the central server as POS, and whether POS streaming
// is currently on at all. Central's POS schema carries no marker id -- every
// POS it receives is taken as THE robot -- so anchors/validation markers must
// be filtered here or the last marker of the frame would win. Boot default
// only: CENTRAL_ID/CENTRAL_POS retarget these live over the RPi dashboard
// channel, the same way cctv_app's did.
int  g_central_marker_id = ROBOT_MARKER_ID;
bool g_central_pos_enabled = true;
// 주행 캘리의 정지 판정 임계값. 컴파일 상수는 부팅 기본값일 뿐이고 ODOM_SETTLE
// 명령으로 현장에서 바꾼다 — 적정값이 렌즈·거리·조명·로봇 감속 특성에 달려 있어
// 책상에서 정할 수 있는 숫자가 아니다(2px 은 잠정값이라고 app_config.h 에
// 적어 둔 그것이다).
//
// 채널별이 아니라 전역인 이유: 이 값은 "마커 코너 검출이 얼마나 흔들리는가"에
// 대한 판단이고, 한 세션은 한 렌즈만 쓴다. 채널마다 따로 두면 세션을 열 때마다
// 어느 채널 값이 적용되는지 헷갈리는 대신 얻는 게 없다.
float g_odom_settle_spread_px = ODOM_SETTLE_SPREAD_PX;
// 몇 프레임을 모아 판정할지. 상한은 버퍼 크기(ODOM_SETTLE_FRAMES)다 — 배열이
// 고정이라 그 이상은 못 받는다. 하한 2: 표준편차를 내려면 최소 두 표본이 필요하다.
int   g_odom_settle_frames = ODOM_SETTLE_FRAMES;
auto eventToArgumentBuffer = [](Event* event) {
  auto blob = event->GetBlobArgument();
  std::pair<std::variant<BaseObject*, char*>, uint64_t> ret((char*)blob.GetRawData(),  // variant
                                                            blob.GetSize());           // size
  return ret;
};

long epoch_ms() {
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return (long)tv.tv_sec * 1000L + tv.tv_usec / 1000L;
}

const char* ProximityStateName(ProximityGuard::State s) {
  switch (s) {
    case ProximityGuard::State::kSafe:    return "safe";
    case ProximityGuard::State::kCaution: return "caution";
    case ProximityGuard::State::kDanger:  return "danger";
  }
  return "?";
}

/**
 * Convex hull (Andrew's monotone chain), self-written for IVA_SYNC instead
 * of calling cv::convexHull().
 *
 * Not a style choice: cv::convexHull() reliably crashed this app at
 * component load on the camera (2026-08-18) -- EVERY build with a linked
 * call site to it aborted in LifeCycleManager::CreateComponents before any
 * of this code could possibly run, and every build without one didn't,
 * bisected command-by-command until this was the last variable left (a
 * locally-constructed std::vector<cv::Point2f> turned out to be a second,
 * independent trigger of the identical symptom -- see the "no std::vector"
 * note on the IVA_SYNC handler below). Root cause not identified beyond
 * that; both point at something in a rarely-linked corner of this
 * platform's static OpenCV build rather than at anything in the calling
 * code. This function uses no OpenCV algorithm entry points and no
 * container beyond a fixed local buffer, so it does not share whatever
 * that cause is.
 *
 * `pts` is sorted in place. Returns the hull vertex count (<= n), written
 * counter-clockwise into `out` (capped at `cap`, which the caller sizes to
 * hold every input point so the cap never actually binds).
 */
int ConvexHull2f(cv::Point2f* pts, int n, cv::Point2f* out, int cap) {
  if (n < 3 || out == NULL || cap < 3) return 0;
  std::sort(pts, pts + n, [](const cv::Point2f& a, const cv::Point2f& b) {
    return (a.x != b.x) ? (a.x < b.x) : (a.y < b.y);
  });
  auto cross = [](const cv::Point2f& o, const cv::Point2f& a, const cv::Point2f& b) {
    return (double)(a.x - o.x) * (b.y - o.y) - (double)(a.y - o.y) * (b.x - o.x);
  };
  // Upper bound on hull size is n (every point on the hull); the working
  // buffer holds both chains before the duplicate join point is dropped, so
  // it needs headroom above that -- 2n is the standard bound for this
  // algorithm. HomographyMapper::kMaxAnchors is the only n this is ever
  // called with here, so size against that rather than a template/VLA.
  cv::Point2f hull[2 * HomographyMapper::kMaxAnchors];
  int k = 0;
  for (int i = 0; i < n; ++i) {
    while (k >= 2 && cross(hull[k - 2], hull[k - 1], pts[i]) <= 0) --k;
    hull[k++] = pts[i];
  }
  const int lower = k + 1;
  for (int i = n - 2; i >= 0; --i) {
    while (k >= lower && cross(hull[k - 2], hull[k - 1], pts[i]) <= 0) --k;
    hull[k++] = pts[i];
  }
  --k;  // last point duplicates the first
  const int m = std::min(k, cap);
  for (int i = 0; i < m; ++i) out[i] = hull[i];
  return m;
}

#if ENABLE_SHELL_CMD
// JSON-escapes SRC into DST (quote/backslash escaped, control bytes -> space),
// for the one place in this file that puts genuinely unpredictable text
// (shell command text, shell OUTPUT — filenames, grep hits, /status dumps)
// into a JSON string field. Ported from cctv_app's json_escape() with one
// addition: if DST is too small and the copy has to stop mid-string, back off
// any UTF-8 continuation bytes already written, same rule as CopyUtf8() in
// app_config.h — half a character makes the whole line invalid UTF-8 and the
// JSON parse on the other end throws away the rest of the message with it
// (see the 2026-08-05 commit that added CopyUtf8 for exactly this failure).
void JsonEscapeShellText(const char* src, char* dst, size_t dst_size) {
  if (dst == NULL || dst_size == 0) return;
  if (src == NULL) { dst[0] = '\0'; return; }
  size_t out = 0, i = 0;
  while (src[i] != '\0') {
    const unsigned char c = (unsigned char)src[i];
    const size_t need = (c == '"' || c == '\\') ? 2 : 1;
    if (out + need + 1 > dst_size) break;  // +1 leaves room for the '\0'
    if (c == '"' || c == '\\') {
      dst[out++] = '\\';
      dst[out++] = (char)c;
    } else if (c >= 0x20) {
      dst[out++] = (char)c;
    } else {
      dst[out++] = ' ';
    }
    ++i;
  }
  if (src[i] != '\0') {  // stopped early: the cut may have landed mid-character
    while (out > 0 && ((unsigned char)dst[out - 1] & 0xC0) == 0x80) --out;
  }
  dst[out] = '\0';
}
#endif

#if ENABLE_STATUS_PAGE
/**
 * Fixed-capacity JSON accumulator for BuildStatusJson().
 *
 * A stack buffer rather than std::string/ostringstream on purpose: the whole
 * point of the status page is to be too cheap to bother switching off, and
 * this way the response is assembled with zero heap allocations (one copy at
 * the very end, into the string the SDK wants). Overflow truncates instead of
 * growing, and BuildStatusJson() turns that into an explicit error object
 * rather than shipping half of one.
 *
 * Budget (measured, 2026-08-05): ~7 KB for four lenses of detections plus the
 * calibration block's board probe, and ~3.3 KB more when all four marker lists
 * are full (24 each). ~10 KB of 16 KB. The per-point residuals still to come
 * add roughly 2 KB at the same fill, so this is no longer a buffer with a
 * comfortable margin — anything added here should be costed first.
 *
 * No JSON escaping anywhere: every string that reaches here is a compile-time
 * constant, an IP literal, or an ISO timestamp this file generated. Nothing
 * from the network is echoed back.
 */
struct JsonBuf {
  // 24 KB, raised from 16 KB on 2026-08-05 after the budget was actually
  // measured on .13 rather than estimated:
  //
  //   base, nothing detected, no markers registered    3,382 B
  //   four lenses x 24 registered markers             +3,611 B  (37.6 B each)
  //   four lenses x 24 detections                    +~10,000 B (~104 B each)
  //                                                  ----------
  //                                                  ~17,000 B
  //
  // which does not fit in 16 KB, before the per-point residuals (~2 KB at the
  // same fill) are added at all. The detection cap of 24 per lens is what makes
  // the top line possible; it was raised from 8 so that a 5x7 ChArUco board
  // would not be silently clipped, and four lenses can legitimately hit it.
  //
  // Still a stack local rather than a static: the file's threading note says
  // one thread, but a static buffer turns any future violation of that into
  // silently interleaved JSON, which is far harder to diagnose than the 8 KB
  // this costs.
  static const int kCapacity = 24576;

  char buf[kCapacity];
  int  len;

  JsonBuf() : len(0) { buf[0] = '\0'; }

  // True once a format has been cut short. Checked before the response goes
  // out: a truncated object is invalid JSON, and "parse error" in the browser
  // would say nothing about where it came from.
  bool truncated() const { return len >= (int)sizeof(buf) - 1; }

  void addf(const char* fmt, ...) __attribute__((format(printf, 2, 3))) {
    if (len >= (int)sizeof(buf) - 1) return;
    va_list ap;
    va_start(ap, fmt);
    const int w = vsnprintf(buf + len, sizeof(buf) - len, fmt, ap);
    va_end(ap);
    if (w > 0) {
      len += w;
      if (len > (int)sizeof(buf) - 1) len = (int)sizeof(buf) - 1;  // truncated
    }
  }
};

/**
 * The marker's centre and its heading reference, in full-frame pixels.
 *
 * ONE definition, because two places need it and they must agree exactly: the
 * collection session averages these points into the fit, and the pose packet
 * maps them to world millimetres. If the two ever disagree — someone switches
 * one to the marker-origin corner, say — every reported position carries a
 * fixed offset of half a marker while the residual table still reads perfect,
 * because the fit moved with it. That failure was previously prevented by a
 * comment at each site asking the next editor not to cause it.
 *
 * Centre is the mean of the four corners. Heading points at the midpoint of
 * the top edge (corners 0-1); the angle itself is measured in world space, not
 * here — see SendPosePackets.
 */
void MarkerPoints(const ArucoProcessor::Detection& d, float* cx, float* cy,
                  float* tx, float* ty) {
  *cx = (d.corners2d[0].x + d.corners2d[1].x + d.corners2d[2].x + d.corners2d[3].x) * 0.25f;
  *cy = (d.corners2d[0].y + d.corners2d[1].y + d.corners2d[2].y + d.corners2d[3].y) * 0.25f;
  if (tx) *tx = (d.corners2d[0].x + d.corners2d[1].x) * 0.5f;
  if (ty) *ty = (d.corners2d[0].y + d.corners2d[1].y) * 0.5f;
}

// Resident set size in KB, or 0 if unavailable.
//
// getrusage() is used for the CPU times below, but its ru_maxrss is the PEAK,
// which never falls — useless for watching a leak develop. Current RSS has to
// come from /proc. statm rather than status: the second field is resident
// pages, so it is one fscanf instead of a scan for a "VmRSS:" line.
long current_rss_kb() {
  FILE* f = fopen("/proc/self/statm", "r");
  if (f == NULL) return 0;
  unsigned long total_pages = 0, resident_pages = 0;
  const int got = fscanf(f, "%lu %lu", &total_pages, &resident_pages);
  fclose(f);
  if (got != 2) return 0;
  return (long)((resident_pages * (unsigned long)sysconf(_SC_PAGESIZE)) / 1024UL);
}

// What the C library says about the heap, as opposed to what the kernel says
// about resident pages.
//
// RSS alone cannot tell a leak from an allocator holding on: freed memory stays
// resident until the allocator decides to give it back, so "RSS is climbing"
// describes both. These two numbers separate them:
//
//   in_use   — bytes currently handed out to the program. Climbing means the
//              program is genuinely holding more. That is a leak.
//   free     — bytes the allocator owns but has not returned to the kernel.
//              Climbing while in_use is flat means fragmentation or arena
//              growth, and the program is fine.
//
// Measured on .13 (2026-08-05): RSS rose ~1.1 KB per detected frame with
// detection on and was FLAT with it off, over 30+ minutes with no plateau. That
// isolated it to the detection path but could not say which of the two it was —
// which is what this exists to answer.
void heap_bytes(unsigned long* in_use, unsigned long* freed) {
  struct mallinfo2 mi = mallinfo2();
  if (in_use) *in_use = (unsigned long)mi.uordblks;
  if (freed) *freed = (unsigned long)mi.fordblks;
}
#endif  // ENABLE_STATUS_PAGE
}

SampleComponent::SampleComponent() : SampleComponent(_SampleComponent_Id, "SampleComponent") {}

SampleComponent::SampleComponent(ClassID id, const char* name) : Component(id, name) {
  // Outside the ENABLE_STATUS_PAGE guard: the frame path increments these
  // whether or not the page exists, so leaving them uninitialised in a build
  // without the page would be reading indeterminate values.
  raw_events_ = 0;
  raw_no_image_ = 0;
#if ENABLE_STATUS_PAGE
  start_ms_ = epoch_ms();
  cpu_sample_cpu_s_ = 0.0;
  core_n_ = 0;              // 0 = /proc/stat not read yet; first call reports -1
  for (int i = 0; i < kMaxCores; ++i) {
    core_busy_[i] = core_total_[i] = 0;
    core_pct_[i] = -1.0;
  }
  // 0 = no baseline yet, so every rate reports -1 until a full window has run.
  stats_sample_ms_ = 0;
  last_trim_ms_ = 0;
  cpu_report_ms_ = 0;
  heap_probe_seq_ = 0;
  heap_probe_n_ = 0;
  heap_detect_bytes_ = 0;
  heap_rest_bytes_ = 0;
  stats_window_s_ = 0.0;
  cpu_pct_ = -1.0;
  for (int c = 0; c < kMaxChannels; ++c) in_fps_[c] = -1.0;
#endif
  // Keep glibc from opening a heap arena per thread.
  //
  // OpenCV's detector runs work through parallel_for_, and glibc gives each
  // thread that allocates its own arena — up to 8 x cores of them. Every arena
  // keeps its own free lists and its own trailing slack, so a workload that
  // allocates and frees the same amount forever still grows its resident set as
  // the arenas multiply and fragment. That is the shape measured here: RSS
  // climbing ~1.1 KB per detected frame while the program's own live data did
  // not change.
  //
  // 2 rather than 1 because this app really does allocate from more than one
  // thread and a single arena would serialise them on a lock; the cost of that
  // lands on the one scheduler thread everything already shares.
  //
  // Costs nothing to be wrong about: if arenas were never the problem, this
  // changes nothing except a small amount of allocator contention.
  mallopt(M_ARENA_MAX, 2);

  detect_duty_pct_ = DETECT_DUTY_PCT;  // the compiled value is the starting one
  marker_hold_ms_ = MARKER_HOLD_MS;    // ditto — see HOLD_MS
  for (int c = 0; c < kMaxChannels; ++c) {
    detect_budget_until_ms_[c] = 0;
    last_delivery_ms_[c] = 0;
    detect_skipped_[c] = 0;
    last_queue_ms_[c] = -1;  // -1 = not measured yet
    recent_head_[c] = 0;
    recent_n_[c] = 0;
    recent_skipped_[c] = 0;
    memset(recent_skip_[c], 0, sizeof(recent_skip_[c]));
    pts_offset_min_[c] = LONG_MAX;
    delivered_[c] = 0;
    delivered_prev_[c] = 0;
  }
  governor_active_ = 0;  // 0 = no frame charged yet; /status reports null
  hg_map_.ch = -1;  // -1 = HG_MAP never run; /status reports null
  hg_map_.px = hg_map_.py = hg_map_.wx = hg_map_.wy = 0.0;
  hg_map_.ok = false;
  hg_map_.reason = "";
  anchor_cmd_.ch = -1;  // -1 = no ANCHOR_ command yet; /status reports null
  anchor_cmd_.n = 0;
  anchor_cmd_.ok = false;
  anchor_cmd_.reason[0] = '\0';
  zone_radius_div_ = ZONE_FOOT_RADIUS_DIV;
  det_stream_ = false;  // 오버레이가 켤 때만 (DET_STREAM)
  zone_danger_mm_ = ZONE_MARGIN_DANGER_MM;
  zone_warn_mm_ = ZONE_MARGIN_WARN_MM;
  zone_bands_on_ = false;  // H 있는 채널에서 켤 때만 (ZONE_BANDS)
  zone_alarm_level_ = 2;  // 접근금지 밴드 진입부터 경보 (ZONE_ALARM_LEVEL)
  iva_sync_.ch = -1;  // -1 = no IVA_SYNC yet; /status reports null
  iva_sync_.ok = false;
  iva_sync_.n = 0;
  iva_sync_.reason[0] = '\0';
  for (int c = 0; c < kMaxChannels; ++c) {
    proximity_guard_[c].Configure(PROXIMITY_CAUTION_ENTER_MM, PROXIMITY_CAUTION_EXIT_MM,
                                  PROXIMITY_DANGER_ENTER_MM, PROXIMITY_DANGER_EXIT_MM,
                                  PROXIMITY_MIN_DWELL_MS);
  }
}

/**
 * Record one offered frame in this lens's sliding window.
 *
 * Called on BOTH paths — the frame that got detected and the frame the
 * governor threw away — because the window measures duty, and duty is a ratio
 * against what the camera offered, not against what we chose to do.
 */
void SampleComponent::NoteFrame(int ch, bool skipped) {
  unsigned char& slot = recent_skip_[ch][recent_head_[ch]];
  if (recent_n_[ch] == kRecentWindow)
    recent_skipped_[ch] -= slot;  // the frame falling out of the window
  else
    ++recent_n_[ch];
  slot = skipped ? 1 : 0;
  recent_skipped_[ch] += slot;
  recent_head_[ch] = (recent_head_[ch] + 1) % kRecentWindow;
}

SampleComponent::~SampleComponent() { pose_sender_close(); }

bool SampleComponent::Initialize() {
  // Step markers: if the app dies during start-up, the last line printed tells
  // us exactly which stage aborted (the camera only exposes stdout, not a
  // backtrace).
  printf("[ArucoPosePNM] init: begin\n");
  fflush(stdout);

  RegisterURI();
  printf("[ArucoPosePNM] init: URI registered\n");
  fflush(stdout);

  // Neither of the two below may abort start-up: a detector or socket failure
  // should leave the app running (and reporting) rather than dying silently.
  try {
    // One detector per lens. The core is SDK-agnostic; it is fed that
    // channel's NV12 Y plane each frame.
    for (int c = 0; c < kMaxChannels; ++c) {
      aruco_[c].reset(new ArucoProcessor());
      seq_[c] = 0;
      dynroi_[c].setChannel(c);  // stamped into every DYNROI_STATE this lens sends
      // Off at boot on every lens (2026-08-10) — see the member's doc comment
      // in the header. An operator turns on the lens(es) actually in use
      // (POST /detect or "DETECT <ch> 1"); nothing here guesses which ones
      // those are.
      detect_enabled_[c] = false;
      search_scale_[c] = SEARCH_DOWNSCALE;
      memset(&odom_pending_[c], 0, sizeof(odom_pending_[c]));
      odom_session_deadline_ms_[c] = 0;
      odom_m_mm_[c] = 0.0;
      odom_n_mm_[c] = 0.0;
      for (int s = 0; s < kMaxRecentMarkers; ++s) recent_marker_[c][s].id = -1;
      for (int s = 0; s < kMaxRecentWiseAiObjects; ++s) {
        recent_wiseai_obj_[c][s].object_id[0] = '\0';
        recent_wiseai_obj_[c][s].zone_state = -1;
      }
      iva_zone_[c].n = 0;  // no zone until this channel's first IVA_SYNC
      iva_zone_[c].have_world = false;
#if ENABLE_STATUS_PAGE
      last_frame_ms_[c] = 0;  // 0 = no frame yet; /status reports age_ms -1
      last_markers_[c] = 0;
      last_w_[c] = 0;
      last_h_[c] = 0;
      last_dets_[c].clear();
      last_dets_approx_[c] = false;
#endif
    }
    printf("[ArucoPosePNM] init: %d ArucoProcessors ready\n", kMaxChannels);
  } catch (const std::exception& e) {
    printf("[ArucoPosePNM] init: ArucoProcessor FAILED: %s\n", e.what());
  } catch (...) {
    printf("[ArucoPosePNM] init: ArucoProcessor FAILED (unknown)\n");
  }
  fflush(stdout);

  // Loads each lens's persisted K/dist and works out whether PERSIST_DIR can
  // be written at all. Must not abort start-up either: a camera with no
  // calibration yet is the normal state on first install.
  try {
    calib_.Init();
    // AFTER calib_.Init(): it is the one that discovers where this app can
    // write, and homography_ reuses that answer rather than repeating the
    // search. See HomographyMapper::Init().
    homography_.Init(calib_);
  } catch (...) {
    printf("[ArucoPosePNM] init: calib FAILED\n");
  }
  fflush(stdout);

  // Persistent, non-blocking TCP to the RPi dashboard. Safe to call before the
  // server is up: pose_sender reconnects on its own.
  try {
    pose_sender_init(POSE_SERVER_IP, POSE_SERVER_PORT);
    printf("[ArucoPosePNM] init: pose stream -> %s:%d\n", POSE_SERVER_IP, POSE_SERVER_PORT);
  } catch (...) {
    printf("[ArucoPosePNM] init: pose_sender FAILED\n");
  }
  fflush(stdout);

#if ENABLE_CENTRAL_TLS_STREAM
  // Independent of pose_sender above: safe to call even when the CA file is
  // still the unconfigured placeholder -- init() fails closed (returns -1,
  // no socket ever opens) rather than throwing.
  try {
    if (central_tls_sender_init(CENTRAL_TLS_SERVER_IP, CENTRAL_TLS_SERVER_PORT,
                                CENTRAL_TLS_CA_FILE) == 0) {
      // The deployed/certificate-verified .8 address is primary. The wired .2
      // candidate remains available after its SAN is added to the certificate.
      central_tls_sender_set_fallback(CENTRAL_TLS_SERVER_IP_FALLBACK);
      printf("[ArucoPosePNM] init: central TLS -> %s:%d (fallback %s)\n",
             CENTRAL_TLS_SERVER_IP, CENTRAL_TLS_SERVER_PORT, CENTRAL_TLS_SERVER_IP_FALLBACK);
    } else {
      printf("[ArucoPosePNM] init: central TLS disabled (certificate/config error)\n");
    }
  } catch (...) {
    printf("[ArucoPosePNM] init: central_tls_sender FAILED\n");
  }
  fflush(stdout);
#endif

  bool ok = Component::Initialize();
  printf("[ArucoPosePNM] init: done (Component::Initialize=%d)\n", (int)ok);
  fflush(stdout);
  return ok;
}

bool SampleComponent::ProcessAEvent(Event* event) {
  switch (event->GetType()) {
    case (int32_t)IAppDispatcher::EEventType::eHttpRequest: {
      HandleHttpRequest(event);
      break;
    }
    case static_cast<int32_t>(IPOpenPlatformManager::EAppEventType::eNetworkSettingChanged): {
      std::cout << "Network setting is changed!" << std::endl;
      setting_changed_time_ = GetCurrentTimeToString();
      break;
    }
    case static_cast<int32_t>(IPStreamProviderManagerVideoRaw::EEventType::eVideoRawData): {
      ProcessRawVideo(event);
      break;
    }
    case static_cast<int32_t>(IPMetadataManager::EEventType::eMetadataRequest): {
      ProcessWiseAiMetadata(event);
      break;
    }
    default:
      Component::ProcessAEvent(event);
      break;
  }
  return true;
}

bool SampleComponent::HandleHttpRequest(Event* event) {
  if (event->IsReply()) {
  } else {
    auto* oas = reinterpret_cast<OpenAppSerializable*>(event->GetBaseObjectArgument());
    auto path_info = oas->GetFCGXParam("PATH_INFO");

    if (path_info == "/writeeventlog") {
      auto body = oas->GetRequestBody();
      JsonUtility::JsonDocument doc(JsonUtility::Type::kObjectType);
      doc.Parse(body);

      if (doc.HasParseError()) {
        oas->SetStatusCode(400);
        oas->SetResponseBody("request body parse error");
        return false;
      }

      if (doc.HasMember("log")) {
        std::string log_msg = doc["log"].GetString();
        auto* log = new Log(Log::LogType::EVENT_LOG, Log::LogDetailType::EVENT_OPENAPP, 0, time(NULL), String(log_msg));
        SendNoReplyEvent("LogManager", static_cast<int>(ILogManager::EEvent::eWrite), 0, log);
      }
#if ENABLE_STATUS_PAGE
    } else if (path_info == "/status") {
      // Read-only, so GET is the honest verb. The dispatcher is told the same
      // (RegisterURI registers GET alone), but it is checked here too: the
      // handler must not depend on the dispatcher having enforced it.
      if (oas->GetMethod() != "GET") {
        oas->SetStatusCode(405);
        oas->SetResponseBody("use GET");
        return false;
      }
      const std::string body = BuildStatusJson();
      oas->SetResponseBody(body.c_str(), body.size());

    } else if (path_info == "/cmd") {
      // POST only, and registered that way too. Unlike /status this CHANGES
      // something, and a GET that changes state can be fired by any page the
      // operator happens to visit while logged in — an <img src> is enough,
      // and the browser attaches the cached digest credentials for free.
      if (oas->GetMethod() != "POST") {
        oas->SetStatusCode(405);
        oas->SetResponseBody("use POST");
        return false;
      }

      // The body is ONE command line, in the same language the pose link
      // speaks ("DETECT 0 1", "SCALE 2 4", "DYNROI 1 120 8") — see
      // DispatchCommand. Mirroring the RPi dashboard's own /cmd endpoint, so a
      // command that works there works here unchanged.
      //
      // Not a shell: every handler parses with sscanf/strtol into its own
      // bounded arguments. Nothing here reaches a process, a file, or a path.
      // Too long is REFUSED, not trimmed.
      //
      // The old code resized to 480 and carried on. For every command that
      // existed then that was harmless — they are all a name and two or three
      // numbers, so an over-long body was garbage either way. ANCHOR_SET_ALL
      // broke the assumption: it is up to 24 hand-measured triples, ~620 bytes
      // at the worst plausible coordinates, and a body cut mid-number still
      // parses into a complete-looking marker. Silently calibrating a lens from
      // a list nobody sent is the exact failure this app keeps designing
      // against, so the limit now matches the pose link's 1023 and says so when
      // it is hit.
      std::string line = oas->GetRequestBody();
      if (line.size() >= POSE_SENDER_MAX_LINE) {
        char msg[64];
        snprintf(msg, sizeof(msg), "command line too long (max %d bytes)",
                 POSE_SENDER_MAX_LINE - 1);
        oas->SetStatusCode(413);
        oas->SetResponseBody(msg);
        return false;
      }

      if (!DispatchCommand(line.c_str())) {
        oas->SetStatusCode(400);
        oas->SetResponseBody("unknown command (DETECT|SCALE|DYNROI|DYNROI_CH|DYNROI_IDS|"
                             "CALIB_K_*|K_LOAD|HG_*|ANCHOR_*|IVA_SYNC|IVA_ZONE_SET|ZONE_*|DET_STREAM)");
        return false;
      }

      // Answer with the full status so the page repaints from one round trip
      // instead of commanding and then polling.
      const std::string after = BuildStatusJson();
      oas->SetResponseBody(after.c_str(), after.size());
#endif
    } else if (path_info == "/checksetting") {
      JsonUtility::JsonDocument doc(JsonUtility::Type::kObjectType);
      auto& alloc = doc.GetAllocator();
      doc.AddMember("latest_changed", setting_changed_time_, alloc);

      rapidjson::StringBuffer strbuf;
      rapidjson::Writer<rapidjson::StringBuffer> writer(strbuf);
      doc.Accept(writer);

      oas->SetResponseBody(strbuf.GetString(), strbuf.GetLength());
    }
  }
  return true;
}

void SampleComponent::RegisterURI() {
  printf("[SampleComponent] Register URI\n");

  Vector<String> methods;
  methods.push_back("GET");
  methods.push_back("POST");

  auto* write_uri = new ("OpenAPI") IAppDispatcher::OpenAPIRegistrar(String("/writeeventlog"), GetInstanceName(), methods);
  auto* check_uri = new ("OpenAPI") IAppDispatcher::OpenAPIRegistrar(String("/checksetting"), GetInstanceName(), methods);

  SendNoReplyEvent("AppDispatcher", static_cast<int32_t>(IAppDispatcher::EEventType::eRegisterCommand), 0, write_uri);
  SendNoReplyEvent("AppDispatcher", static_cast<int32_t>(IAppDispatcher::EEventType::eRegisterCommand), 0, check_uri);

#if ENABLE_STATUS_PAGE
  // GET only — the single-method constructor, not the Vector one the two above
  // use. /status has no side effects, so nothing else needs to reach it.
  auto* status_uri = new ("OpenAPI") IAppDispatcher::OpenAPIRegistrar(String("/status"), GetInstanceName(), String("GET"));
  SendNoReplyEvent("AppDispatcher", static_cast<int32_t>(IAppDispatcher::EEventType::eRegisterCommand), 0, status_uri);

  // POST only — /cmd changes state, see the handler.
  auto* cmd_uri = new ("OpenAPI") IAppDispatcher::OpenAPIRegistrar(String("/cmd"), GetInstanceName(), String("POST"));
  SendNoReplyEvent("AppDispatcher", static_cast<int32_t>(IAppDispatcher::EEventType::eRegisterCommand), 0, cmd_uri);
#endif
}

std::string SampleComponent::GetCurrentTimeToString() {
  auto now = std::chrono::system_clock::now();
  auto now_time_t = std::chrono::system_clock::to_time_t(now);
  auto now_tm = ::gmtime(&now_time_t);

  std::stringstream ss;
  ss << std::put_time(now_tm, "%FT%T");
  return ss.str();
}

#if ENABLE_STATUS_PAGE
/**
 * Everything the on-camera page shows, as one JSON object.
 *
 * Runs only when someone presses [Refresh] — there is no timer behind it — so
 * this is the entire cost of the dashboard, paid per click: two small file
 * reads, one syscall, and ~2 KB of formatting. For scale, one detect() call on
 * one channel is 5..9 ms.
 *
 * Called on the scheduler thread that also runs ProcessRawVideo(), so no frame
 * can be half-processed while this reads: no locking is needed, and adding any
 * would be the one way to make this page actually cost something.
 */
std::string SampleComponent::BuildStatusJson() {
  const long now_ms = epoch_ms();

  JsonBuf j;
  j.addf("{\"app\":{\"name\":\"ArucoPosePNM\",\"version\":\"%s\","
         // __DATE__/__TIME__ are the compile time of THIS file, in the build
         // container's clock. Reliable as a build marker only because
         // build_install.sh moves the build dir aside and rebuilds from
         // scratch every run — an incremental build that skipped this file
         // would leave a stale date here.
         "\"built\":\"%s %s\",\"uptime_s\":%ld,\"channels\":%d},",
         APP_VERSION, __DATE__, __TIME__, (now_ms - start_ms_) / 1000L, kMaxChannels);

  // --- process resources -----------------------------------------------
  struct rusage ru;
  double cpu_s = 0.0;
  long peak_rss_kb = 0;
  if (getrusage(RUSAGE_SELF, &ru) == 0) {
    cpu_s = (double)ru.ru_utime.tv_sec + (double)ru.ru_utime.tv_usec / 1e6 +
            (double)ru.ru_stime.tv_sec + (double)ru.ru_stime.tv_usec / 1e6;
    peak_rss_kb = (long)ru.ru_maxrss;  // KB on Linux
  }

  SampleRates(now_ms, cpu_s);

  // cores: cpu_pct is measured against wall clock, so it can exceed 100 only
  // if the app is actually allowed to run on more than one core. Reporting the
  // count next to it is what makes that number readable — and it answers
  // whether splitting the channels across schedulers could buy anything.
  // heap_in_use_kb next to rss_kb is what makes a rising RSS diagnosable:
  // in_use climbing is the program holding more (a leak), in_use flat while RSS
  // climbs is the allocator holding more (fragmentation). See heap_bytes().
  unsigned long heap_used = 0, heap_free = 0;
  heap_bytes(&heap_used, &heap_free);
  j.addf("\"proc\":{\"rss_kb\":%ld,\"peak_rss_kb\":%ld,\"heap_in_use_kb\":%lu,"
         "\"heap_free_kb\":%lu,\"cpu_s\":%.2f,"
         "\"cpu_pct\":%.1f,\"cpu_window_s\":%.1f,\"cores\":%ld,",
         current_rss_kb(), peak_rss_kb, heap_used / 1024UL, heap_free / 1024UL,
         cpu_s, cpu_pct_, stats_window_s_,
         (long)sysconf(_SC_NPROCESSORS_ONLN));
  // Bytes per probed frame, split. Whichever of the two keeps climbing is where
  // to look; the other one is exonerated.
  j.addf("\"heap_probe\":{\"n\":%lu,\"detect_b\":%lld,\"rest_b\":%lld},",
         heap_probe_n_, heap_detect_bytes_, heap_rest_bytes_);

  // Whole-camera load, one figure per core, measured in the window above. See
  // the member declaration for why this is not the same thing as cpu_pct.
  j.addf("\"core_pct\":[");
  for (int i = 0; i < core_n_; ++i) j.addf("%s%.1f", i ? "," : "", core_pct_[i]);
  j.addf("]},");

  // duty_pct is the RUNTIME value (DUTY <pct>), not the compiled default, and
  // the bounds travel with it so the page can build its input without knowing
  // them. default_pct is carried too: a tuned value that has drifted somewhere
  // unhelpful is only recognisable next to what it started as.
  // `active` is what the duty is being divided BY, so it is the difference
  // between "60% and my lens is slow" and "60% split four ways, which is 15%
  // each, and that is the number you are looking at". Without it the split is
  // guesswork from the outside.
  j.addf("\"governor\":{\"duty_pct\":%d,\"min\":%d,\"max\":%d,\"default_pct\":%d,"
         "\"active\":%d,\"share_pct\":%.1f},",
         detect_duty_pct_, kDutyMin, kDutyMax, DETECT_DUTY_PCT, governor_active_,
         governor_active_ > 0 ? (double)detect_duty_pct_ / governor_active_ : -1.0);

  // The flicker-hold window (HOLD_MS), same reporting shape as governor above
  // so the page can show a live value next to its bounds and boot default.
  j.addf("\"hold_ms\":{\"ms\":%ld,\"min\":%ld,\"max\":%ld,\"default_ms\":%d},",
         marker_hold_ms_, kHoldMsMin, kHoldMsMax, MARKER_HOLD_MS);

  // The raw group is a LITERAL and has to be kept in step with
  // SampleComponent_manifest_instance_0.json by hand; the manifest is JSON
  // read at load time, so there is nothing to share it with. It earns its keep
  // because a wrong group is silent: GroupSPMgrVideoRaw3 ("Full VideoRaw") was
  // tried on 2026-08-04 and delivered no frames at all — no error, no connect
  // event, just 0% CPU and four dead channels. Printing which group is
  // subscribed turns that into an obvious diagnosis.
  //
  // Retried on 2026-08-05 with Source=OpenPlatform instead of the four
  // per-sensor sources, which was the one difference an external write-up
  // claimed mattered. Same result: zero frames, and w/h stayed 0x0, so nothing
  // ever arrived to be measured. Two different source spellings on that group
  // now say the same thing — the group is very likely not built into this
  // camera's firmware (the SDK header marks it DEF_FULL_RAW, a compile-time
  // switch on the camera side, not something an app declares).
  //
  // cam_fps used to be the literal 5. It was written when 5 fps had just been
  // measured, and it was honest then — but a constant cannot go stale visibly,
  // so it kept asserting 5 through every experiment that might have changed it,
  // including two that changed the raw group outright. An external review
  // (docs/CCTV_4CH_LATEST_REVIEW.md 3.1) reached for it as evidence, which is
  // exactly what a hardcoded measurement invites. It is now the real delivery
  // rate, differenced from delivered_[] over the shared window above, and -1
  // until that window has run once.
  // raw_group is DECLARED, not observed: it mirrors the manifest, which this
  // app cannot read at runtime. Keep it in sync with
  // manifests/SampleComponent_manifest_instance_0.json by hand — there is no
  // second source of truth to check it against, which is exactly why it says
  // what it is rather than pretending to be a measurement.
  //
  // Per channel rather than one string because it stopped being one value the
  // moment ch0 was moved to another group to test it (2026-08-05), and a
  // single shared string then described three lenses correctly and lied about
  // the fourth — the one being looked at. All four are back on Raw2 now; the
  // shape stays so the next such test is one edit, not a refactor.
  // ONE place to edit when a lens is moved to another group, so the label and
  // the manifest cannot drift in two files at once.
  static const char* const kRawGroup[kMaxChannels] = {
      "GroupSPMgrVideoRaw2", "GroupSPMgrVideoRaw2",
      "GroupSPMgrVideoRaw2", "GroupSPMgrVideoRaw2"};
  j.addf("\"video\":{\"raw_group\":[");
  for (int c = 0; c < kMaxChannels; ++c) j.addf("%s\"%s\"", c ? "," : "", kRawGroup[c]);
  // raw_events / raw_no_image are the pair that tells "no event arrived" apart
  // from "an event arrived and yielded nothing". Cumulative since start-up,
  // not a rate: the question they answer is whether the number is zero.
  j.addf("],\"raw_events\":%lu,\"raw_no_image\":%lu,\"in_fps\":[",
         raw_events_, raw_no_image_);
  for (int c = 0; c < kMaxChannels; ++c) j.addf("%s%.2f", c ? "," : "", in_fps_[c]);
  j.addf("],\"in_window_s\":%.1f},", stats_window_s_);

  // --- pose link --------------------------------------------------------
  unsigned long sent = 0, dropped = 0;
  int queued = 0;
  pose_sender_get_stats(&sent, &dropped, &queued);
  j.addf("\"pose\":{\"server\":\"%s\",\"port\":%d,\"connected\":%s,"
         "\"sent\":%lu,\"dropped\":%lu,\"queued\":%d},",
         POSE_SERVER_IP, POSE_SERVER_PORT, pose_sender_is_connected() ? "true" : "false",
         sent, dropped, queued);

#if ENABLE_CENTRAL_TLS_STREAM
  // --- central server link -----------------------------------------------
  // "active_ip" is which of CENTRAL_TLS_SERVER_IP / _IP_FALLBACK the link is
  // actually on right now (or would dial next) -- with two candidates,
  // "server" alone (always the primary) can't tell you that.
  j.addf("\"central\":{\"server\":\"%s\",\"active_ip\":\"%s\",\"port\":%d,\"link\":\"%s\","
         "\"enabled\":%s,\"pos_enabled\":%s,\"marker_id\":%d},",
         CENTRAL_TLS_SERVER_IP, central_tls_sender_active_ip(), CENTRAL_TLS_SERVER_PORT,
         central_tls_sender_state(), central_tls_sender_enabled() ? "true" : "false",
         g_central_pos_enabled ? "true" : "false", g_central_marker_id);
#endif

  // Absorbed from /checksetting so the page needs one request, not two.
  if (setting_changed_time_.empty())
    j.addf("\"network\":{\"latest_changed\":null},");
  else
    j.addf("\"network\":{\"latest_changed\":\"%s\"},", setting_changed_time_.c_str());

  // --- per channel ------------------------------------------------------
  j.addf("\"channels\":[");
  for (int c = 0; c < kMaxChannels; ++c) {
    const cv::Rect roi = aruco_[c] ? aruco_[c]->roi() : cv::Rect();
    const cv::Rect& man = manual_roi_[c];

    // frames = frames this lens actually DETECTED (the CAM_POSE "seq"), and it
    // stays cumulative because it IS the packet sequence number — the page
    // needs it to match what the server sees. The duty ratio the page actually
    // reads comes from `recent` instead: n frames offered in the window, of
    // which `skipped` were dropped by the governor. Cumulative `skipped` is
    // still here for the long view, but nothing on the page divides by it.
    // in_n is raw frames DELIVERED by the SDK; frames/skipped are what this app
    // did with them. Delivered counts even while detection is off, which is the
    // whole point — see ProcessRawVideo.
    j.addf("%s{\"ch\":%d,\"detect\":%s,\"scale\":%d,\"in_n\":%lu,\"frames\":%lu,\"skipped\":%lu,"
           "\"recent\":{\"n\":%d,\"skipped\":%d},"
           "\"queue_ms\":%ld,\"age_ms\":%ld,\"w\":%d,\"h\":%d,"
           "\"det_ms\":%.2f,\"markers\":%d,\"calibrating\":%s,"
           // Milliseconds of governor-enforced idle still owed on this lens.
           // The channel table's "대기" column is queue_ms — how stale the frame
           // was — which is a different thing that happens to share the word,
           // and neither of them answered "is the governor holding this lens
           // back right now, and for how much longer". Negative means the lens
           // is free to detect on the next frame that arrives.
           "\"budget_ms\":%ld,",
           // "ch" is 1-based here (c+1) — the human-facing lens number,
           // matching how the dashboard already labels lenses everywhere
           // else. Every array index below stays c (0-based); only the
           // reported field shifts. index.html's getStatus() undoes this
           // shift immediately after fetch, before any of its ~40 call
           // sites that assume a 0-based ch (command arguments, dataset.ch,
           // matching against other 0-based selects) ever see the value —
           // see the comment there for why the boundary is one line instead
           // of every call site.
           (c == 0) ? "" : ",", c + 1, detect_enabled_[c] ? "true" : "false", search_scale_[c],
           delivered_[c], seq_[c], detect_skipped_[c], recent_n_[c], recent_skipped_[c],
           last_queue_ms_[c],
           last_frame_ms_[c] ? (now_ms - last_frame_ms_[c]) : -1L,
           last_w_[c], last_h_[c],
           aruco_[c] ? aruco_[c]->lastDetectMs() : -1.0, last_markers_[c],
           calib_.Collecting(c) ? "true" : "false",
           // A lens that has never been charged has a deadline of 0, and 0
           // minus an epoch timestamp is -1.8e12 — arithmetically "free to
           // detect", but it renders as a number nobody can read as that.
           // Report the same 0 the never-charged and the just-expired case
           // both mean: nothing owed.
           detect_budget_until_ms_[c] == 0
               ? 0L
               : detect_budget_until_ms_[c] - now_ms);

    // roi is what detect() actually scanned last frame (tracker or manual);
    // manual_roi is the operator's box that SEARCH falls back to. An empty
    // rect means full frame in both cases.
    j.addf("\"roi\":{\"x\":%d,\"y\":%d,\"w\":%d,\"h\":%d},"
           "\"manual_roi\":{\"x\":%d,\"y\":%d,\"w\":%d,\"h\":%d},",
           roi.x, roi.y, roi.width, roi.height, man.x, man.y, man.width, man.height);

    if (aruco_[c]) {
      j.addf("\"tune\":{\"perim\":%.4f,\"ecc\":%.2f,\"thresh\":%.1f,\"poly\":%.4f,"
             "\"scan_passes\":%d,\"scan_win\":%d},",
             aruco_[c]->minPerimRate(), aruco_[c]->errCorrRate(), aruco_[c]->adaptiveThreshC(),
             aruco_[c]->polyAccuracyRate(), aruco_[c]->scanPasses(), aruco_[c]->scanWin());
    } else {
      j.addf("\"tune\":null,");
    }

    j.addf("\"dynroi\":{\"enabled\":%s,\"tracking\":%s,\"margin\":%d,\"max_miss\":%d,"
           "\"track_ids\":[",
           dynroi_[c].enabled() ? "true" : "false", dynroi_[c].tracking() ? "true" : "false",
           dynroi_[c].margin(), dynroi_[c].maxMiss());
    const std::vector<int>& ids = dynroi_[c].trackIds();
    for (size_t i = 0; i < ids.size(); ++i) j.addf("%s%d", (i == 0) ? "" : ",", ids[i]);
    j.addf("]},");

    // Per-marker corners for the browser overlay (index.html "채널 오버레이").
    // Full-frame pixel coordinates, same space as the pose packets.
    //
    // Capped at 24, raised from 8 on 2026-08-04: "a handful of markers per
    // lens" turned out to be wrong the moment a ChArUco board came into view —
    // a 5x7 board is 17 markers on its own, so the overlay was silently
    // dropping nine of them. 24 covers that board with room to spare; the
    // truncation guard below is what catches it if some scene ever exceeds the
    // buffer anyway.
    j.addf("\"dets\":[");
    const std::vector<ArucoProcessor::Detection>& lastDets = last_dets_[c];
    const size_t nDets = lastDets.size() < 24 ? lastDets.size() : 24;
    for (size_t i = 0; i < nDets; ++i) {
      const ArucoProcessor::Detection& d = lastDets[i];
      if (d.corners2d.size() < 4) continue;
      j.addf("%s{\"id\":%d,\"approx\":%s,\"c\":[[%.2f,%.2f],[%.2f,%.2f],[%.2f,%.2f],[%.2f,%.2f]]}",
             (i == 0) ? "" : ",", d.id, last_dets_approx_[c] ? "true" : "false",
             d.corners2d[0].x, d.corners2d[0].y, d.corners2d[1].x, d.corners2d[1].y,
             d.corners2d[2].x, d.corners2d[2].y, d.corners2d[3].x, d.corners2d[3].y);
    }
    j.addf("]}");
  }
  j.addf("],");

  AppendCalibJson(&j, now_ms);
  j.addf("}");

  // Truncation returns an ERROR OBJECT, not a truncated one.
  //
  // A cut JSON object is not parseable, so the page would show "parse error"
  // and nothing about where it came from — and the field that got cut is by
  // definition the one that is missing from the evidence. Answering with a
  // small valid object instead means the failure names itself, and the length
  // says how far over the limit it went.
  //
  // This stopped being a can't-happen assertion on 2026-08-05: see the measured
  // budget on JsonBuf. The capacity now exceeds the worst case, but only by
  // about a quarter, so this path is a real guard rather than an assertion.
  if (j.truncated()) {
    char err[96];
    snprintf(err, sizeof(err),
             "{\"error\":\"status buffer too small\",\"capacity\":%d}", JsonBuf::kCapacity);
    printf("[ArucoPosePNM] /status truncated at %d bytes — grow JsonBuf (or trim "
           "the marker/residual blocks)\n", j.len);
    fflush(stdout);
    return std::string(err);
  }

  return std::string(j.buf, (size_t)j.len);
}

/**
 * Roll every rate on the page, at most once per kStatsWindowMs.
 *
 * Split out of BuildStatusJson() because it is measurement, not formatting: it
 * reads /proc, differences three unrelated counter families and writes six
 * members, and none of that is easier to follow wedged into the top of a JSON
 * builder whose remaining 200 lines only format cached results. The one
 * function that has to stay auditable against the JsonBuf budget should not
 * also own file I/O.
 *
 * Runs on the scheduler thread, like everything else here.
 */
void SampleComponent::SampleRates(long now_ms, double cpu_s) {
  // Every rate on this page is measured here, over ONE window that rolls at
  // most once a second, and cached until it rolls again. See the declaration of
  // stats_sample_ms_ for why the window is not simply the gap between two
  // requests: it used to be, and a second client polling a few tens of
  // milliseconds behind the first made a healthy lens read 0.0 fps.
  //
  // Rates are cumulative-counter differences, so all of them are -1 until a
  // baseline exists AND a window has elapsed over it. Reporting a number early
  // would mean reporting one measured over a few milliseconds, which is the
  // thing this block exists to stop.
  const bool have_baseline = (stats_sample_ms_ != 0);
  const long stats_dt_ms = have_baseline ? (now_ms - stats_sample_ms_) : 0;
  if (!have_baseline || stats_dt_ms >= kStatsWindowMs) {
  // have_baseline already implies stats_dt_ms >= kStatsWindowMs here.
  const bool measurable = have_baseline;

  // This app against the wall clock. A cumulative average since start-up
  // would be dominated by whatever the app did minutes ago and would barely
  // move when something goes wrong now — the opposite of what this is for.
  cpu_pct_ = measurable
                 ? (cpu_s - cpu_sample_cpu_s_) * 1000.0 / (double)stats_dt_ms * 100.0
                 : -1.0;
  cpu_sample_cpu_s_ = cpu_s;

  // Raw frames the SDK delivered, per lens. This is the number that answers
  // "how fast does the camera actually feed us" — see ProcessRawVideo for why
  // frames+skipped could not.
  for (int c = 0; c < kMaxChannels; ++c) {
    in_fps_[c] = measurable ? (double)(delivered_[c] - delivered_prev_[c]) * 1000.0 /
                                  (double)stats_dt_ms
                            : -1.0;
    delivered_prev_[c] = delivered_[c];
  }

  // Whole-camera load, one figure per core. Read here rather than in a helper
  // because /proc/stat is cumulative, so the reading and its differencing
  // belong in one place — and the "nothing to compare against yet" case (-1)
  // then sits next to the same case for cpu_pct.
  int n = 0;
  FILE* f = fopen("/proc/stat", "r");
  if (f != NULL) {
    char line[256];
    while (n < kMaxCores && fgets(line, sizeof(line), f) != NULL) {
      int idx = -1;
      unsigned long long v[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
      // "cpuN ..." only. The bare "cpu " aggregate line is skipped: it is the
      // sum, and the sum of per-core percentages is not a percentage of
      // anything a person can act on.
      if (sscanf(line, "cpu%d %llu %llu %llu %llu %llu %llu %llu %llu %llu %llu",
                 &idx, &v[0], &v[1], &v[2], &v[3], &v[4], &v[5], &v[6], &v[7], &v[8],
                 &v[9]) < 5)
        continue;
      if (idx != n) continue;  // out of order or non-contiguous: stop trusting it

      unsigned long long total = 0;
      for (int k = 0; k < 10; ++k) total += v[k];
      // Busy = everything that is not idle. iowait (v[4]) counts as idle: the
      // core is available, it is a disk that is not. Treating it as busy would
      // make an SD-card write look like computation.
      const unsigned long long busy = total - (v[3] + v[4]);

      core_pct_[n] = -1.0;
      if (core_n_ > n && total > core_total_[n]) {
        const unsigned long long dt = total - core_total_[n];
        const unsigned long long db = busy - core_busy_[n];
        core_pct_[n] = (double)db * 100.0 / (double)dt;
      }
      core_busy_[n] = busy;
      core_total_[n] = total;
      ++n;
    }
    fclose(f);
  }
  core_n_ = n;

  // Hand the allocator's spare pages back to the kernel, occasionally.
  //
  // free() does not shrink the resident set; the allocator keeps the pages in
  // case they are wanted again, which is the right default for a process nobody
  // is watching. This one IS watched, on a device with no swap, and the pages
  // it holds are ones it demonstrably stops needing — a detect allocates and
  // releases the same working set every frame.
  //
  // Rate-limited to kTrimIntervalMs because the call walks the heap's free
  // lists. At ~30 s against a 250 ms detect it is far below noise, and doing it
  // per window (1 s) would put it in the same order as real work for no gain:
  // the pages it would return are ones the next frame asks for again.
  //
  // Deliberately NOT a fix for a leak — malloc_trim cannot return memory that
  // is still allocated. If heap_in_use keeps climbing in /status, this changes
  // nothing and the cause is in the code, not the allocator.
  if (now_ms - last_trim_ms_ >= kTrimIntervalMs) {
    malloc_trim(0);
    last_trim_ms_ = now_ms;
  }

  stats_window_s_ = (double)stats_dt_ms / 1000.0;
  stats_sample_ms_ = now_ms;
}
}

/**
 * The "calib" block of /status.
 *
 * Split out because it is the one part with a shape of its own: a shared board
 * and a shared session on one side, four independent results on the other. The
 * void* is so sample_component.h does not have to see JsonBuf, which is a
 * detail of this file.
 */
void SampleComponent::AppendCalibJson(void* jsonbuf, long now_ms) {
  JsonBuf& j = *reinterpret_cast<JsonBuf*>(jsonbuf);
  const CharucoBoardConfig b = calib_.Board();

  j.addf("\"calib\":{");
  j.addf("\"persist\":{\"ok\":%s,\"dir\":\"%s\",\"cwd\":\"%s\",",
         calib_.Persistable() ? "true" : "false", calib_.PersistDir(), calib_.Cwd());

  // What is ACTUALLY on disk, listed rather than assumed.
  //
  // Two questions this answers that nothing else can. First, "did my save
  // land" — the save button reports the return value of fopen/fprintf, which
  // is not the same as a file existing with bytes in it. Second, "is this app
  // writing anything I did not ask for" — an app that quietly accumulates
  // files in persistent storage is a slow leak nobody notices until the
  // partition is full, and there is no shell on this camera to go and look.
  //
  // Cheap enough to leave on: one readdir of a directory that holds a handful
  // of small text files, only when someone requests /status.
  j.addf("\"files\":[");
  {
    DIR* d = opendir(calib_.PersistDir());
    int n = 0;
    if (d) {
      struct dirent* e;
      while ((e = readdir(d)) != NULL && n < 24) {
        if (e->d_name[0] == '.') continue;  // . .. and the write probe
        char path[512];
        snprintf(path, sizeof(path), "%s/%s", calib_.PersistDir(), e->d_name);
        struct stat st;
        if (stat(path, &st) != 0) continue;
        j.addf("%s{\"name\":\"%s\",\"bytes\":%ld,\"age_s\":%ld}", n ? "," : "",
               e->d_name, (long)st.st_size, (long)(now_ms / 1000 - (long)st.st_mtime));
        ++n;
      }
      closedir(d);
    }
  }
  j.addf("],");

  // Where else this app could have written, and whether any of it is outside
  // the app directory.
  //
  // This exists to answer one question that costs a calibration when it is
  // guessed wrong: deleting the app with KeepOldSettings unchecked removes the
  // app directory, and every writable location found so far is inside it. If
  // the sandbox lets us write anywhere else, the answer is here; if it does
  // not, that is worth knowing once rather than being rediscovered by losing
  // something. The camera refuses SSH, so /status is the only window there is.
  //
  // `selectable:false` entries are probed and reported but never used —
  // relocating storage on the strength of a probe would orphan the files
  // already written somewhere the next boot would not look.
  j.addf("\"candidates\":[");
  for (int i = 0; i < calib_.CandidateCount(); ++i) {
    const IntrinsicsCalib::PersistCandidate& c = calib_.Candidate(i);
    j.addf("%s{\"path\":\"%s\",\"exists\":%s,\"writable\":%s,\"selectable\":%s,"
           "\"chosen\":%s}",
           i ? "," : "", c.path, c.exists ? "true" : "false",
           c.writable ? "true" : "false", c.selectable ? "true" : "false",
           c.chosen ? "true" : "false");
  }
  j.addf("]},");
  j.addf("\"board\":{\"sx\":%d,\"sy\":%d,\"square_mm\":%.2f,\"marker_mm\":%.2f,"
         "\"dict\":%d,\"margin_x_mm\":%.2f,\"margin_y_mm\":%.2f,\"corners\":%d},",
         b.squares_x, b.squares_y, b.square_length_mm, b.marker_length_mm,
         b.dictionary_id, b.outer_margin_x_mm, b.outer_margin_y_mm,
         (b.squares_x - 1) * (b.squares_y - 1));

  // One session block per lens, always four. `sessions` is an array now, not a
  // single object with a `ch` field: with several open at once there is no
  // "the" session, and a page that reads one would show whichever lens the
  // camera happened to mention.
  //
  // Shared policy (target views, RMS limit, gates) sits outside the array — one
  // board, one operator, one standard — so that four copies cannot drift.
  j.addf("\"session_policy\":{\"target\":%d,\"rms_limit\":%.3f,\"gates\":%s},",
         calib_.TargetViews(), calib_.RmsLimit(), calib_.Gates() ? "true" : "false");

  // Whatever the last CALIB_K_* / K_LOAD command had to say.
  //
  // IntrinsicsCalib has kept this string since the port and NOTHING EVER READ
  // IT (found 2026-08-05). Every message written through NoteMessage() — the
  // board config rejection, "이 렌즈의 K/dist를 지웠습니다", the revert
  // confirmation, the K_LOAD validation failure, a refused session — went into
  // a buffer with no reader, so roughly ten commands answered a press with
  // nothing at all. /cmd replies with this object and nothing else, so a
  // command that was refused and one that worked looked identical: the form
  // simply snapped back on the next poll.
  //
  // Not per session, because the reason it exists is the commands that have no
  // session — a board setting, a load, a refusal to open one in the first
  // place. Per-lens capture quality already has its own `reason` below.
  //
  // No JSON escaping for the same reason anchor_cmd.reason needs none: every
  // string that reaches here is a literal in this source or a number this code
  // printf'd, so neither a quote nor a backslash can appear.
  j.addf("\"message\":\"%s\",", calib_.FailReason());

  j.addf("\"sessions\":[");
  for (int c = 0; c < kMaxChannels; ++c) {
    const CalibViewQuality& q = calib_.LastQuality(c);
    j.addf("%s{\"ch\":%d,\"state\":%d,\"last_capture\":%d,\"collecting\":%s,"
           "\"pending\":%s,\"views\":%d,\"pruned\":%d,\"rms\":%.4f,"
           "\"probe_age_ms\":%ld,",
           (c == 0) ? "" : ",", c, (int)calib_.State(c), (int)calib_.LastCapture(c),
           calib_.Collecting(c) ? "true" : "false",
           calib_.CapturePending(c) ? "true" : "false", calib_.Views(c),
           calib_.PrunedViews(c), calib_.Rms(c), calib_.ProbeAgeMs(c, now_ms));
    // The rejection reason is the single most useful field on this page — a
    // capture that does nothing is otherwise indistinguishable from one that
    // was never received. Per lens, because one board pose gets four verdicts.
    j.addf("\"corners\":%d,\"corners_total\":%d,\"coverage\":%.4f,"
           "\"sharpness\":%.1f,\"move_px\":%.1f,\"reason\":\"%s\",",
           q.corners_found, q.corners_total, q.coverage_ratio, q.sharpness,
           q.mean_move_px, q.reason ? q.reason : "");

    // Where the board is in THIS lens right now, so the page can draw it.
    // Capped: a 7x5 board has 24 interior corners, and anything claiming far
    // more is not a board.
    j.addf("\"probe\":[");
    const std::vector<cv::Point2f>& pc = calib_.ProbeCorners(c);
    const std::vector<int>& pid = calib_.ProbeIds(c);
    const size_t np = pc.size() < 96 ? pc.size() : 96;
    for (size_t i = 0; i < np; ++i)
      j.addf("%s[%.1f,%.1f,%d]", (i == 0) ? "" : ",", pc[i].x, pc[i].y,
             (i < pid.size()) ? pid[i] : -1);
    j.addf("],");

    // Does the board in front of THIS lens actually have the configured shape?
    //
    // Two numbers, not a verdict, because the page has to be able to show the
    // operator why: sx by sy against sy by sx, both fitted to the marker
    // centres. There are two physical boards in this project with transposed
    // layouts, and every count on this page is identical for the pair — 17
    // markers, 24 corners — so without this the only way to tell which one is
    // in view is to have been there when it was put down.
    j.addf("\"board_fit\":{\"rms\":%.2f,\"rms_transposed\":%.2f}}",
           calib_.BoardFitRms(c), calib_.BoardFitRmsTransposed(c));
  }
  j.addf("],");

  j.addf("\"lenses\":[");
  for (int c = 0; c < kMaxChannels; ++c) {
    double fx, fy, cx, cy, d[5];
    if (calib_.Get(c, &fx, &fy, &cx, &cy, d)) {
      j.addf("%s{\"ch\":%d,\"have\":true,\"fx\":%.4f,\"fy\":%.4f,\"cx\":%.4f,\"cy\":%.4f,"
             "\"dist\":[%.8f,%.8f,%.8f,%.8f,%.8f]",
             (c == 0) ? "" : ",", c, fx, fy, cx, cy, d[0], d[1], d[2], d[3], d[4]);
    } else {
      j.addf("%s{\"ch\":%d,\"have\":false", (c == 0) ? "" : ",", c);
    }
    // The value this one replaced, if there is one. Carried in full rather than
    // as a bare "you can revert": the only way to decide whether to step back
    // is to see what you would be stepping back TO, and a revert taken blind is
    // how a good calibration gets swapped out for the bad one twice.
    double pfx, pfy, pcx, pcy, pd[5];
    if (calib_.GetPrevious(c, &pfx, &pfy, &pcx, &pcy, pd)) {
      j.addf(",\"prev\":{\"fx\":%.4f,\"fy\":%.4f,\"cx\":%.4f,\"cy\":%.4f,"
             "\"dist\":[%.8f,%.8f,%.8f,%.8f,%.8f]}",
             pfx, pfy, pcx, pcy, pd[0], pd[1], pd[2], pd[3], pd[4]);
    } else {
      j.addf(",\"prev\":null");
    }
    j.addf("}");
  }
  j.addf("],");

  // One entry per lens, always four, `have:false` rather than absent. The page
  // draws a row per lens either way, and "this lens has no H" is a state worth
  // showing — a missing key would render as an empty row that looks like a
  // rendering bug rather than a calibration that was never done.
  //
  // %.9e per entry for the reason given in ReportHomography(): H is defined
  // only up to scale, so one matrix spans a wide exponent range.
  j.addf("\"homography\":[");
  for (int c = 0; c < kMaxChannels; ++c) {
    double h[9];
    if (homography_.Get(c, h)) {
      // k_stale: this lens's K/dist was replaced after this matrix was fitted.
      // Reported next to the matrix rather than in a corner of its own, because
      // it is a property OF the matrix — see HomographyMapper::CalibStale().
      j.addf("%s{\"ch\":%d,\"have\":true,\"undistorted\":%s,\"mode_undistorted\":%s,"
             "\"mappable\":%s,\"k_stale\":%s,\"camera_z_mm\":%.1f,"
             "\"H\":[%.9e,%.9e,%.9e,%.9e,%.9e,%.9e,%.9e,%.9e,%.9e],",
             (c == 0) ? "" : ",", c,
             homography_.FittedUndistorted(c) ? "true" : "false",
             homography_.CoordModeUndistorted(c) ? "true" : "false",
             homography_.Mappable(c) ? "true" : "false",
             homography_.CalibStale(c) ? "true" : "false",
             homography_.CameraHeightMm(c),
             h[0], h[1], h[2], h[3], h[4], h[5], h[6], h[7], h[8]);
    } else {
      // The mode still matters with no matrix: it is what the next fit will
      // use, and it is the thing HG_COORD_MODE has to be able to show changing.
      j.addf("%s{\"ch\":%d,\"have\":false,\"mode_undistorted\":%s,",
             (c == 0) ? "" : ",", c,
             homography_.CoordModeUndistorted(c) ? "true" : "false");
    }

    // The registered markers travel with the lens they belong to rather than in
    // an array of their own, so a page cannot pair a matrix with the wrong
    // list. They are reported whether or not a matrix exists — the list is the
    // INPUT, and the state worth seeing most is "markers registered, nothing
    // fitted from them yet".
    //
    // %.1f, not %.9e: these are tape measurements. The wide format is for H,
    // whose entries span orders of magnitude within one matrix; using it here
    // would triple the biggest block in this object for no readable precision.
    // At four lenses and 24 markers this array is ~3.3 KB of the 16 KB buffer,
    // which is why the truncation guard below is now load-bearing rather than
    // an assertion.
    j.addf("\"anchors\":[");
    const int na = homography_.AnchorCount(c);
    for (int i = 0; i < na; ++i) {
      Anchor a;
      if (!homography_.AnchorAt(c, i, &a)) break;
      j.addf("%s{\"id\":%d,\"wx\":%.1f,\"wy\":%.1f}", (i == 0) ? "" : ",", a.id,
             a.wx_mm, a.wy_mm);
    }
    j.addf("],");

    // Collection session. `good/target` is the bar; the three id lists are what
    // make the bar readable when it is not moving. See FitProgress — a session
    // stuck at 0% because one marker is covered looks exactly like a session
    // stuck at 0% because the lens is pointed the wrong way, and only
    // missing_ids tells them apart.
    const HomographyMapper::FitProgress& fp = homography_.Fit(c);
    j.addf("\"session\":{\"collecting\":%s,\"good\":%d,\"target\":%d,\"total\":%d,"
           "\"max\":%d,\"result\":\"%s\",",
           fp.active ? "true" : "false", fp.good, HomographyMapper::kFitTargetFrames,
           fp.total, HomographyMapper::kFitMaxFrames,
           fp.last_result);
    j.addf("\"seen_ids\":[");
    for (int i = 0; i < fp.seen_n; ++i) j.addf("%s%d", i ? "," : "", fp.seen_ids[i]);
    j.addf("],\"missing_ids\":[");
    for (int i = 0; i < fp.missing_n; ++i) j.addf("%s%d", i ? "," : "", fp.missing_ids[i]);
    j.addf("],\"unusable_ids\":[");
    for (int i = 0; i < fp.unusable_n; ++i) j.addf("%s%d", i ? "," : "", fp.unusable_ids[i]);
    j.addf("]},");

    // The derived marker plane, and the camera pose the floor H implies.
    //
    // derived_z_mm next to camera_z_mm is the diagnostic pair: one is what the
    // matrix thinks the camera height is, the other is what somebody measured
    // with a tape. They should agree, and how far apart they are is how much
    // scale error the decomposition is carrying — which is the whole reason
    // CAMERA_HEIGHT exists. A nadir that is nowhere near under the camera is
    // the tell-tale of a bad K.
    {
      double cz = 0.0, nx = 0.0, ny = 0.0;
      const char* pose_why = "";
      const bool pose_ok = homography_.CameraPose(c, &cz, &nx, &ny, &pose_why);
      double hm[9];
      const bool ready = homography_.GetMarkerPlane(c, hm);
      j.addf("\"marker_plane\":{\"ready\":%s,\"reason\":\"%s\","
             "\"camera_z_mm\":%.1f,\"pose_ok\":%s,\"derived_z_mm\":%.1f,"
             "\"nadir_mm\":[%.1f,%.1f]",
             ready ? "true" : "false", ready ? "" : homography_.MarkerPlaneReason(c),
             homography_.CameraHeightMm(c), pose_ok ? "true" : "false",
             pose_ok ? cz : -1.0, pose_ok ? nx : 0.0, pose_ok ? ny : 0.0);
      if (ready) {
        // %.9e for the same reason H gets it: defined only up to scale, so one
        // matrix spans a wide exponent range and a fixed format flattens the
        // perspective terms to zero.
        j.addf(",\"H_marker\":[%.9e,%.9e,%.9e,%.9e,%.9e,%.9e,%.9e,%.9e,%.9e]",
               hm[0], hm[1], hm[2], hm[3], hm[4], hm[5], hm[6], hm[7], hm[8]);
      }
      j.addf("},");
    }

    // Per-point residuals from the last fit. RAM only — see FitResult for why
    // they are never written to disk.
    //
    // %.1f throughout: these are millimetres of disagreement with a tape
    // measure, and a tenth of one is already past what the tape can settle.
    // At four lenses and 24 points the block is ~1.9 KB, which is the last
    // large thing in this object; see the budget note on JsonBuf.
    const HomographyMapper::FitResult& fr = homography_.Residuals(c);
    if (!fr.have) {
      // Absent, not zeroed. "No fit has been checked" and "the fit is perfect"
      // must not render the same way, and 0.0 mm reads as the second one.
      j.addf("\"fit\":null}");
    } else {
      j.addf("\"fit\":{\"n\":%d,\"loo_valid\":%s,\"advisory\":%s,"
             "\"rmse_in_mm\":%.1f,\"rmse_loo_mm\":%.1f,\"max_loo_mm\":%.1f,"
             "\"max_loo_id\":%d,\"pts\":[",
             fr.n, fr.loo_valid ? "true" : "false", fr.advisory ? "true" : "false",
             fr.rmse_in_mm, fr.rmse_loo_mm, fr.max_loo_mm, fr.max_loo_id);
      for (int i = 0; i < fr.n; ++i)
        j.addf("%s{\"id\":%d,\"in\":%.1f,\"loo\":%.1f}", i ? "," : "", fr.pt[i].id,
               fr.pt[i].in_mm, fr.pt[i].loo_mm);
      j.addf("]}}");
    }
  }
  j.addf("],");

  // The limits the page must not exceed, published rather than duplicated.
  //
  // index.html had 24 and 4 written into its own validation, so raising
  // kMaxAnchors in the header would have left the dashboard refusing a list the
  // camera would happily accept — with nothing on the camera side to explain
  // it. Same reasoning as the governor block, which ships its bounds for
  // exactly this purpose.
  j.addf("\"limits\":{\"max_anchors\":%d,\"min_fit\":%d,\"min_loo\":%d,"
         "\"loo_advisory_below\":%d},",
         HomographyMapper::kMaxAnchors, HomographyMapper::kMinFitAnchors,
         HomographyMapper::kMinLooAnchors, HomographyMapper::kLooAdvisoryBelow);

  // Marker height is SHARED, so it sits outside the per-lens array. Putting a
  // copy in each lens's block would invite a page to edit one of them.
  j.addf("\"marker_height_mm\":%.1f,", homography_.MarkerHeightMm());

  // Last ANCHOR_* outcome. See AnchorCmdResult: this is how a rejected marker
  // list becomes visible to whoever sent it, since /cmd answers with this
  // object and nothing else.
  if (anchor_cmd_.ch < 0) {
    j.addf("\"anchor_cmd\":null,");
  } else {
    j.addf("\"anchor_cmd\":{\"ch\":%d,\"n\":%d,\"ok\":%s,\"reason\":\"%s\"},",
           anchor_cmd_.ch, anchor_cmd_.n, anchor_cmd_.ok ? "true" : "false",
           anchor_cmd_.reason);
  }

#if ENABLE_CENTRAL_TLS_STREAM
  // 주행 캘리(오도메트리). 세션이 안 돌아도 측정 슬롯 상태는 보여야 한다 —
  // 로봇 좌표가 파생/측정 중 어느 쪽으로 나가는지가 여기서만 드러난다.
  // 정지 판정 설정은 전역이라 배열 밖에 둔다. /status 에 실어야 하는 이유는
  // 이게 런타임에 바뀌는 값이기 때문이다 — 잔차가 이상할 때 "그때 임계값이
  // 얼마였나"를 답할 수 있어야 한다.
  j.addf("\"odom_settle\":{\"spread_px\":%.2f,\"frames\":%d,\"max_frames\":%d},",
         g_odom_settle_spread_px, g_odom_settle_frames, ODOM_SETTLE_FRAMES);
  j.addf("\"odom\":[");
  for (int c = 0; c < kMaxChannels; ++c) {
    const HomographyMapper::FitResult& fr = homography_.OdomResiduals(c);
    double closure = -1.0;
    homography_.OdomClosureMm(c, &closure);
    double sc = 0.0, crm = -1.0, cmx = -1.0;
    const char* cwhy = "";
    const bool cok = homography_.CompareMarkerPlanes(c, &sc, &crm, &cmx, &cwhy);
    j.addf("%s{\"ch\":%d,\"active\":%s,\"points\":%d,"
           "\"measured\":%s,\"preferred\":%s,\"k_stale\":%s,\"height_stale\":%s,"
           "\"n\":%d,\"loo_valid\":%s,\"rmse_in_mm\":%.1f,\"rmse_loo_mm\":%.1f,"
           "\"closure_mm\":%.1f,"
           "\"compare\":{\"ok\":%s,\"scale\":%.5f,\"rmse_mm\":%.1f,\"max_mm\":%.1f,\"reason\":\"%s\"}}",
           (c == 0) ? "" : ",", c,
           homography_.OdomActive(c) ? "true" : "false",
           homography_.OdomCount(c),
           homography_.MeasuredMarkerPlaneReady(c) ? "true" : "false",
           homography_.PreferMeasured(c) ? "true" : "false",
           homography_.MeasuredKStale(c) ? "true" : "false",
           homography_.MeasuredHeightStale(c) ? "true" : "false",
           fr.n, fr.loo_valid ? "true" : "false", fr.rmse_in_mm, fr.rmse_loo_mm,
           closure,
           cok ? "true" : "false", sc, crm, cmx, cok ? "" : (cwhy ? cwhy : ""));
  }
  j.addf("],");
#endif

  // Last IVA_SYNC. Raw sensor pixels — see IvaSyncResult's comment — so
  // tools/iva_push.sh can lift "points" straight into WiseAI's areaCoordinates
  // with no conversion.
  if (iva_sync_.ch < 0) {
    j.addf("\"iva_sync\":null,");
  } else {
    j.addf("\"iva_sync\":{\"ch\":%d,\"ok\":%s,\"n\":%d,\"points\":[",
           iva_sync_.ch, iva_sync_.ok ? "true" : "false", iva_sync_.n);
    for (int i = 0; i < iva_sync_.n; ++i) {
      j.addf("%s{\"x\":%.1f,\"y\":%.1f}", (i == 0) ? "" : ",",
             iva_sync_.px[i], iva_sync_.py[i]);
    }
    j.addf("],\"reason\":\"%s\"},", iva_sync_.reason);
  }

  // Per-channel WiseAI proximity state. state()/last_transition_ms() are
  // read-only, cost nothing to poll, and are the only way to see "is this
  // camera currently reporting danger" without waiting for the next Human
  // detection's stdout line to scroll past.
  j.addf("\"proximity\":[");
  for (int c = 0; c < kMaxChannels; ++c) {
    j.addf("%s{\"ch\":%d,\"state\":\"%s\",\"last_transition_ms\":%ld}",
           (c == 0) ? "" : ",", c, ProximityStateName(proximity_guard_[c].state()),
           proximity_guard_[c].last_transition_ms());
  }
  j.addf("],");

  // Last HG_MAP. %.4f rather than %.2f: this is the only window onto the
  // undistort maths, and rounding to 10 um would hide a discrepancy of exactly
  // the size worth arguing about when comparing against a reference
  // implementation.
  if (hg_map_.ch < 0) {
    j.addf("\"hg_map\":null}");
  } else {
    j.addf("\"hg_map\":{\"ch\":%d,\"px\":%.4f,\"py\":%.4f,\"ok\":%s,"
           "\"wx_mm\":%.4f,\"wy_mm\":%.4f,\"reason\":\"%s\"}}",
           hg_map_.ch, hg_map_.px, hg_map_.py, hg_map_.ok ? "true" : "false",
           hg_map_.wx, hg_map_.wy, hg_map_.reason);
  }
}
#endif  // ENABLE_STATUS_PAGE

/**
 * Raw-video callback. The NV12 Y plane is already 8-bit grayscale, so it is
 * wrapped in a cv::Mat with no copy and no colour conversion — the detector
 * only ever needs luma.
 */
void SampleComponent::ProcessRawVideo(Event* event) {
  // First frame only: prove the callback is actually firing. Without this a
  // silent "no frames" and a crash-on-start look identical from outside.
  static bool first = true;
  if (first) {
    first = false;
    printf("[ArucoPosePNM] first raw-video callback\n");
    fflush(stdout);
  }

  auto ret = eventToArgumentBuffer(event);

  // Counted here, ahead of everything — this is the last point at which an
  // event is known to have arrived at all. See the members for why.
  ++raw_events_;

  IPVideoFrameRaw* raw_frame = new ("GetImage") IPLVideoFrameRaw();
  raw_frame->DeserializeBaseObject(raw_frame, ret);
  // GetRawImage() returns a COPY of raw_frame's own shared_ptr<RawImage>, so
  // img keeps the actual image data alive on its own reference count — the
  // wrapper object has done its one job (deserializing the event into that
  // RawImage) and everything below reads through img/image, never raw_frame,
  // so it can be freed right here.
  //
  // It never was: found 2026-08-11 chasing a real, currently-active leak
  // (~20 KB/s at idle, no detection running on any channel) surfaced by
  // ReportCpu()'s new heap_in_use_kb figures on the pose link — every single
  // raw-video callback (all four lenses, independent of DETECT) allocated one
  // of these and never freed it, on both this path and the !img early return
  // below. BaseObject's tagged `new (tag) T()` is deallocated with a plain
  // `delete ptr` — the tag only selects the allocator at the call site; a
  // normal delete resolves to BaseObject::operator delete(void*), confirmed
  // against base_object.h. ~IPLVideoFrameRaw() is empty but virtual (the
  // `override` on it proves the base destructor is), so deleting through the
  // IPVideoFrameRaw* base pointer is well-defined.
  std::shared_ptr<RawImage> img(raw_frame->GetRawImage());
  delete raw_frame;
  if (!img) {
    ++raw_no_image_;
    return;
  }

  const long t_frame_ms = epoch_ms();

  // Ahead of the per-channel loop and every skip path in it, same reasoning as
  // raw_events_/report_cpu_if_due() in cctv_app: CPU load is worth knowing
  // precisely when detection is off on every lens, not just when it's on.
  if (t_frame_ms - cpu_report_ms_ >= kCpuReportIntervalMs) {
    cpu_report_ms_ = t_frame_ms;
    ReportCpu();
  }

  for (RawImage* image = img.get(); image; image = image->next) {
    if (image->format != RAW_FMT_NV12) continue;

    // Which lens produced this frame. Everything downstream is per channel.
    const int ch = (int)image->chan_id;
    if (ch < 0 || ch >= kMaxChannels) continue;

    // Every frame the SDK hands over, counted BEFORE any decision this app
    // makes about it.
    //
    // This is the only number that answers "how fast does the camera actually
    // feed us", and until 2026-08-05 there was no such number. `frames` counts
    // detections and `skipped` counts what the governor threw away, so their
    // sum is the delivery rate — but ONLY while detection is on. A lens with
    // DETECT 0 returns below without touching either counter, so it reads as
    // 0 fps whether the SDK is sending 5 frames a second or none at all. Three
    // of the four lenses sat in exactly that state while the raw-fps question
    // was being argued, which is how a switched-off channel came to look like
    // evidence about the SDK.
    //
    // Placed above the format check would be wrong in the other direction: a
    // frame in an unexpected format was still delivered, but it is not a frame
    // this app could ever have used, and mixing the two would hide a format
    // problem inside a healthy-looking rate.
    ++delivered_[ch];
    // When, for the governor's channel count. The counter above cannot answer
    // "is this lens still feeding us" without a previous reading to subtract,
    // and the only place that differencing happens is SampleRates() — which
    // runs when /status is polled, so in_fps_ freezes at its last value the
    // moment nobody has the page open. A governor that divided the budget by a
    // number that only moves while a browser is watching would change the
    // app's timing depending on whether anyone was looking at it.
    last_delivery_ms_[ch] = t_frame_ms;

    // A lens with a calibration session open is doing a different job, and the
    // two switches below do not apply to it:
    //
    //   - detect_enabled_ is about marker output, which is suppressed during a
    //     session anyway. Refusing to calibrate a lens because its marker
    //     search is off would be a rule with no reason behind it.
    //   - the governor's budget was set by the cost of ArUco SEARCH on this
    //     lens (1.3 s of enforced idle at four channels). Making an operator
    //     who is standing there holding a board wait that out — or judging the
    //     capture on a frame from over a second ago — is the wrong trade. The
    //     board probe throttles itself instead (CALIB_PROBE_MS).
    const bool calibrating = calib_.Collecting(ch);
    // A homography collection session on THIS lens. Per channel, unlike the
    // K/dist session: that one is exclusive because a person has to stand in
    // front of the lens holding a board, whereas the markers here are already
    // stuck to the floor and nobody has to be anywhere.
    const bool hg_collecting = homography_.FitCollecting(ch);
    // Tried exempting hg_collecting from the governor too (2026-08-10), on
    // the theory that it deserved the same treatment as calibrating above.
    // Reverted the same day: the two are NOT alike where it matters here.
    // calibrating's exemption only ever lets through a brief CAPTURE burst
    // (bounded, rare) — the continuous aiming view is a SEPARATE, cheap,
    // self-throttled probe (CALIB_PROBE_MS). hg_collecting has no such split:
    // exempting it meant the single most expensive detect mode there is
    // (full-frame, single-stage, unshrunk — see two_stage below) ran flat out
    // on EVERY frame for the whole session, with nothing left to drop a frame
    // that arrived faster than the last one finished. Measured result: CPU
    // over 100%, the event queue growing, and /status unreachable — exactly
    // the failure this governor exists to prevent (see the comment on the
    // skip check below). Left throttled; use DUTY <pct> (up to kDutyMax) for
    // a faster-but-still-bounded session instead of removing the bound.

    // Switched-off lens: drop the frame before anything expensive. Checked
    // ahead of the plane lookup and the cv::Mat wrapper so a disabled channel
    // costs one comparison, which is the whole point of the switch.
    //
    // A collection session overrides the switch, the same way the K/dist
    // session does: CALIB_START on a lens whose detection happens to be off
    // would otherwise sit at 0 of 20 for five minutes and time out, with the
    // one fact that explains it visible on a different row of the page.
    if (!detect_enabled_[ch] && !calibrating && !hg_collecting) break;

    // Over the detection budget: drop this frame just as cheaply. Dropping is
    // the correct answer rather than a queue — the next frame carries fresher
    // data, and letting the queue grow is what made HTTP unreachable.
    if (!calibrating && t_frame_ms < detect_budget_until_ms_[ch]) {
      ++detect_skipped_[ch];
      NoteFrame(ch, true);
      break;
    }

    // How stale this frame already is. pts is the capture instant on the SoC's
    // own millisecond clock; t_frame_ms is now, on the wall clock. Their
    // difference is a fixed clock offset plus however long the frame waited in
    // the queue, so the smallest difference ever seen IS the offset, and the
    // excess over it is the wait. See the header for why this matters: without
    // it a pose packet reports only its own detect time and looks fresh no
    // matter how far behind the app has fallen.
    //
    // Per channel, not shared. One shared minimum assumes the four lenses put
    // frames into the pipeline with the same delay, and if they do not, the
    // channel with the shortest path sets the baseline and every other channel
    // reports its own fixed pipeline offset as queue time — a constant error
    // that never varies and so never looks like one.
    const long pts_ms = (long)image->pts;
    const long offset = t_frame_ms - pts_ms;
    if (offset < pts_offset_min_[ch]) pts_offset_min_[ch] = offset;
    const long queue_ms = offset - pts_offset_min_[ch];
    last_queue_ms_[ch] = queue_ms;

    // Locate the luma plane. NV12 is Y followed by interleaved UV; we only
    // want Y, and `pitch` (not width) is the row stride.
    void* y_addr = NULL;
    for (uint32_t p = 0; p < image->num_planes; ++p) {
      if (image->plane[p].type == RAW_PLANE_Y) {
        y_addr = (void*)image->plane[p].vir_addr;
        break;
      }
    }
    if (y_addr == NULL) continue;

    if (!aruco_[ch]) continue;

    // Past every early-out, so this frame really is about to be detected.
    NoteFrame(ch, false);

    cv::Mat gray((int)image->height, (int)image->width, CV_8UC1, y_addr, (size_t)image->pitch);

    // --- intrinsics calibration, on the lens that has the session open ------
    // Placed before the ArUco pass and taking the frame first, because during
    // a session this lens's job IS the board: the operator is standing in
    // front of it holding one, and pose output from it is not being used.
    if (calibrating) {
      if (calib_.CapturePending(ch)) {
        calib_.TakePendingCapture(ch, gray);
        // Report once the burst actually settles (CALIB_CAPTURE_BURST_MS —
        // see TakePendingCapture), not on every retry inside it: the retries
        // are meant to be invisible to the operator, and a REJECTED line per
        // attempt would flood the log with failures nobody asked about.
        if (!calib_.CapturePending(ch)) ReportCalibKProgress(ch);
      } else {
        // Throttled board probe so the page can say "board visible, N corners"
        // and draw where it is. The calibration UI has no photo, so without
        // this, aiming the board would be blind. See CALIB_PROBE_MS.
        //
        // ProbeIfDue() only returns true when it actually ran (its own
        // CALIB_PROBE_MS throttle passed), so this pushes the held corner set
        // over the pose link exactly when it changed instead of every frame.
        if (calib_.ProbeIfDue(ch, gray, t_frame_ms, marker_hold_ms_))
          ReportCalibKProbe(ch);
      }
      // Pose output is suppressed for this lens while calibrating — the same
      // rule the original app used. What it would publish now is a person's
      // hands and a test pattern, and the server has no way to know that.
      last_frame_ms_[ch] = t_frame_ms;
      last_w_[ch] = (int)image->width;
      last_h_[ch] = (int)image->height;
      // Drop the stale detections with them. Leaving them meant /status kept
      // reporting the markers this lens saw BEFORE the session — next to a
      // frame age of 266 ms, because the frame really is fresh. The overlay
      // then drew half-minute-old marker positions as if they were current,
      // which is the one thing a coordinate overlay must never do.
      last_markers_[ch] = 0;
      last_dets_[ch].clear();
      last_dets_approx_[ch] = false;
      // The flicker hold must not resurrect a pre-session detection the
      // instant calibrating ends — same reasoning as the three lines above.
      for (int s = 0; s < kMaxRecentMarkers; ++s) recent_marker_[ch][s].id = -1;
      break;
    }

    // Narrow the search to this channel's tracked box before detecting.
    // detectMarkers() costs time proportional to pixels, so this is where the
    // speed-up comes from. Corners come back full-frame either way.
    //
    // The second argument suppresses the tracker, and a collection session is
    // exactly what it was put there for. The tracked box follows whichever
    // markers it has been holding onto; a session needs EVERY registered marker
    // in one frame, and any frame missing one is thrown away whole. Left on, a
    // session on a lens with a settled ROI would discard all 200 frames and
    // time out while the markers it wanted sat in plain view outside the box.
    //
    // It is per channel, and the check above is per channel too — a session on
    // ch1 must not push ch0 into full-frame scanning, which would cost the
    // other three lenses their share of the one thread for no reason.
    aruco_[ch]->setRoi(dynroi_[ch].roiForFrame(manual_roi_[ch], hg_collecting));

    // Full resolution once the tracker has a box (already 7..16 ms and the
    // corners from it are what the server consumes); shrunk while searching,
    // where the scan is full-frame and the result only has to be good enough
    // to aim the next one. The policy lives here because `tracking()` is the
    // thing that distinguishes the two cases.
    //
    // Reduced-scale search is HALF of a two-stage scheme: shrink to find the
    // marker, then let the tracker re-measure it at full resolution next
    // frame. The second stage only exists while dynROI is on, so the scale is
    // applied only then. With dynROI off the approximate corners would be the
    // only ones ever produced — and they are deliberately not published, so
    // the app would detect markers and report none of them. That is exactly
    // what happened before this condition was added.
    //
    // A collection session forces the single-stage path. The shrunken search
    // produces deliberately approximate corners — good enough to aim the next
    // frame at, and never published for that reason — and a homography fitted
    // from them would bake that approximation into every world coordinate the
    // lens ever reports. Detection costs more this way, which is why the
    // session has a frame budget rather than a frame count.
    const bool two_stage = dynroi_[ch].enabled() && !hg_collecting;
    aruco_[ch]->setSearchScale((two_stage && !dynroi_[ch].tracking()) ? search_scale_[ch] : 1);

    // Split the frame's heap growth between the marker search and everything
    // this app does with the result.
    //
    // /status showed heap_in_use climbing ~1 KB per delivered frame with
    // detection on and dead flat with it off (measured .13, 2026-08-05), which
    // proves the growth is real — freed memory would show as heap_free — and
    // puts it somewhere past the detect_enabled_ early-out. That is still a
    // hundred lines of code and an OpenCV call. Two numbers split it in one
    // run, and reading them costs a mallinfo2 every kHeapProbeEvery-th frame
    // rather than on every one.
    unsigned long heap_h0 = 0;
    const bool heap_probe = (++heap_probe_seq_ % kHeapProbeEvery) == 0;
    if (heap_probe) heap_bytes(&heap_h0, NULL);

    // A throw inside detect() would otherwise unwind through the SDK's event
    // dispatch and take the process down.
    std::vector<ArucoProcessor::Detection> dets;
    try {
      aruco_[ch]->detect(gray, dets);
    } catch (const std::exception& e) {
      printf("[ArucoPosePNM] ch%d detect FAILED: %s\n", ch, e.what());
      fflush(stdout);
      break;
    } catch (...) {
      printf("[ArucoPosePNM] ch%d detect FAILED (unknown)\n", ch);
      fflush(stdout);
      break;
    }

    unsigned long heap_h1 = 0;
    if (heap_probe) {
      heap_bytes(&heap_h1, NULL);
      // Signed, because a probe that lands while the allocator happens to be
      // releasing would otherwise wrap into a huge positive number and swamp
      // the running total on a single sample.
      heap_detect_bytes_ += (long long)heap_h1 - (long long)heap_h0;
    }

    // Charge this search against THIS channel's share of the budget.
    //
    // The total duty is split evenly over the channels that are actually
    // searching, so each gets DUTY/n. A search costing c may therefore repeat
    // every c * n * 100 / DUTY, and what is owed as idle time is that minus c:
    //
    //     wait = c * (100 * n / DUTY - 1)
    //
    // With four channels at 60%: a 250 ms full-frame scan waits ~1.4 s (~0.7
    // fps on that lens), while a 7 ms tracking scan waits only ~40 ms (~21
    // fps). A cheap channel is never held back to the pace of an expensive
    // one — it simply does not use up its share.
    //
    // n is a count of who is really consuming the thread, not a count of
    // switches, because it sits in the denominator: every channel wrongly
    // included there takes idle time away from the ones doing the work. Both
    // corrections below are arithmetic, not policy.
    {
      const double cost_ms = aruco_[ch]->lastDetectMs();
      if (cost_ms > 0.0) {
        int active = 0;
        for (int c = 0; c < kMaxChannels; ++c) {
          // Whatever makes channel c run a marker search counts, and the
          // switch is not the only thing that does: both session types
          // deliberately override it (see the early-out above), and a
          // homography session runs the most expensive mode there is —
          // full-frame, single-stage, unshrunk. Counting only the switch made
          // a collection session on a DETECT 0 lens invisible to the
          // governor, so the other lenses divided the budget as if that
          // thread time were free.
          if (!detect_enabled_[c] && !homography_.FitCollecting(c) &&
              !calib_.Collecting(c))
            continue;
          // ...and only while frames are still arriving on it. A lens that is
          // switched on but whose stream has stopped costs nothing, so
          // leaving it in the denominator holds the surviving lenses to a
          // quarter of the thread with three quarters of it idle — the
          // failure looks like a performance problem on the healthy lenses
          // rather than a dead stream on the broken one.
          //
          // Never-delivered (0) counts as active on purpose: at start-up the
          // four lenses come up a few frames apart, and guessing "idle" for a
          // channel that is about to arrive is the one error here that
          // over-commits the thread instead of under-using it.
          if (last_delivery_ms_[c] != 0 &&
              t_frame_ms - last_delivery_ms_[c] > kActiveIdleMs)
            continue;
          ++active;
        }
        if (active < 1) active = 1;
        governor_active_ = active;
        const double wait = cost_ms * (100.0 * active / detect_duty_pct_ - 1.0);
        detect_budget_until_ms_[ch] = epoch_ms() + (long)(wait > 0.0 ? wait : 0.0);
      }
    }

    // Fold this frame into the box (also emits DYNROI_STATE on transitions).
    dynroi_[ch].update(dets, cv::Size((int)image->width, (int)image->height));

#if ENABLE_STATUS_PAGE
    // last_frame_ms_/last_w_/last_h_ always reflect THIS frame, detection or
    // not — that is what "camera alive" means. last_markers_ is set below,
    // after the flicker hold decides what actually gets shown.
    last_frame_ms_[ch] = t_frame_ms;
    last_w_[ch] = (int)image->width;
    last_h_[ch] = (int)image->height;
#endif

    // Corners from a shrunk image are accurate to a few pixels, which is fine
    // for placing an ROI and not fine for a robot. They are not published:
    // dynroi_ has just switched to TRACK on the strength of them, so the very
    // next frame on this channel produces exact corners from the full-
    // resolution pass. The cost is one frame (~33 ms) at the moment a marker
    // is re-acquired, right after a stretch of having none at all.
    //
    // The heartbeat still goes out on that frame, so "camera alive, marker
    // lost" stays distinguishable from a dead link — it just carries no
    // corners, which is exactly what is true: nothing precise was measured.
    // Belt and braces: suppress only when a full-resolution pass really is
    // coming. lastWasScaled() alone was not enough — it says the corners are
    // approximate, not that anything better will follow.
    const bool approximate = two_stage && aruco_[ch]->lastWasScaled() && !dets.empty();

    // Keep this channel's recent-sighting cache warm from every EXACT
    // detection (approximate SEARCH-stage corners are not trustworthy enough
    // to stand in for a marker that goes missing later — see FillRecentMarkers
    // and MARKER_HOLD_MS). Run unconditionally, not just while a homography
    // session is open, so a session's first frame already has history.
    if (!approximate) {
      for (size_t i = 0; i < dets.size(); ++i)
        RememberMarker(ch, dets[i], t_frame_ms);
    }

#if ENABLE_STATUS_PAGE
    // Flicker hold, DISPLAY ONLY, per marker: this frame's own `dets` unioned
    // with anything still live in the recent-sighting cache (age permitting —
    // MARKER_HOLD_MS). Per-marker rather than a whole-frame swap: a channel
    // that finds a DIFFERENT subset of the same markers on consecutive
    // attempts — routine once the detection governor starts spacing attempts
    // out — would otherwise flicker every marker missing from THIS frame's
    // subset even though `dets` is never actually empty. Neither
    // SendPosePackets below nor the homography FeedFrame above ever sees this
    // — both already ran on the frame's own `dets`.
    BuildHeldDets(ch, dets, t_frame_ms, &last_dets_[ch]);
    last_markers_[ch] = (int)last_dets_[ch].size();
    last_dets_approx_[ch] = approximate;
#endif

#if ENABLE_CENTRAL_TLS_STREAM
    // 주행 캘리: 무장된 지점이 있으면 이 프레임을 표본으로 쓴다.
    //
    // `approximate`(동적 ROI 가 추정한 좌표) 프레임은 넘기지 않는다 — 캘리는
    // EXACT 검출만 써야 한다. 정적 앵커 수집이 같은 이유로 !approximate 를
    // 조건에 둔다.
    //
    // 거버너는 그대로 적용받는다. 정적 앵커 수집을 거버너에서 빼봤다가 CPU 가
    // 100% 를 넘고 /status 가 죽어 되돌린 기록이 있다(위 hg_collecting 주석).
    // stop-and-go 라 급할 이유도 없다.
    //
    // pts 는 SoC 자체 시계라 epoch_ms() 와 직접 비교할 수 없다. 위에서 이미
    // 구해 둔 pts_offset_min_[ch](관측된 최소 차이 = 고정 클럭 오프셋)를 더해
    // 벽시계로 옮긴다. 이 추정치는 참값보다 작을 수 있어 촬영 시각을 조금
    // 이르게 보는데, 그러면 요청 직후 프레임이 걸러져 **더 기다리는 쪽으로만**
    // 틀린다 — 이동 중 프레임을 채택하는 것보다 안전한 방향이다.
    if (!approximate) OdomFeedFrame(ch, dets, pts_ms + pts_offset_min_[ch], t_frame_ms);
    // 캡처·세션 데드라인은 여기가 아니라 OdomSweepTimeouts() 가 본다 — 이 자리는
    // 이 렌즈의 !approximate 프레임이 와야 도는데, 데드라인이 필요한 상황이 바로
    // 그 프레임이 안 오는 상황이다.
#endif

    // Feed the collection session, if this lens has one open.
    //
    // This and the world coordinates on the pose packet are the only work the
    // frame path does for the homography. The cost is one centre per detection
    // plus, inside FeedFrame(), one undistort per REGISTERED marker — against a
    // detection that takes 180..300 ms on this lens while a session is open, it
    // does not register.
    //
    // The centre is the mean of the four corners, which is the same point the
    // world coordinate will be computed from later. Using a different
    // definition in the two places (the marker origin corner, say) would put a
    // fixed offset of half a marker between where the calibration thinks the
    // marker is and where the tracker reports it, and every position would be
    // wrong by that amount with a residual table that looked perfect.
    if (hg_collecting && !approximate) {
      // 64 is well past the 24 that can be registered; extra markers in view
      // are ignored by FeedFrame() anyway, and a stack array keeps the frame
      // path free of allocation.
      int   ids[64];
      float cxs[64], cys[64];
      int   nd = 0;
      for (size_t i = 0; i < dets.size() && nd < 64; ++i) {
        const ArucoProcessor::Detection& d = dets[i];
        if (d.corners2d.size() < 4) continue;
        ids[nd] = d.id;
        MarkerPoints(d, &cxs[nd], &cys[nd], NULL, NULL);
        ++nd;
      }
      // Fill in any registered anchor missing from THIS exact frame from its
      // recent sighting. Anchors are floor-fixed for the length of a capture
      // session, so a sighting up to MARKER_HOLD_MS old is still the true
      // position — and without this, FeedFrame() (which needs every
      // registered marker in the SAME frame or discards it whole) throws away
      // an otherwise-good frame every time any one anchor blinks out for a
      // single detection. Correctness matters more than latency on this path;
      // nothing about a calibration session is real-time.
      nd = FillRecentMarkers(ch, ids, cxs, cys, nd, 64, t_frame_ms);
      homography_.FeedFrame(ch, ids, cxs, cys, nd);

      // hg_collecting was true going into this call (the branch condition
      // above); FeedFrame() is the only thing that can end a session
      // (FinishFit, on reaching the target good-frame count, or the frame
      // budget running out). Not collecting any more means it just did —
      // report the fit (H + LOO residual summary) now, once, right when a
      // TCP-only dashboard would otherwise have no way to learn it happened.
      if (!homography_.FitCollecting(ch)) {
        ReportHomography(ch);
        ReportHomographyFit(ch);
        ReportHomographyFitPoints(ch);
        // Terminal signal, sent last so it lands after the H/residuals above:
        // the dashboard raised a "collecting…" banner on the ack and only this
        // clears it (via the bridge's [calib] SUCCESS/FAILED broadcast).
        ReportHgDone(ch);
#if ENABLE_CENTRAL_TLS_STREAM
        // 피팅이 성공했으면 서버로도 바로 올린다 (2026-08-14). 주행 캘리가
        // CentralCalibDone() 에서 하는 것과 같은 자리다.
        //
        // 예전에는 이 자리에서 아무것도 안 나갔고, 서버가 바닥 H 를 알게 되는
        // 유일한 길은 대시보드가 HG_QUERY 로 값을 받아 자기가 조립한 번들을
        // CENTRAL_HMATRIX 로 되돌려보내는 왕복이었다. 그래서 관리자 창을 안
        // 열어 둔 채 캘리하면 결과가 카메라 안에만 남았고, 반대로 대시보드는
        // "이 HG 가 새 캘리인지 단순 조회의 답인지"를 값이 변했는지로 추측해야
        // 했다(HG 는 두 경우가 같은 타입이다). 여기서 보내면 둘 다 사라진다 —
        // 새 캘리가 났다는 것을 아는 곳은 카메라뿐이고, 그게 바로 여기다.
        if (homography_.Available(ch)) SendFloorBundle(ch, "fit-complete");
#endif
      }
    }
    static const std::vector<ArucoProcessor::Detection> kNoDetections;
    SendPosePackets(ch, approximate ? kNoDetections : dets, t_frame_ms,
                    (int)image->width, (int)image->height, pts_ms, queue_ms);

    if (heap_probe) {
      unsigned long heap_h2 = 0;
      heap_bytes(&heap_h2, NULL);
      heap_rest_bytes_ += (long long)heap_h2 - (long long)heap_h1;
      ++heap_probe_n_;
    }
    break;  // one frame per event
  }

  // Same cadence as the frames: the dashboard's DYNROI toggles land here.
  PollDashboardCommands();
#if ENABLE_CENTRAL_TLS_STREAM
  PollCentralCommands();
  // 프레임 경로 밖의 안전망. 굶은 렌즈의 캡처도 여기서 사유를 달고 끝난다.
  OdomSweepTimeouts();
#endif
}

/**
 * WiseAI 사람/차량 bbox 이벤트. 파싱은 `wiseai_metadata.{h,cc}` — 필드명·좌표계는
 * 2026-08-18 실측으로 확정됨 (픽셀, (left,top,right,bottom), 발끝 = 하단 중앙; 자세한
 * 근거는 docs/08.18/2026-08-18_WISEAI_METADATA_SAMPLE.md).
 *
 * 사람 발끝은 바닥 평면이라 PixelToWorld() (바닥 H), 로봇은 마커 평면이라
 * PixelToWorldMarker() (시차 보정된 H_marker) — 서로 다른 평면이니 각자 맞는
 * 경로로 월드mm를 구한 뒤에 거리를 재야 한다. 픽셀 공간에서 먼저 거리를 재고
 * 나중에 스케일을 곱하는 지름길은 원근 때문에 위치마다 배율이 달라 쓸 수 없다
 * (SendPosePackets 의 각도 계산과 같은 이유).
 *
 * 로봇 위치는 이 채널의 recent_marker_ 캐시에서 ROBOT_MARKER_ID를 찾는다 —
 * WiseAI 이벤트와 ArUco 검출은 별도 프레임 소스라 정확히 같은 타이밍에 안
 * 온다. marker_hold_ms_(SendPosePackets/FillRecentMarkers와 같은 상수)보다
 * 오래된 목격은 신뢰하지 않는다.
 */
void SampleComponent::ProcessWiseAiMetadata(Event* event) {
  // Hanwha's own metadata_sample source uses event->GetAttachment<...>(),
  // but this project's SDK API.md documents eventToArgumentBuffer() +
  // DeserializeBaseObject() instead -- the same pattern ProcessRawVideo()
  // already uses for IPVideoFrameRaw, proven to link and dlopen on this
  // camera. sample_component is a MODULE library: an unresolved symbol does
  // NOT fail the build, only dlopen() on the real device (see CMakeLists.txt
  // comment on IPLVideoFrameRaw) -- so GetAttachment<>()'s template
  // instantiation compiling clean here was no guarantee it would load.
  // Using the already-proven path removes that as a variable.
  auto ret = eventToArgumentBuffer(event);
  IPMetadataManager::MetadataOutput param;
  param.DeserializeBaseObject(&param, ret);
  const int channel = param.channel();
  const std::string& metadata = param.output();
  bool wrote_log = false;
#if ENABLE_WISEAI_FRAME_LOG
  printf("[ArucoPosePNM] WiseAI metadata ch=%d output=%s\n",
         channel, metadata.c_str());
  wrote_log = true;
#endif

  if (channel < 0 || channel >= kMaxChannels) {
    if (wrote_log) fflush(stdout);
    return;
  }
  const long now_ms = epoch_ms();

  // Robot's last-known marker centre on this channel, if fresh enough to
  // trust -- same staleness rule FillRecentMarkers()/SendPosePackets() use.
  const RecentMarker* robot = NULL;
  for (int s = 0; s < kMaxRecentMarkers; ++s) {
    const RecentMarker& rm = recent_marker_[channel][s];
    if (rm.id != ROBOT_MARKER_ID) continue;
    if (now_ms - rm.seen_ms > marker_hold_ms_) continue;
    robot = &rm;
    break;
  }

  // Every Human in this metadata frame is compared with the same recent
  // robot sighting. Project that marker lazily, once per callback rather than
  // once per person; callbacks containing only an IVA event pay no H work.
  double robot_wx = 0.0, robot_wy = 0.0;
  bool robot_world_checked = false;
  bool robot_world_ready = false;

  std::vector<WiseAiDetection> detections;
  if (metadata.find("<tt:Object") != std::string::npos)
    ParseWiseAiMetadata(metadata, channel, &detections);
  for (const auto& d : detections) {
    if (d.class_type != "Human") continue;  // Face/Head parts aren't foot points
#if ENABLE_WISEAI_FRAME_LOG
    printf("[ArucoPosePNM] WiseAI ch=%d Human foot_px=(%.1f,%.1f) likelihood=%.2f\n",
           channel, d.foot_u, d.foot_v, d.likelihood);
    wrote_log = true;
#endif
    // Our own zone verdict, on the foot point -- the one part of the bbox
    // that lies on the plane the zone was calibrated on. See iva_zone_'s
    // comment for the measurement that says WiseAI's own Enter/Exit cannot
    // stand in for this. -1 when this channel has no zone: no verdict, rather
    // than a false "outside" that would fire an Exit the moment one is synced.
    // The foot is judged as a DISC, not a point: a standing person's feet
    // cover an area, and a single pixel flickers in and out at the boundary
    // in a way real feet do not. Radius scales with the bbox (see
    // ZONE_FOOT_RADIUS_DIV) so it means roughly the same real-world size at
    // any range. "Inside" = the disc reaches the zone, so someone standing
    // just outside the line still counts -- for a warning zone, erring toward
    // triggering is the right direction.
    const float zone_d = IvaZoneDistancePx(channel, d.foot_u, d.foot_v);
    const bool have_zone = (zone_d >= 0.0f);
    const float foot_r = (d.right - d.left) / (float)zone_radius_div_;

    // Distance in millimetres, when this channel can say. Preferred over the
    // pixel figure for everything a human reads or a threshold compares:
    // "0.3 m from the zone" is a claim that survives the person turning
    // sideways, where the bbox-width radius moved by 20% on the same person
    // (measured 2026-08-20). Pixels remain the fallback for a channel with no
    // homography, where no world statement is possible at all.
    double fwx = 0.0, fwy = 0.0;
    const bool foot_world =
        homography_.PixelToWorld(channel, d.foot_u, d.foot_v, &fwx, &fwy, NULL);
    const float zone_mm = foot_world
        ? IvaZoneWorldDistanceMm(channel, fwx, fwy)
        : -1.0f;
    const bool have_mm = (zone_mm >= 0.0f);

    // 0 clear, 1 warn, 2 danger, 3 inside; -1 = no verdict possible.
    int zone_level = -1;
    if (have_mm) {
      zone_level = (zone_mm <= 0.0f) ? 3
                 : (zone_mm <= (float)zone_danger_mm_) ? 2
                 : (zone_mm <= (float)zone_warn_mm_) ? 1 : 0;
    } else if (have_zone) {
      zone_level = (zone_d <= foot_r) ? 3 : 0;   // 밀리미터가 없으면 2단계뿐
    }
    // `inside` in the optional realtime feed remains the polygon verdict;
    // ZONE_EVENT itself uses zone_alarm_level_ and may fire in an outer band.
    const int zone_now = (zone_level < 0) ? -1 : ((zone_level == 3) ? 1 : 0);
    const ZoneEdge edge = RememberWiseAiObject(channel, d, now_ms, zone_level);
    if (edge != kZoneNoChange) {
      // Foot point in world mm too, when this channel can: the zone is a floor
      // region, so "where on the floor" is the answer an alarm actually wants,
      // and pixels are only meaningful to something holding this same lens's
      // calibration. Omitted, not zeroed, when PixelToWorld can't -- (0,0) is
      // a real point on the floor and a wrong claim to make.
      char zpayload[512];
      int w = snprintf(zpayload, sizeof(zpayload),
                       "{\"ch\":%d,\"object_id\":\"%.32s\","
                       "\"action\":\"%s\",\"foot_u\":%.1f,\"foot_v\":%.1f,"
                       "\"left\":%.1f,\"top\":%.1f,\"right\":%.1f,\"bottom\":%.1f",
                       channel, d.object_id.c_str(),
                       (edge == kZoneEntered) ? "Enter" : "Exit",
                       d.foot_u, d.foot_v, d.left, d.top, d.right, d.bottom);
      if (foot_world && w > 0 && w < (int)sizeof(zpayload)) {
        w += snprintf(zpayload + w, sizeof(zpayload) - w,
                      ",\"foot_wx\":%.0f,\"foot_wy\":%.0f", fwx, fwy);
      }
      // When the camera SAW this crossing, not when we are sending it. Absent
      // rather than 0 if the frame carried no parseable stamp -- 0 is a real
      // epoch millisecond (1970) and would read as a wildly stale event.
      if (d.utc_ms != 0 && w > 0 && w < (int)sizeof(zpayload)) {
        w += snprintf(zpayload + w, sizeof(zpayload) - w, ",\"t_ms\":%ld", d.utc_ms);
      }
      // The disc that made this call, so the decision can be second-guessed
      // downstream without re-deriving it: radius used, and how far the foot
      // actually was from the zone (0 = the point itself was inside).
      if (w > 0 && w < (int)sizeof(zpayload)) {
        w += snprintf(zpayload + w, sizeof(zpayload) - w,
                      ",\"foot_r\":%.1f,\"zone_d\":%.1f", foot_r, zone_d);
      }
      if (have_mm && w > 0 && w < (int)sizeof(zpayload)) {
        w += snprintf(zpayload + w, sizeof(zpayload) - w, ",\"zone_mm\":%.0f", zone_mm);
      }
      // 어느 단계를 넘어서 난 이벤트인지 -- 받는 쪽이 "접근금지였나 존 내부였나"를
      // 구분해 다른 소리를 낼 수 있게 한다.
      if (zone_level >= 0 && w > 0 && w < (int)sizeof(zpayload)) {
        w += snprintf(zpayload + w, sizeof(zpayload) - w,
                      ",\"level\":%d,\"alarm_level\":%d", zone_level, zone_alarm_level_);
      }
      if (w <= 0 || w >= (int)sizeof(zpayload) - 1) continue;
      zpayload[w++] = '}';
      zpayload[w] = '\0';

      char zjson[544];
      const int zw = snprintf(zjson, sizeof(zjson),
                              "{\"type\":\"ZONE_EVENT\",%s", zpayload + 1);
      if (zw <= 0 || zw >= (int)sizeof(zjson)) continue;

      printf("[ArucoPosePNM] %s\n", zjson);
      wrote_log = true;
      // 운영 경로: 카메라의 기존 role=CCTV TLS 세션으로 중앙 서버에 전달한다.
      // 연결이 내려가 있으면 send_typed가 즉시 실패하고 영상 처리는 계속된다.
      central_tls_sender_send_typed("ZONE_EVENT", zpayload);
      // Control line, not the realtime one, for the same reason IVA_EVENT uses
      // it: 관리자 화면 표시와 RP_CCTV_BRIDGE=1 과도기 경로도 유지한다.
      pose_sender_send_control_line(zjson);
    }

    // Per-frame position feed for the dashboard overlay (DET_STREAM). Sent on
    // the REALTIME line, not the control line ZONE_EVENT uses: the next frame
    // supersedes this one, so a drop costs nothing, while a retry queue would
    // deliver stale positions late and make the overlay lag reality. Same
    // reasoning CAM_POSE is sent that way.
    //
    // Deliberately no printf to go with it: at ~5 lines/s per channel this
    // would bury the app's console log, which is already short enough to lose
    // history to rotation.
    if (det_stream_) {
      char dj[288];
      int dw = snprintf(dj, sizeof(dj),
                        "{\"type\":\"WISEAI_DET\",\"ch\":%d,\"object_id\":\"%.32s\","
                        "\"left\":%.1f,\"top\":%.1f,\"right\":%.1f,\"bottom\":%.1f,"
                        "\"foot_u\":%.1f,\"foot_v\":%.1f,\"foot_r\":%.1f",
                        channel, d.object_id.c_str(), d.left, d.top, d.right, d.bottom,
                        d.foot_u, d.foot_v, foot_r);
      // Zone verdict only when this channel HAS a zone -- "outside" and "no
      // zone to be outside of" are different, and the overlay colours them
      // differently.
      if (have_zone && dw > 0 && dw < (int)sizeof(dj)) {
        dw += snprintf(dj + dw, sizeof(dj) - dw, ",\"zone_d\":%.1f,\"inside\":%s",
                       zone_d, (zone_now == 1) ? "true" : "false");
      }
      // 밀리미터 거리와 단계는 호모그래피가 있을 때만 실린다 -- 없으면 필드를
      // 아예 빼서, 화면이 "0mm"를 진짜 거리로 오독하지 않게 한다.
      if (have_mm && dw > 0 && dw < (int)sizeof(dj)) {
        dw += snprintf(dj + dw, sizeof(dj) - dw, ",\"zone_mm\":%.0f", zone_mm);
      }
      if (zone_level >= 0 && dw > 0 && dw < (int)sizeof(dj)) {
        dw += snprintf(dj + dw, sizeof(dj) - dw, ",\"level\":%d", zone_level);
      }
      if (d.utc_ms != 0 && dw > 0 && dw < (int)sizeof(dj)) {
        dw += snprintf(dj + dw, sizeof(dj) - dw, ",\"t_ms\":%ld", d.utc_ms);
      }
      if (dw > 0 && dw < (int)sizeof(dj)) snprintf(dj + dw, sizeof(dj) - dw, "}");
      pose_sender_send_line(dj);
    }

    double distance_mm = -1.0;
    bool have_distance = false;
    const char* why = "";
    if (robot == NULL) {
      why = "로봇 마커를 최근에 못 봄";
    } else if (!foot_world) {
      why = "사람 발끝의 월드 좌표를 못 구함 (호모그래피/왜곡 보정 확인 필요)";
    } else {
      if (!robot_world_checked) {
        robot_world_checked = true;
        robot_world_ready = homography_.PixelToWorldMarker(
            channel, robot->cx, robot->cy, &robot_wx, &robot_wy);
      }
      if (robot_world_ready) {
        distance_mm = hypot(fwx - robot_wx, fwy - robot_wy);
        have_distance = true;
      } else {
        why = "로봇 마커의 월드 좌표를 못 구함 (호모그래피/마커평면 미준비)";
      }
    }

    const ProximityGuard::State st = have_distance
        ? proximity_guard_[channel].Update(distance_mm, now_ms)
        : proximity_guard_[channel].Hold();
#if ENABLE_WISEAI_FRAME_LOG
    if (have_distance) {
      printf("[ArucoPosePNM] WiseAI ch=%d proximity dist_mm=%.0f state=%s\n",
             channel, distance_mm, ProximityStateName(st));
    } else {
      printf("[ArucoPosePNM] WiseAI ch=%d proximity 거리 계산 불가 (%s) state=%s(유지)\n",
             channel, why, ProximityStateName(st));
    }
    wrote_log = true;
#else
    (void)st;
    (void)why;
#endif
  }

  // IVA area rule events (object entered/exited/intruded a polygon set via
  // IVA_SYNC + tools/iva_push.sh). The parser and Enter/Exit vocabulary were
  // confirmed against live captures on 2026-08-19. Every event is forwarded,
  // not just Enter, so the receiver retains the camera's complete rule state.
  // Sent over the CONTROL line
  // (pose_sender_send_control_line), not the drop-tolerant realtime one
  // CAM_POSE uses: an alarm-triggering event should survive a momentarily
  // busy link, unlike a pose sample where the next frame supersedes a
  // dropped one anyway.
  std::vector<WiseAiIvaAreaEvent> iva_events;
  if (metadata.find("IvaArea") != std::string::npos)
    ParseWiseAiIvaAreaEvents(metadata, &iva_events);
  for (const auto& e : iva_events) {
    // The bbox that goes with this event's object_id, if this channel's
    // bbox stream has seen it recently -- see RecentWiseAiObjectBbox()'s
    // comment. An IVA_EVENT never carries a position of its own (WiseAI's
    // schema doesn't have one), so this is the only way to answer "where"
    // for whatever's listening downstream (an alarm UI wanting to draw the
    // box, say). Absent, not zeros, when there is no recent sighting to
    // match -- a bbox of all-0 reads as a real (if degenerate) box to
    // anything downstream that isn't reading carefully, and "top-left
    // corner" is a specific and wrong claim to make about an object this
    // channel hasn't actually located.
    float bl = 0.0f, bt = 0.0f, br = 0.0f, bb = 0.0f;
    const bool have_bbox =
        RecentWiseAiObjectBbox(channel, e.object_id.c_str(), now_ms, &bl, &bt, &br, &bb);

    char json[512];
    int w = snprintf(json, sizeof(json),
                     "{\"type\":\"IVA_EVENT\",\"ch\":%d,\"rule\":\"%.64s\",\"object_id\":\"%.32s\","
                     "\"action\":\"%.32s\",\"state\":%s",
                     channel, e.rule_name.c_str(), e.object_id.c_str(), e.action.c_str(),
                     e.state ? "true" : "false");
    if (have_bbox && w > 0 && w < (int)sizeof(json)) {
      w += snprintf(json + w, sizeof(json) - w,
                    ",\"left\":%.1f,\"top\":%.1f,\"right\":%.1f,\"bottom\":%.1f",
                    bl, bt, br, bb);
    }
    // WiseAI's own decision time, same clock as ZONE_EVENT's t_ms -- which is
    // what lets the two verdicts be compared directly (see iva_zone_'s
    // comment for why they disagree at all). Absent, not 0, when unparseable.
    if (e.utc_ms != 0 && w > 0 && w < (int)sizeof(json)) {
      w += snprintf(json + w, sizeof(json) - w, ",\"t_ms\":%ld", e.utc_ms);
    }
    if (w > 0 && w < (int)sizeof(json)) snprintf(json + w, sizeof(json) - w, "}");

    printf("[ArucoPosePNM] %s\n", json);
    wrote_log = true;
    pose_sender_send_control_line(json);
  }

  if (wrote_log) fflush(stdout);
}

/**
 * Record that marker `d.id` was just seen at `d.corners2d` on this channel.
 *
 * A same-id match always wins; failing that, the first empty slot; failing
 * that, whichever entry is oldest. Eviction-by-age is deliberate: a marker
 * not seen in a while is the one least likely to still be a registered
 * anchor in play, so it is the right one to make room for a new arrival.
 */
void SampleComponent::RememberMarker(int ch, const ArucoProcessor::Detection& d, long now_ms) {
  if (d.corners2d.size() < 4) return;
  RecentMarker* target = NULL;
  for (int s = 0; s < kMaxRecentMarkers; ++s)
    if (recent_marker_[ch][s].id == d.id) { target = &recent_marker_[ch][s]; break; }
  if (!target) {
    long oldest_ms = LONG_MAX;
    for (int s = 0; s < kMaxRecentMarkers; ++s) {
      RecentMarker& rm = recent_marker_[ch][s];
      if (rm.id < 0) { target = &rm; break; }
      if (rm.seen_ms < oldest_ms) { oldest_ms = rm.seen_ms; target = &rm; }
    }
  }
  target->id = d.id;
  for (int k = 0; k < 4; ++k) target->corners[k] = d.corners2d[k];
  MarkerPoints(d, &target->cx, &target->cy, NULL, NULL);
  target->seen_ms = now_ms;
}

/**
 * Record `d`'s bbox as "seen just now" for its ObjectId on this channel.
 *
 * Same same-id-wins / first-empty / oldest-evicted order as RememberMarker()
 * -- see recent_wiseai_obj_'s comment in the header for why this cache
 * exists at all. A blank object_id (WiseAI sent no ObjectId, or the XML
 * didn't parse it) is not worth caching -- there is nothing an IVA_EVENT
 * could ever match it against, so it would only occupy a slot for free.
 */
SampleComponent::ZoneEdge SampleComponent::RememberWiseAiObject(
    int ch, const WiseAiDetection& d, long now_ms, int zone_level) {
  if (d.object_id.empty() || d.object_id.size() >= sizeof(RecentWiseAiObject::object_id))
    return kZoneNoChange;
  RecentWiseAiObject* target = NULL;
  for (int s = 0; s < kMaxRecentWiseAiObjects; ++s)
    if (d.object_id == recent_wiseai_obj_[ch][s].object_id) { target = &recent_wiseai_obj_[ch][s]; break; }
  const bool reused = (target != NULL);
  if (!target) {
    long oldest_ms = LONG_MAX;
    for (int s = 0; s < kMaxRecentWiseAiObjects; ++s) {
      RecentWiseAiObject& ro = recent_wiseai_obj_[ch][s];
      if (ro.object_id[0] == '\0') { target = &ro; break; }
      if (ro.seen_ms < oldest_ms) { oldest_ms = ro.seen_ms; target = &ro; }
    }
  }
  // Edge detection, before the slot is overwritten. Three cases produce no
  // edge on purpose: an object we have not judged before (a track that BEGINS
  // inside the zone never "entered" it -- it appeared there, which is a
  // different thing and would otherwise fire an alarm for every new track over
  // the zone), a frame we cannot judge (zone_now < 0), and a slot recycled
  // from a different object, whose stored state belongs to that other track.
  //
  // 저장하는 값이 "안/밖"이 아니라 단계(0 정상 · 1 주의 · 2 접근금지 · 3 존 내부)
  // 라는 게 핵심이다. 경보가 울리는 지점은 zone_alarm_level_ 이 정하고, 엣지는
  // 그 임계값을 넘나들 때만 난다 -- 주의 밴드를 서성이는 사람이 경보를 연발로
  // 울리지 않으면서, 접근금지선을 넘는 순간은 놓치지 않는다.
  ZoneEdge edge = kZoneNoChange;
  const signed char prev = reused ? target->zone_state : (signed char)-1;
  if (zone_level >= 0 && prev >= 0) {
    const bool was = (prev >= (signed char)zone_alarm_level_);
    const bool now = (zone_level >= zone_alarm_level_);
    if (was != now) edge = now ? kZoneEntered : kZoneExited;
  }

  CopyUtf8(target->object_id, sizeof(target->object_id), d.object_id.c_str());
  target->left = d.left;
  target->top = d.top;
  target->right = d.right;
  target->bottom = d.bottom;
  target->seen_ms = now_ms;
  // A frame we could not judge parks the previous verdict rather than
  // clearing it, so that syncing a zone mid-track resumes from what we knew.
  if (zone_level >= 0) target->zone_state = (signed char)zone_level;
  else if (!reused) target->zone_state = -1;
  return edge;
}

/**
 * Same ray-cast-then-nearest-edge as IvaZoneDistancePx(), on the world copy of
 * the polygon. Separate function rather than a shared templated one because
 * the two differ in what they mean, not just in which array they read: the
 * pixel version answers "where on the screen", this one answers "how far in
 * millimetres", and only the second can be compared against a margin.
 */
float SampleComponent::IvaZoneWorldDistanceMm(int ch, double wx, double wy) const {
  if (ch < 0 || ch >= kMaxChannels) return -1.0f;
  const IvaZone& z = iva_zone_[ch];
  if (z.n < 3 || !z.have_world) return -1.0f;

  bool in = false;
  for (int i = 0, j = z.n - 1; i < z.n; j = i++) {
    const double xi = z.wx[i], yi = z.wy[i];
    const double xj = z.wx[j], yj = z.wy[j];
    if (((yi > wy) != (yj > wy)) &&
        (wx < (xj - xi) * (wy - yi) / (yj - yi) + xi))
      in = !in;
  }
  if (in) return 0.0f;

  float best = -1.0f;
  for (int i = 0, j = z.n - 1; i < z.n; j = i++) {
    const double ax = z.wx[j], ay = z.wy[j];
    const double vx = z.wx[i] - ax, vy = z.wy[i] - ay;
    const double ux = wx - ax, uy = wy - ay;
    const double len2 = vx * vx + vy * vy;
    double t = (len2 > 0.0) ? ((ux * vx + uy * vy) / len2) : 0.0;
    if (t < 0.0) t = 0.0;
    if (t > 1.0) t = 1.0;
    const double dx = wx - (ax + t * vx), dy = wy - (ay + t * vy);
    const float d = (float)sqrt(dx * dx + dy * dy);
    if (best < 0.0f || d < best) best = d;
  }
  return best;
}

/**
 * Send one band outline: the set of floor points exactly `margin_mm` outside
 * the zone, projected back to raw sensor pixels so the dashboard can draw it
 * without doing any geometry.
 *
 * The offset is a true Minkowski sum with a disc, not a mitre: at each vertex
 * the boundary is an ARC of radius margin_mm, and mitring instead would send
 * the corner shooting off to infinity as the angle sharpens. Arcs also happen
 * to be what the band physically is -- the set of points within `margin_mm` of
 * the polygon.
 *
 * Perspective is why this is computed here and not in the browser: the band is
 * a constant width on the FLOOR, so on screen it is wide near the camera and
 * narrow far from it. Only WorldToPixel() knows that, and it already exists.
 */
static void SendOneBand(int ch, const char* kind, int margin_mm,
                        const HomographyMapper& hg,
                        const float* wx, const float* wy, int n) {
  // Outward direction depends on winding, so measure it (shoelace) rather than
  // assuming: a hull built from anchors can come out either way round.
  double area2 = 0.0;
  for (int i = 0, j = n - 1; i < n; j = i++)
    area2 += (double)wx[j] * wy[i] - (double)wx[i] * wy[j];
  const double sgn = (area2 >= 0.0) ? 1.0 : -1.0;

  char json[1400];
  int w = snprintf(json, sizeof(json),
                   "{\"type\":\"ZONE_BAND\",\"ch\":%d,\"kind\":\"%s\",\"mm\":%d,"
                   "\"points\":[", ch, kind, margin_mm);
  bool first = true;
  const int kArc = 3;  // arc samples per corner -- enough to read as round
  for (int i = 0; i < n; ++i) {
    const int prev = (i + n - 1) % n, next = (i + 1) % n;
    // Outward normals of the two edges meeting at vertex i.
    double e1x = wx[i] - wx[prev], e1y = wy[i] - wy[prev];
    double e2x = wx[next] - wx[i], e2y = wy[next] - wy[i];
    const double l1 = sqrt(e1x * e1x + e1y * e1y), l2 = sqrt(e2x * e2x + e2y * e2y);
    if (l1 <= 0.0 || l2 <= 0.0) continue;
    const double n1x = sgn * (e1y / l1), n1y = -sgn * (e1x / l1);
    const double n2x = sgn * (e2y / l2), n2y = -sgn * (e2x / l2);
    double a1 = atan2(n1y, n1x), a2 = atan2(n2y, n2x);
    double sweep = a2 - a1;
    while (sweep > M_PI) sweep -= 2.0 * M_PI;
    while (sweep < -M_PI) sweep += 2.0 * M_PI;
    for (int k = 0; k <= kArc; ++k) {
      const double a = a1 + sweep * ((double)k / kArc);
      float px = 0.0f, py = 0.0f;
      if (!hg.WorldToPixel(ch, wx[i] + margin_mm * cos(a),
                           wy[i] + margin_mm * sin(a), &px, &py))
        continue;  // 지평선 너머 등 — 그 점만 건너뛴다
      if (w <= 0 || w >= (int)sizeof(json) - 32) break;
      w += snprintf(json + w, sizeof(json) - w, "%s{\"x\":%.1f,\"y\":%.1f}",
                    first ? "" : ",", px, py);
      first = false;
    }
  }
  if (w > 0 && w < (int)sizeof(json) - 4) snprintf(json + w, sizeof(json) - w, "]}");
  if (!first) pose_sender_send_control_line(json);
}

void SampleComponent::FillZoneWorld(int ch) {
  if (ch < 0 || ch >= kMaxChannels) return;
  IvaZone& z = iva_zone_[ch];
  z.have_world = false;
  if (z.n < 3) return;
  // All-or-nothing: a zone with some vertices mapped and others not is not a
  // polygon, and half a polygon would silently produce wrong distances.
  for (int i = 0; i < z.n; ++i) {
    double wx = 0.0, wy = 0.0;
    if (!homography_.PixelToWorld(ch, z.px[i], z.py[i], &wx, &wy, NULL)) return;
    z.wx[i] = (float)wx;
    z.wy[i] = (float)wy;
  }
  z.have_world = true;
}

void SampleComponent::SendZoneBands(int ch) {
  if (!zone_bands_on_ || ch < 0 || ch >= kMaxChannels) return;
  const IvaZone& z = iva_zone_[ch];
  if (z.n < 3 || !z.have_world) return;
  SendOneBand(ch, "danger", zone_danger_mm_, homography_, z.wx, z.wy, z.n);
  SendOneBand(ch, "warn", zone_warn_mm_, homography_, z.wx, z.wy, z.n);
}

bool SampleComponent::HandleZoneBands(const char* cmd) {
  if (strncmp(cmd, "ZONE_BANDS", 10) != 0) return false;
  int on = -1;
  if (sscanf(cmd + 10, "%d", &on) == 1) zone_bands_on_ = (on != 0);
  printf("[ArucoPosePNM] ZONE_BANDS %s (danger %dmm, warn %dmm)\n",
         zone_bands_on_ ? "on" : "off", zone_danger_mm_, zone_warn_mm_);
  fflush(stdout);
  char json[128];
  snprintf(json, sizeof(json),
           "{\"type\":\"ZONE_BANDS\",\"on\":%s,\"danger_mm\":%d,\"warn_mm\":%d}",
           zone_bands_on_ ? "true" : "false", zone_danger_mm_, zone_warn_mm_);
  pose_sender_send_control_line(json);
  for (int c = 0; c < kMaxChannels; ++c) SendZoneBands(c);
  return true;
}

/**
 * ZONE_ALARM_LEVEL <2|3> — where the alarm line sits.
 *
 * 3 = only once the foot is inside the polygon. 2 = also the no-approach band
 * (the default, and the point of having bands at all). 1 would alarm on the
 * caution band too, which is allowed but will fire a lot.
 *
 * Refuses 0: "alarm whenever a person exists" is never what anyone means, and
 * accepting it would turn every detection into an Enter event.
 */
bool SampleComponent::HandleZoneAlarmLevel(const char* cmd) {
  if (strncmp(cmd, "ZONE_ALARM_LEVEL", 16) != 0) return false;
  int lv = -1;
  if (sscanf(cmd + 16, "%d", &lv) == 1) {
    if (lv < 1 || lv > 3) {
      printf("[ArucoPosePNM] ZONE_ALARM_LEVEL %d 거부 — 1..3 이어야 함 (현재 %d)\n",
             lv, zone_alarm_level_);
    } else {
      zone_alarm_level_ = lv;
      // 임계값이 바뀌면 지금 기억하고 있는 단계들은 옛 기준의 것이다. 비우지
      // 않으면 임계값을 낮춘 순간 이미 안에 있던 사람들이 일제히 Enter 로
      // 잡혀 경보가 한꺼번에 터진다.
      for (int c = 0; c < kMaxChannels; ++c)
        for (int s = 0; s < kMaxRecentWiseAiObjects; ++s)
          recent_wiseai_obj_[c][s].zone_state = -1;
      printf("[ArucoPosePNM] ZONE_ALARM_LEVEL %d (%s부터 경보)\n", lv,
             (lv == 3) ? "존 내부" : (lv == 2) ? "접근금지 밴드" : "주의 밴드");
    }
  } else {
    printf("[ArucoPosePNM] ZONE_ALARM_LEVEL 현재 %d\n", zone_alarm_level_);
  }
  fflush(stdout);
  char json[96];
  snprintf(json, sizeof(json), "{\"type\":\"ZONE_ALARM_LEVEL\",\"level\":%d}",
           zone_alarm_level_);
  pose_sender_send_control_line(json);
  return true;
}

bool SampleComponent::HandleZoneMargin(const char* cmd) {
  if (strncmp(cmd, "ZONE_MARGIN", 11) != 0) return false;
  int dmm = -1, wmm = -1;
  if (sscanf(cmd + 11, "%d %d", &dmm, &wmm) == 2) {
    // Refuse rather than reorder: a caller that had them backwards meant
    // something, and silently swapping hides the mistake.
    if (dmm < 0 || wmm <= dmm || wmm > 20000) {
      printf("[ArucoPosePNM] ZONE_MARGIN 거부 — 0 <= danger < warn <= 20000 이어야 함 "
             "(받은 값 %d, %d)\n", dmm, wmm);
    } else {
      zone_danger_mm_ = dmm;
      zone_warn_mm_ = wmm;
      printf("[ArucoPosePNM] ZONE_MARGIN danger %dmm, warn %dmm\n", dmm, wmm);
    }
  } else {
    printf("[ArucoPosePNM] ZONE_MARGIN 현재 danger %dmm, warn %dmm\n",
           zone_danger_mm_, zone_warn_mm_);
  }
  fflush(stdout);
  char json[128];
  snprintf(json, sizeof(json),
           "{\"type\":\"ZONE_BANDS\",\"on\":%s,\"danger_mm\":%d,\"warn_mm\":%d}",
           zone_bands_on_ ? "true" : "false", zone_danger_mm_, zone_warn_mm_);
  pose_sender_send_control_line(json);
  for (int c = 0; c < kMaxChannels; ++c) SendZoneBands(c);
  return true;
}

/**
 * DET_STREAM <0|1> — start/stop the per-detection position feed.
 *
 * The dashboard turns this on when its overlay opens and off when it closes,
 * so the feed exists only while something is drawing it. Answering with the
 * current value when given no argument lets the caller ask as well as set.
 */
bool SampleComponent::HandleDetStream(const char* cmd) {
  if (strncmp(cmd, "DET_STREAM", 10) != 0) return false;
  int on = -1;
  if (sscanf(cmd + 10, "%d", &on) == 1) {
    det_stream_ = (on != 0);
    printf("[ArucoPosePNM] DET_STREAM %s\n", det_stream_ ? "on" : "off");
  } else {
    printf("[ArucoPosePNM] DET_STREAM 현재 %s\n", det_stream_ ? "on" : "off");
  }
  fflush(stdout);
  char json[96];
  snprintf(json, sizeof(json), "{\"type\":\"DET_STREAM\",\"on\":%s}",
           det_stream_ ? "true" : "false");
  pose_sender_send_control_line(json);
  return true;
}

/**
 * Ray casting: count how many polygon edges a ray from (x, y) crosses.
 *
 * Odd = inside. Written out rather than calling cv::pointPolygonTest for the
 * same reason ConvexHull2f() is written out rather than calling
 * cv::convexHull() -- see that function's comment for the load-time crash that
 * rule exists to avoid. Fixed arrays only, no std::vector, no cv:: algorithm.
 */
float SampleComponent::IvaZoneDistancePx(int ch, float x, float y) const {
  if (ch < 0 || ch >= kMaxChannels) return -1.0f;
  const IvaZone& z = iva_zone_[ch];
  if (z.n < 3) return -1.0f;

  bool in = false;
  for (int i = 0, j = z.n - 1; i < z.n; j = i++) {
    const float xi = z.px[i], yi = z.py[i];
    const float xj = z.px[j], yj = z.py[j];
    if (((yi > y) != (yj > y)) &&
        (x < (xj - xi) * (y - yi) / (yj - yi) + xi))
      in = !in;
  }
  if (in) return 0.0f;

  // Outside: distance to the nearest edge SEGMENT (not the infinite line --
  // clamping t to [0,1] is what makes a point past an edge's end measure to
  // the corner, which is the real distance).
  float best = -1.0f;
  for (int i = 0, j = z.n - 1; i < z.n; j = i++) {
    const float ax = z.px[j], ay = z.py[j];
    const float vx = z.px[i] - ax, vy = z.py[i] - ay;
    const float wx = x - ax, wy = y - ay;
    const float len2 = vx * vx + vy * vy;
    float t = (len2 > 0.0f) ? ((wx * vx + wy * vy) / len2) : 0.0f;
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    const float dx = x - (ax + t * vx), dy = y - (ay + t * vy);
    const float d = sqrtf(dx * dx + dy * dy);
    if (best < 0.0f || d < best) best = d;
  }
  return best;
}

/**
 * ZONE_RADIUS <div> — resize the foot disc at runtime.
 *
 * Reports the current value when called with no parseable number, so the
 * operator can ask as well as set. Refuses out-of-range rather than clamping,
 * for the reason HandleHoldMs() spells out: silently substituting a different
 * number leaves the operator believing the machine agreed.
 */
bool SampleComponent::HandleZoneRadius(const char* cmd) {
  if (strncmp(cmd, "ZONE_RADIUS", 11) != 0) return false;
  int div = -1;
  if (sscanf(cmd + 11, "%d", &div) == 1) {
    if (div < 1 || div > 64) {
      printf("[ArucoPosePNM] ZONE_RADIUS %d 거부 — 1..64 범위여야 함 (현재 %d)\n",
             div, zone_radius_div_);
    } else {
      const int old = zone_radius_div_;
      zone_radius_div_ = div;
      printf("[ArucoPosePNM] ZONE_RADIUS %d -> %d (반경 = bbox 폭 / %d)\n",
             old, div, div);
    }
  } else {
    printf("[ArucoPosePNM] ZONE_RADIUS 현재 %d (반경 = bbox 폭 / %d)\n",
           zone_radius_div_, zone_radius_div_);
  }
  fflush(stdout);
  return true;
}

bool SampleComponent::RecentWiseAiObjectBbox(int ch, const char* object_id, long now_ms,
                                             float* left, float* top, float* right,
                                             float* bottom) const {
  if (ch < 0 || ch >= kMaxChannels || object_id == NULL || object_id[0] == '\0') return false;
  for (int s = 0; s < kMaxRecentWiseAiObjects; ++s) {
    const RecentWiseAiObject& ro = recent_wiseai_obj_[ch][s];
    if (ro.object_id[0] == '\0') continue;
    if (strcmp(ro.object_id, object_id) != 0) continue;
    if (now_ms - ro.seen_ms > WISEAI_OBJECT_HOLD_MS) return false;
    *left = ro.left; *top = ro.top; *right = ro.right; *bottom = ro.bottom;
    return true;
  }
  return false;
}

int SampleComponent::FillRecentMarkers(int ch, int* ids, float* cxs, float* cys, int n,
                                       int cap, long now_ms) const {
  for (int s = 0; s < kMaxRecentMarkers && n < cap; ++s) {
    const RecentMarker& rm = recent_marker_[ch][s];
    if (rm.id < 0) continue;
    if (now_ms - rm.seen_ms > marker_hold_ms_) continue;
    bool already = false;
    for (int k = 0; k < n; ++k)
      if (ids[k] == rm.id) { already = true; break; }
    if (already) continue;
    ids[n] = rm.id;
    cxs[n] = rm.cx;
    cys[n] = rm.cy;
    ++n;
  }
  return n;
}

/**
 * this frame's `fresh` detections, unioned with anything left in the
 * recent-sighting cache that is not already in `fresh` and is still within
 * marker_hold_ms_. See the doc comment in sample_component.h.
 */
void SampleComponent::BuildHeldDets(int ch, const std::vector<ArucoProcessor::Detection>& fresh,
                                    long now_ms, std::vector<ArucoProcessor::Detection>* out) const {
  *out = fresh;
  for (int s = 0; s < kMaxRecentMarkers; ++s) {
    const RecentMarker& rm = recent_marker_[ch][s];
    if (rm.id < 0) continue;
    if (now_ms - rm.seen_ms > marker_hold_ms_) continue;
    bool already = false;
    for (size_t k = 0; k < fresh.size(); ++k)
      if (fresh[k].id == rm.id) { already = true; break; }
    if (already) continue;
    ArucoProcessor::Detection d;
    d.id = rm.id;
    d.corners2d.assign(rm.corners, rm.corners + 4);
    out->push_back(d);
  }
}

/**
 * Stream this frame's detections to the dashboard (IF-TCP-003 wire format).
 *
 * One CAM_POSE line per detected marker; on a miss a confidence:0 heartbeat is
 * still sent so the server can tell "camera alive, marker lost" apart from
 * "camera/link dead". `seq_` is a FRAME counter — several packets can share it.
 */
void SampleComponent::SendPosePackets(int ch,
                                      const std::vector<ArucoProcessor::Detection>& fresh,
                                      long t_frame_ms, int frame_w, int frame_h,
                                      long t_capture, long queue_ms) {
  char json[768];
  ++seq_[ch];

  // No server: stop before doing the work, not after.
  //
  // The camera is a standalone instrument. Detection, the dashboard, the
  // calibration sessions and every command all work with the pose link down,
  // and that is deliberate — the RPi is a consumer of this app, not a
  // dependency of it. What was NOT deliberate is how much this function kept
  // spending on output nobody could receive.
  //
  // With markers in view that is, per second: ~340 JSON packets formatted and
  // handed to a sender that drops them, and — since the world coordinates
  // landed — ~680 undistortPoints + projectPoints round trips to compute
  // positions that go straight in the bin. All of it on the one thread that
  // also runs detection, while the operator is looking at a page that has just
  // become slower for no reason they can see.
  //
  // seq_ is still advanced above: it counts FRAMES DETECTED, which is a fact
  // about this camera and is what /status reports as the effective rate. It
  // must not depend on whether anyone was listening.
  if (!pose_sender_is_connected()) return;

  const double detMs = aruco_[ch] ? aruco_[ch]->lastDetectMs() : -1.0;

  // "ch" identifies the lens. Consumers that predate multi-channel simply
  // ignore the extra field and see one merged stream, which is why it is added
  // rather than replacing anything.
  if (fresh.empty()) {
    snprintf(json, sizeof(json),
             "{\"type\":\"CAM_POSE\",\"ch\":%d,\"seq\":%lu,\"t\":%ld,\"t_frame\":%ld,"
             "\"t_capture\":%ld,\"queue_ms\":%ld,"
             "\"w\":%d,\"h\":%d,\"t_det\":%.1f,\"confidence\":0,\"corners\":[]}",
             ch, seq_[ch], epoch_ms(), t_frame_ms, t_capture, queue_ms, frame_w, frame_h, detMs);
    pose_sender_send_line(json);
    return;
  }

  for (size_t i = 0; i < fresh.size(); ++i) {
    const ArucoProcessor::Detection& d = fresh[i];
    if (d.corners2d.size() < 4) continue;

    int len = snprintf(json, sizeof(json),
                       "{\"type\":\"CAM_POSE\",\"ch\":%d,\"seq\":%lu,\"t\":%ld,\"t_frame\":%ld,"
                       "\"t_capture\":%ld,\"queue_ms\":%ld,"
                       "\"w\":%d,\"h\":%d,\"t_det\":%.1f,\"id\":%d,\"confidence\":1.0,"
                       "\"corners\":["
                       "{\"x\":%.2f,\"y\":%.2f},{\"x\":%.2f,\"y\":%.2f},"
                       "{\"x\":%.2f,\"y\":%.2f},{\"x\":%.2f,\"y\":%.2f}]}",
                       ch, seq_[ch], epoch_ms(), t_frame_ms, t_capture, queue_ms,
                       frame_w, frame_h, detMs, d.id,
                       d.corners2d[0].x, d.corners2d[0].y, d.corners2d[1].x, d.corners2d[1].y,
                       d.corners2d[2].x, d.corners2d[2].y, d.corners2d[3].x, d.corners2d[3].y);
    if (len <= 0 || len >= (int)sizeof(json)) continue;

    // World millimetres, through the MARKER plane — the one place in the frame
    // path that costs real work, and the reason the whole homography exists.
    //
    // Absent, not null, when this lens cannot produce it. A `world` of
    // {0,0,0} or null reads as a position to anything downstream that is not
    // reading carefully, and "the robot is at the origin" is a specific and
    // wrong claim. A missing key cannot be misread that way.
    //
    // Raw corner pixels stay on the packet either way. They are the server's
    // only means of cross-checking the camera's own mapping, and dropping them
    // once world coordinates existed would make the two impossible to compare
    // exactly when they disagreed.
    double wcx = 0.0, wcy = 0.0, wtx = 0.0, wty = 0.0;
    float cx = 0.0f, cy = 0.0f, tx = 0.0f, ty = 0.0f;
    MarkerPoints(d, &cx, &cy, &tx, &ty);  // same definition the fit averaged

    if (homography_.PixelToWorldMarker(ch, cx, cy, &wcx, &wcy) &&
        homography_.PixelToWorldMarker(ch, tx, ty, &wtx, &wty)) {
      // Both points go to world FIRST, and the angle is measured there.
      //
      // Measuring the angle in pixels and rotating it afterwards is the
      // mistake this ordering exists to prevent: a homography is not a
      // rotation, so it does not preserve angles. Under perspective a marker
      // at the edge of the frame is sheared, and its pixel-space heading is
      // off by an amount that varies with WHERE the robot is — a bias that
      // moves as the robot drives, which is the hardest kind to notice.
      const double theta_deg = atan2(wty - wcy, wtx - wcx) * 180.0 / 3.14159265358979323846;
      const int w = snprintf(json + len - 1, sizeof(json) - (size_t)len + 1,
                             ",\"world\":{\"x\":%.1f,\"y\":%.1f,\"theta\":%.2f}}",
                             wcx, wcy, theta_deg);
      if (w > 0 && (size_t)(len - 1 + w) < sizeof(json)) len = len - 1 + w;
    }

    pose_sender_send_line(json);

#if ENABLE_CENTRAL_TLS_STREAM
    // The central server owns undistort + H_marker; it wants only the raw
    // pixels, in its own POS schema, for the one marker that is the robot.
    // approximate frames never reach here (caller substitutes kNoDetections),
    // so these corners are always full-resolution.
    if (g_central_pos_enabled && d.id == g_central_marker_id) {
      const float central_corners[4][2] = {
          {d.corners2d[0].x, d.corners2d[0].y},
          {d.corners2d[1].x, d.corners2d[1].y},
          {d.corners2d[2].x, d.corners2d[2].y},
          {d.corners2d[3].x, d.corners2d[3].y},
      };
      // +1: docs/PROTOCOL.md's channel convention (v0.4, "채널 규약") is
      // explicit that every wire-facing ch is 1-based, matching CH1..CH4 on
      // the camera's own web UI -- `ch` here is this app's internal 0-based
      // array index and was going out unconverted. The server's default (and
      // POS's) active channel is 1, so channel 0's robot POS packets were
      // being silently dropped by the server's active-channel gate every
      // time -- found 2026-08-11 cross-checking against the live
      // docs/PROTOCOL.md on the Pi, not just the historical action-items
      // doc (docs/08.06/CCTV_ACTION_ITEMS_20260806.md C-1), whose own example
      // code left the base/offset unstated.
      central_tls_sender_send_pos(ch + 1, central_corners);
    }
#endif
  }
}

/**
 * Report the tracker's whole configuration. Shared by DYNROI and DYNROI_IDS so
 * the dashboard never has to merge two half-answers -- every ack is complete.
 */
void SampleComponent::ReportDynRoi() const {
  // One ack per channel so the dashboard can show each lens's state.
  for (int c = 0; c < kMaxChannels; ++c) {
    const std::vector<int>& ids = dynroi_[c].trackIds();
    char idbuf[128];
    int n = 0;
    idbuf[0] = '\0';
    for (size_t i = 0; i < ids.size(); ++i) {
      const int w = snprintf(idbuf + n, sizeof(idbuf) - n, (i == 0) ? "%d" : ",%d", ids[i]);
      if (w <= 0 || (size_t)(n + w) >= sizeof(idbuf)) break;  // truncate, never overflow
      n += w;
    }
    char json[256];
    snprintf(json, sizeof(json),
             "{\"type\":\"DYNROI\",\"ch\":%d,\"enabled\":%d,\"margin\":%d,"
             "\"adaptive\":true,\"max_miss\":%d,\"track_ids\":[%s]}",
             c, dynroi_[c].enabled() ? 1 : 0, dynroi_[c].margin(), dynroi_[c].maxMiss(), idbuf);
    pose_sender_send_control_line(json);
  }
}

/**
 * "DYNROI_IDS [id ...]" -- restrict tracking to these marker ids.
 * No ids at all means "track every marker", which is the default.
 *
 * MUST be dispatched BEFORE HandleDynRoi(): that one matches on the substring
 * "DYNROI", which this name contains, so the looser handler would swallow this
 * command and parse "_IDS ..." as its own numeric arguments.
 */
bool SampleComponent::HandleDynRoiIds(const char* cmd) {
  const char* p = strstr(cmd, "DYNROI_IDS");
  if (p == NULL) return false;
  p += 10;  // strlen("DYNROI_IDS")

  std::vector<int> ids;
  while (*p != '\0' && ids.size() < DYNROI_MAX_TRACK_IDS) {
    char* end = NULL;
    const long v = strtol(p, &end, 10);
    if (end == p) break;  // no more numbers
    bool dup = false;
    for (size_t i = 0; i < ids.size(); ++i)
      if (ids[i] == (int)v) { dup = true; break; }
    if (!dup && v >= 0) ids.push_back((int)v);
    p = end;
  }

  // No per-channel form on the wire yet: apply to every lens.
  for (int c = 0; c < kMaxChannels; ++c) {
    dynroi_[c].setTrackIds(ids);
    // The box was just reset to SEARCH, so hand the ROI back to the manual rect
    // until the filter locks on again.
    if (aruco_[c]) aruco_[c]->setRoi(manual_roi_[c]);
  }

  ReportDynRoi();
  printf("[ArucoPosePNM] dynroi track ids = %d (all channels)\n", (int)ids.size());
  fflush(stdout);
  return true;
}

/**
 * "DYNROI_CH <ch> <0|1> [maxMargin] [maxMiss]" -- same as DYNROI, one lens.
 *
 * A separate command rather than a channel argument on DYNROI: the RPi
 * dashboard already sends "DYNROI 1 120 8", where the 1 is ON. Moving the
 * channel into that first slot would silently turn every existing button into
 * something else. The two coexist and the per-channel one is matched first
 * (its name contains "DYNROI").
 *
 * Worth having because the four lenses do different jobs -- this camera points
 * them in four directions. A lens tracking the robot wants the box; a lens that
 * has to see every floor anchor for homography must not have it. One global
 * flag forces choosing between them.
 */
bool SampleComponent::HandleDynRoiCh(const char* cmd) {
  const char* p = strstr(cmd, "DYNROI_CH");
  if (p == NULL) return false;
  p += 9;  // strlen("DYNROI_CH")

  int ch = -1, on = 0, margin = -1, maxMiss = -1;
  const int got = sscanf(p, "%d %d %d %d", &ch, &on, &margin, &maxMiss);
  if (got >= 2 && ch >= 0 && ch < kMaxChannels) {
    // Omitted margin/maxMiss keep this channel's current values, so toggling
    // does not quietly reset tuning someone arrived at.
    if (margin < 0) margin = dynroi_[ch].margin();
    if (maxMiss < 0) maxMiss = dynroi_[ch].maxMiss();
    if (margin > 960) margin = 960;
    if (maxMiss > 60) maxMiss = 60;

    if (!dynroi_[ch].configure(on != 0, margin, maxMiss) && aruco_[ch])
      aruco_[ch]->setRoi(manual_roi_[ch]);  // tracker off -> manual ROI back

    printf("[ArucoPosePNM] ch%d dynroi %s margin=%d maxMiss=%d\n",
           ch, dynroi_[ch].enabled() ? "ON" : "OFF", margin, maxMiss);
    fflush(stdout);
  }

  ReportDynRoi();
  return true;
}

/**
 * "DYNROI <0|1> [maxMargin] [maxMiss]" -- toggle/tune the tracker at runtime,
 * on every lens at once. A bare "DYNROI" only reports the current state.
 *
 * That query/set split used to be missing here (fixed 2026-08-11): `on`
 * defaulted to 0 and sscanf silently left it there when the dashboard sent a
 * bare "DYNROI" to ask for a status refresh, so every refresh ran
 * configure(false, ...) on all four lenses -- turning dynROI off (and resetting
 * every tracker back to SEARCH even when it was already off) as a side effect
 * of asking. The dashboard does this on every visit to the marker-detection
 * tab (`showTab('raw')` sends a bare DYNROI), so in practice dynROI could not
 * stay on across a tab switch. Same shape as DYNROI_CH's `got >= 2` guard.
 */
bool SampleComponent::HandleDynRoi(const char* cmd) {
  const char* p = strstr(cmd, "DYNROI");
  if (p == NULL) return false;

  int on = 0, margin = dynroi_[0].margin(), maxMiss = dynroi_[0].maxMiss();
  const int got = sscanf(p + 6, "%d %d %d", &on, &margin, &maxMiss);  // 6 = strlen("DYNROI")
  if (got >= 1) {
    if (margin < 0) margin = 0;
    if (margin > 960) margin = 960;
    if (maxMiss < 0) maxMiss = 0;
    if (maxMiss > 60) maxMiss = 60;

    for (int c = 0; c < kMaxChannels; ++c) {
      if (!dynroi_[c].configure(on != 0, margin, maxMiss) && aruco_[c])
        aruco_[c]->setRoi(manual_roi_[c]);  // tracker off -> manual ROI back
    }
    printf("[ArucoPosePNM] dynroi %s margin=%d maxMiss=%d (all channels)\n",
           dynroi_[0].enabled() ? "ON" : "OFF", dynroi_[0].margin(), dynroi_[0].maxMiss());
    fflush(stdout);
  }

  ReportDynRoi();
  return true;
}

/**
 * Switch one lens's marker search on or off.
 *
 * Turning a channel off also resets its tracker: when it comes back it must
 * start from SEARCH rather than from a box built out of a frame that may be
 * minutes old.
 */
bool SampleComponent::SetDetectEnabled(int ch, bool on) {
  if (ch < 0 || ch >= kMaxChannels) return false;
  if (detect_enabled_[ch] == on) return true;

  detect_enabled_[ch] = on;
  // Switching a channel changes n for EVERY channel, so every outstanding wait
  // was computed against a denominator that no longer holds — the same reason
  // DUTY clears them, and it was missing here. Switching one lens off left the
  // other three sitting out up to 1.4 s of idle owed to a channel that had just
  // stopped asking for any, which reads as "DETECT 0 made the others slower".
  // Switching one on is the direction that matters more: their waits were set
  // for a smaller n, so without this the thread is briefly over-committed by
  // exactly the share the new channel is taking.
  for (int c = 0; c < kMaxChannels; ++c) detect_budget_until_ms_[c] = 0;
  if (!on) {
    dynroi_[ch].configure(dynroi_[ch].enabled(), dynroi_[ch].margin(), dynroi_[ch].maxMiss());
    if (aruco_[ch]) aruco_[ch]->setRoi(manual_roi_[ch]);
  }
  printf("[ArucoPosePNM] ch%d detect %s\n", ch, on ? "ON" : "OFF");
  fflush(stdout);
  return true;
}

/**
 * Set the SEARCH shrink factor for one lens. Clamped by ArucoProcessor; the
 * value stored here is what was asked for, so report the effective one.
 */
bool SampleComponent::SetSearchScale(int ch, int n) {
  if (ch < 0 || ch >= kMaxChannels) return false;
  if (n < 1) n = 1;
  if (n > 8) n = 8;
  search_scale_[ch] = n;
  printf("[ArucoPosePNM] ch%d search scale = 1/%d\n", ch, n);
  fflush(stdout);
  return true;
}

/** One DETECT ack per channel, same shape as ReportDynRoi(). */
/**
 * DUTY <pct> — set the detection budget; DUTY on its own just reports it.
 *
 * Global and not per channel because the budget IS shared: the four lenses
 * divide one thread's time, and the governor already splits it by how many are
 * switched on. A per-channel duty would be four numbers that only mean anything
 * added together, and adding to more than 100 would have to be refused — which
 * is the same constraint expressed less clearly.
 *
 * Out-of-range values are refused rather than clamped. Clamping answers "95" to
 * a request for 150 with no indication that anything was changed, and the
 * operator walks away believing the machine agreed with them.
 */
bool SampleComponent::HandleDuty(const char* cmd) {
  const char* p = strstr(cmd, "DUTY");
  if (p == NULL) return false;

  int pct = -1;
  if (sscanf(p + 4, "%d", &pct) == 1) {  // 4 = strlen("DUTY")
    if (pct < kDutyMin || pct > kDutyMax) {
      printf("[ArucoPosePNM] DUTY %d 거부 — %d..%d 범위여야 함 (현재 %d%%)\n", pct,
             kDutyMin, kDutyMax, detect_duty_pct_);
    } else {
      const int old = detect_duty_pct_;
      detect_duty_pct_ = pct;
      // Budgets already charged were computed against the OLD duty, so raising
      // the limit would otherwise not take effect until every channel had
      // served out a wait set under the stricter rule — up to 1.5 s of looking
      // like the command did nothing. Clearing them makes the change visible on
      // the next frame, which is what a person watching the page expects.
      for (int c = 0; c < kMaxChannels; ++c) detect_budget_until_ms_[c] = 0;
      printf("[ArucoPosePNM] DUTY %d%% -> %d%%\n", old, pct);
    }
    fflush(stdout);
  }

  ReportDuty();
  return true;
}

void SampleComponent::ReportDuty() const {
  char json[96];
  snprintf(json, sizeof(json), "{\"type\":\"DUTY\",\"pct\":%d,\"min\":%d,\"max\":%d}",
           detect_duty_pct_, kDutyMin, kDutyMax);
  pose_sender_send_control_line(json);
}

/**
 * "HOLD_MS <ms>" -- the flicker-hold window, at runtime. A bare "HOLD_MS"
 * just reports it, the same pattern as DUTY.
 *
 * Out-of-range values are refused rather than clamped, for the same reason
 * HandleDuty() refuses rather than clamps: silently answering "2000" to a
 * request for 50000 leaves the operator believing the machine agreed.
 */
bool SampleComponent::HandleHoldMs(const char* cmd) {
  const char* p = strstr(cmd, "HOLD_MS");
  if (p == NULL) return false;

  long ms = -1;
  if (sscanf(p + 7, "%ld", &ms) == 1) {  // 7 = strlen("HOLD_MS")
    if (ms < kHoldMsMin || ms > kHoldMsMax) {
      printf("[ArucoPosePNM] HOLD_MS %ld 거부 — %ld..%ld 범위여야 함 (현재 %ldms)\n", ms,
             kHoldMsMin, kHoldMsMax, marker_hold_ms_);
    } else {
      const long old = marker_hold_ms_;
      marker_hold_ms_ = ms;
      printf("[ArucoPosePNM] HOLD_MS %ldms -> %ldms\n", old, ms);
    }
    fflush(stdout);
  }

  ReportHoldMs();
  return true;
}

void SampleComponent::ReportHoldMs() const {
  char json[96];
  snprintf(json, sizeof(json), "{\"type\":\"HOLD_MS\",\"ms\":%ld,\"min\":%ld,\"max\":%ld}",
           marker_hold_ms_, kHoldMsMin, kHoldMsMax);
  pose_sender_send_control_line(json);
}

/**
 * CPU_STAT, throttled to kCpuReportIntervalMs. Ported from cctv_app's
 * report_cpu_if_due() (2026-08-11), but instead of re-deriving app-vs-system
 * CPU math from /proc/self/stat and /proc/stat by hand, this reuses the
 * figures BuildStatusJson() already computes for /status — cpu_pct_ (this
 * app against wall clock) and core_pct_[] (whole-camera load, one per core) —
 * by calling the same SampleRates() they come from. That function is self-
 * throttled to kStatsWindowMs regardless of who calls it or how often, so
 * calling it here on top of /status (if anything ever polls that) is safe.
 *
 * Why this needs to exist at all: this camera is normally driven over the
 * pose link only, and until now NOTHING there ever called SampleRates() —
 * only /status did. With no dashboard polling /status (this project's
 * pose-link-only design), cpu_pct_ sat at -1 ("not measurable yet") forever,
 * and the dashboard's CPU row had nothing to show.
 */
void SampleComponent::ReportCpu() {
  struct rusage ru;
  double cpu_s = 0.0;
  if (getrusage(RUSAGE_SELF, &ru) == 0) {
    cpu_s = (double)ru.ru_utime.tv_sec + (double)ru.ru_utime.tv_usec / 1e6 +
            (double)ru.ru_stime.tv_sec + (double)ru.ru_stime.tv_usec / 1e6;
  }
  SampleRates(epoch_ms(), cpu_s);
  if (cpu_pct_ < 0.0) return;  // no baseline yet (first ~second after boot)

  unsigned long heap_used = 0, heap_free = 0;
  heap_bytes(&heap_used, &heap_free);
  char json[256];
  int n = snprintf(json, sizeof(json),
           "{\"type\":\"CPU_STAT\",\"cpu_pct\":%.1f,\"cores\":%ld,"
           "\"rss_kb\":%ld,\"heap_in_use_kb\":%lu,\"core_pct\":[",
           cpu_pct_, (long)sysconf(_SC_NPROCESSORS_ONLN),
           current_rss_kb(), heap_used / 1024UL);
  for (int i = 0; i < core_n_ && n > 0 && n < (int)sizeof(json) - 16; ++i)
    n += snprintf(json + n, sizeof(json) - n, "%s%.1f", i ? "," : "", core_pct_[i]);
  if (n > 0 && n < (int)sizeof(json) - 2) snprintf(json + n, sizeof(json) - n, "]}");
  pose_sender_send_control_line(json);
}

void SampleComponent::ReportDetect() const {
  for (int c = 0; c < kMaxChannels; ++c) {
    char json[128];
    snprintf(json, sizeof(json), "{\"type\":\"DETECT\",\"ch\":%d,\"enabled\":%d,\"scale\":%d}",
             c, detect_enabled_[c] ? 1 : 0, search_scale_[c]);
    pose_sender_send_control_line(json);
  }
}

/**
 * "SCALE <ch> <n>" -- SEARCH shrink factor for one lens, 1 = full resolution.
 *
 * Same reasoning as DETECT for existing on the pose link at all: this is a
 * knob whose right value can only be found by sweeping it against the real
 * scene, and the HTTP route to it is unavailable exactly when the app is too
 * loaded — which is the situation the knob is for.
 */
bool SampleComponent::HandleScale(const char* cmd) {
  const char* p = strstr(cmd, "SCALE");
  if (p == NULL) return false;

  int ch = -1, n = 1;
  if (sscanf(p + 5, "%d %d", &ch, &n) == 2)  // 5 = strlen("SCALE")
    SetSearchScale(ch, n);

  ReportDetect();
  return true;
}

/**
 * "ARUCO_SCAN <ch> [<passes> [win]]" -- how many full-frame adaptive-
 * threshold passes detectMarkers runs on one lens (ArucoProcessor::
 * setScanPasses). Ported from cctv_app's single-lens ARUCO_SCAN (2026-08-10):
 * runtime because the speed/robustness trade-off (a marker only found at one
 * particular window size gets silently missed at fewer passes) can only be
 * judged against the real scene, and a rebuild costs a full package+upload
 * cycle. Channel-scoped, unlike cctv_app's one global g_aruco, because each
 * lens has its own ArucoProcessor and can want a different setting.
 *
 * A bare "ARUCO_SCAN" reports every lens (ReportDetect()'s convention);
 * "ARUCO_SCAN <ch>" alone reports just that one -- neither mutates anything,
 * matching DUTY/HOLD_MS's "query never has side effects" rule.
 */
bool SampleComponent::HandleArucoScan(const char* cmd) {
  const char* p = strstr(cmd, "ARUCO_SCAN");
  if (p == NULL) return false;

  int ch = -1, passes = 3, win = 13;
  const int n = sscanf(p + 10, "%d %d %d", &ch, &passes, &win);  // 10 = strlen("ARUCO_SCAN")

  if (n <= 0) {
    for (int c = 0; c < kMaxChannels; ++c) ReportArucoScan(c);
    return true;
  }
  if (!ValidCh(ch)) return true;
  if (n >= 2 && aruco_[ch])
    aruco_[ch]->setScanPasses(passes, win);  // n==2 keeps win at its 13 default

  ReportArucoScan(ch);
  return true;
}

void SampleComponent::ReportArucoScan(int ch) const {
  if (!ValidCh(ch)) return;
  char json[128];
  snprintf(json, sizeof(json), "{\"type\":\"ARUCO_SCAN\",\"ch\":%d,\"passes\":%d,\"win\":%d}",
           ch, aruco_[ch] ? aruco_[ch]->scanPasses() : 0, aruco_[ch] ? aruco_[ch]->scanWin() : 0);
  pose_sender_send_control_line(json);
}

/**
 * "DETECT_PARAM <ch> [<name> <value>]" -- tune one of the four detection-RATE
 * knobs (perim/ecc/thresh/poly, see ArucoProcessor::setDetectParam) on one
 * lens. Ported from cctv_app (2026-08-10), channel-scoped for the same reason
 * as ARUCO_SCAN. RAM-only like ARUCO_SCAN: a relaxed threshold that survived a
 * reboot would raise false positives with nothing on screen to explain it.
 *
 * "DETECT_PARAM" alone reports every lens; "DETECT_PARAM <ch>" alone reports
 * just that one; neither mutates anything. The reply always carries all four
 * values so the dashboard can show the full state after any single change.
 */
bool SampleComponent::HandleDetectParam(const char* cmd) {
  const char* p = strstr(cmd, "DETECT_PARAM");
  if (p == NULL) return false;
  p += 13;  // strlen("DETECT_PARAM")

  int ch = -1;
  char name[32] = {0};
  double value = 0.0;
  const int n = sscanf(p, "%d %31s %lf", &ch, name, &value);

  if (n <= 0) {
    for (int c = 0; c < kMaxChannels; ++c) ReportDetectParam(c, true, "", "");
    return true;
  }
  if (!ValidCh(ch)) return true;
  if (n < 3) {
    ReportDetectParam(ch, true, "", "");
    return true;
  }

  // Only ever hand ReportDetectParam one of these four literals, never the raw
  // wire buffer -- see its declaration for why.
  static const char* const kKnown[] = {"perim", "ecc", "thresh", "poly"};
  const char* safeName = "";
  for (int i = 0; i < 4 && safeName[0] == '\0'; ++i)
    if (strcmp(name, kKnown[i]) == 0) safeName = kKnown[i];

  if (!aruco_[ch]) {
    ReportDetectParam(ch, false, safeName, "detector not running");
    return true;
  }
  if (safeName[0] == '\0') {
    ReportDetectParam(ch, false, safeName, "unknown name (use perim|ecc|thresh|poly)");
    return true;
  }
  const bool ok = aruco_[ch]->setDetectParam(safeName, value);
  ReportDetectParam(ch, ok, safeName, "");
  return true;
}

void SampleComponent::ReportDetectParam(int ch, bool ok, const char* name,
                                        const char* reason) const {
  if (!ValidCh(ch)) return;
  char json[320];
  snprintf(json, sizeof(json),
           "{\"type\":\"DETECT_PARAM\",\"ch\":%d,\"ok\":%s,\"name\":\"%s\",\"reason\":\"%s\","
           "\"perim\":%.4f,\"ecc\":%.3f,\"thresh\":%.2f,\"poly\":%.4f}",
           ch, ok ? "true" : "false", name ? name : "", reason ? reason : "",
           aruco_[ch] ? aruco_[ch]->minPerimRate()     : 0.0,
           aruco_[ch] ? aruco_[ch]->errCorrRate()      : 0.0,
           aruco_[ch] ? aruco_[ch]->adaptiveThreshC()  : 0.0,
           aruco_[ch] ? aruco_[ch]->polyAccuracyRate() : 0.0);
  pose_sender_send_control_line(json);
}

#if ENABLE_SHELL_CMD
/**
 * "SHELL <cmd>" -- run <cmd> via /bin/sh, stream stdout+stderr back one JSON
 * line at a time. Ported from cctv_app (2026-08-10) — see ENABLE_SHELL_CMD's
 * warning in app_config.h before reading further.
 *
 * Matched with an anchored prefix, not strstr: unlike every other command
 * here, <cmd> is free text an operator typed, not a fixed numeric/enum
 * argument, so it could legitimately contain the substring "DETECT" or
 * "CALIB_K_STATUS" or any other command name. DispatchCommand() checks this
 * FIRST, ahead of every strstr-based handler, so a shell command's own text
 * never gets a chance to be misread as one of those.
 *
 * One JSON line per output line rather than one blob: JsonEscapeShellText()
 * flattens control characters to spaces, so a multi-line blob would arrive as
 * one run-on line. Per-line also lets the dashboard render it as a terminal
 * transcript as it arrives instead of waiting for the whole thing.
 */
bool SampleComponent::HandleShell(const char* cmd) {
  if (strncmp(cmd, "SHELL", 5) != 0) return false;
  const char* p = cmd + 5;
  while (*p == ' ') ++p;
  if (*p == '\0') return true;  // bare "SHELL": nothing to run, ignore

  char esc[520];
  char json[640];

  JsonEscapeShellText(p, esc, sizeof(esc));
  snprintf(json, sizeof(json), "{\"type\":\"SHELL\",\"stream\":\"start\",\"cmd\":\"%s\"}", esc);
  pose_sender_send_control_line(json);

  // 2>&1 so a failing command reports WHY (the error text is the whole point
  // of running it here) instead of silently producing nothing. Sized to the
  // wire's own line cap (POSE_SENDER_MAX_LINE), not cctv_app's shorter fixed
  // buffer, so a long <cmd> cannot get silently truncated into a different
  // command than the one that was actually typed.
  char line[POSE_SENDER_MAX_LINE + 16];
  snprintf(line, sizeof(line), "%s 2>&1", p);
  FILE* f = popen(line, "r");
  if (f == NULL) {
    pose_sender_send_control_line(
        "{\"type\":\"SHELL\",\"stream\":\"end\",\"exit\":-1,\"lines\":0,\"truncated\":false}");
    return true;
  }

  int lines = 0;
  bool truncated = false;
  char raw[256];
  while (fgets(raw, sizeof(raw), f) != NULL) {
    if (lines >= SHELL_MAX_LINES) {
      truncated = true;
      break;  // pclose() below drops the pipe; the child dies on SIGPIPE at
              // its next write, same as cctv_app's version of this cutoff.
    }
    size_t n = strlen(raw);
    while (n > 0 && (raw[n - 1] == '\n' || raw[n - 1] == '\r')) raw[--n] = '\0';
    JsonEscapeShellText(raw, esc, sizeof(esc));
    snprintf(json, sizeof(json), "{\"type\":\"SHELL\",\"stream\":\"out\",\"line\":\"%s\"}", esc);
    pose_sender_send_control_line(json);
    ++lines;
  }
  const int rc = pclose(f);

  // WEXITSTATUS is only meaningful once WIFEXITED says the child exited
  // normally -- which it did NOT when the loop broke out early above and the
  // child died on SIGPIPE instead. Report -1 ("killed / unknown") rather than
  // a garbage code in that case.
  int exit_code = -1;
  if (rc != -1 && WIFEXITED(rc)) exit_code = WEXITSTATUS(rc);

  snprintf(json, sizeof(json),
           "{\"type\":\"SHELL\",\"stream\":\"end\",\"exit\":%d,\"lines\":%d,\"truncated\":%s}",
           exit_code, lines, truncated ? "true" : "false");
  pose_sender_send_control_line(json);
  return true;
}
#endif  // ENABLE_SHELL_CMD

/**
 * "DETECT <ch> <0|1>" -- switch one lens's search off/on from the pose link.
 * A bare "DETECT" just reports the current state.
 *
 * The HTTP route to the same switch is unreachable precisely when the app is
 * overloaded, so this one exists as the way in that does not depend on the
 * scheduler having spare time: it is drained from the frame callback itself.
 */
bool SampleComponent::HandleDetect(const char* cmd) {
  const char* p = strstr(cmd, "DETECT");
  if (p == NULL) return false;

  int ch = -1, on = 1;
  if (sscanf(p + 6, "%d %d", &ch, &on) >= 1)  // 6 = strlen("DETECT")
    SetDetectEnabled(ch, on != 0);

  ReportDetect();
  return true;
}

/**
 * Drain commands the dashboard sent on the pose link. Ordering matters:
 * DYNROI_IDS must be tried first (see above).
 */
bool SampleComponent::DispatchCommand(const char* cmd) {
  if (cmd == NULL) return false;
#if ENABLE_SHELL_CMD
  // Checked before anything else, and unconditionally (not folded into the
  // "matches on substrings" note below): <cmd>'s ARGUMENT is free text an
  // operator typed, not a fixed numeric/enum value like every other command
  // here, so it could legitimately contain "DETECT" or "CALIB_K_STATUS" or
  // any other command's name as a substring. Every handler below matches
  // loosely (strstr/strncmp-prefix), so if one of them ran first on a SHELL
  // command whose own text happened to contain its keyword, it would swallow
  // the shell command and this handler would never see it.
  if (HandleShell(cmd)) return true;
#endif
  // Ordering matters below this point too: these match on substrings, so a
  // longer name that CONTAINS a shorter one has to be tried first. DYNROI_IDS
  // before DYNROI is the existing case — otherwise "DYNROI" swallows it and
  // parses "_IDS ..." as its own numeric arguments.
#if ENABLE_CENTRAL_TLS_STREAM
  if (HandleCentral(cmd)) return true;  // CENTRAL_* — central-server link control
#endif
  if (HandleCalibK(cmd)) return true;   // CALIB_K_* — all matched in one place
  if (HandleHomography(cmd)) return true;  // HG_* — likewise
  if (HandleAnchors(cmd)) return true;     // ANCHOR_*
  if (HandleScale(cmd)) return true;
  if (HandleArucoScan(cmd)) return true;
  // Before HandleDetect: "DETECT_PARAM" contains "DETECT", and HandleDetect()
  // matches that with strstr -- tried in the other order it would swallow
  // every DETECT_PARAM command and silently just re-report DETECT state.
  if (HandleDetectParam(cmd)) return true;
  if (HandleDuty(cmd)) return true;
  if (HandleHoldMs(cmd)) return true;
  if (HandleZoneRadius(cmd)) return true;  // ZONE_RADIUS — 발끝 판정 반경
  if (HandleDetStream(cmd)) return true;   // DET_STREAM — 검출 위치 스트림
  if (HandleZoneBands(cmd)) return true;   // ZONE_BANDS — 완충 밴드 on/off
  if (HandleZoneMargin(cmd)) return true;  // ZONE_MARGIN — 밴드 거리(mm)
  if (HandleZoneAlarmLevel(cmd)) return true;  // ZONE_ALARM_LEVEL — 경보 임계 단계
  if (HandleDetect(cmd)) return true;
  if (HandleDynRoiIds(cmd)) return true;
  if (HandleDynRoiCh(cmd)) return true;
  if (HandleDynRoi(cmd)) return true;  // must stay last of the DYNROI* three
  return false;
}

/**
 * CALIB_K_* — intrinsics calibration, in the RPi dashboard's own vocabulary.
 *
 * Matched with exact prefixes rather than the substring style used above,
 * because the names here overlap heavily (CALIB_K_BOARD_SAVE vs CALIB_K_SAVE,
 * CALIB_K_START vs CALIB_K_STATUS) and substring matching would pick whichever
 * happened to be tested first.
 *
 * Commands:
 *   CALIB_K_CONFIG sx sy square marker dict marginx marginy   (shared board)
 *   CALIB_K_BOARD_SAVE
 *   CALIB_K_SET <targetViews> <rmsLimit>
 *   CALIB_K_GATE <0|1>
 *   CALIB_K_START <ch>          — opens the one session, on that lens
 *   CALIB_K_STOP                — abandon without computing
 *   CALIB_K_CAPTURE [<ch>]      — capture the next frame: that lens, or (bare)
 *                                 every lens with an open session
 *   CALIB_K_UNDO
 *   CALIB_K_COMPUTE
 *   CALIB_K_SAVE <ch>
 *   CALIB_K_REVERT <ch>         — swap back to the value before the last save
 *   K_LOAD <ch> fx fy cx cy d0 d1 d2 d3 d4
 *
 * The on-camera page still reads all of this from /status — that answer path
 * is unchanged. Every state-changing branch here ALSO pushes a CALIB_K_*
 * status line on the pose link now (2026-08-10; see the ReportCalibK* block
 * above HandleHomography()) — a dashboard driving this over TCP has no
 * /status of its own to poll, and until this existed it had no way to learn
 * a session started, a capture was accepted or rejected, a compute finished,
 * or a save landed.
 */
bool SampleComponent::HandleCalibK(const char* cmd) {
  const char* reason = NULL;

  if (strncmp(cmd, "CALIB_K_CONFIG", 14) == 0) {
    CharucoBoardConfig c = calib_.Board();
    const int n = sscanf(cmd + 14, "%d %d %f %f %d %f %f", &c.squares_x, &c.squares_y,
                         &c.square_length_mm, &c.marker_length_mm, &c.dictionary_id,
                         &c.outer_margin_x_mm, &c.outer_margin_y_mm);
    if (n < 5) return true;  // recognised but ignored — a partial board is not a board
    // The reason is carried into /status rather than dropped: a rejected board
    // otherwise looks identical to an applied one — the form simply snaps back
    // to the old numbers on the next refresh with nothing said.
    const bool ok = calib_.SetBoard(c, &reason);
    calib_.NoteMessage(ok ? "보드 설정을 적용했습니다 (RAM)" : reason);
    // Board-wide, not per lens — but the reply's shape carries per-lens view
    // counters too (cctv.py's combined CONFIG/STATUS/UNDO handler), so every
    // channel gets one: the RPi dashboard has no other way to learn the new
    // board applies to all four.
    for (int c = 0; c < kMaxChannels; ++c)
      ReportCalibKConfig(c, "CALIB_K_CONFIG", ok, ok ? "" : reason);
    return true;
  }
  if (strncmp(cmd, "CALIB_K_BOARD_SAVE", 18) == 0) {
    const bool ok = calib_.SaveBoard();
    calib_.NoteMessage(ok ? "보드 설정 저장됨" : "보드 설정 저장 실패");
    ReportCalibKBoardSave(ok);
    return true;
  }
  if (strncmp(cmd, "CALIB_K_SET", 11) == 0) {
    int views = 0;
    double rms = 0.0;
    sscanf(cmd + 11, "%d %lf", &views, &rms);
    calib_.SetParams(views, rms);
    for (int c = 0; c < kMaxChannels; ++c)
      ReportCalibKConfig(c, "CALIB_K_STATUS", true, "");
    return true;
  }
  if (strncmp(cmd, "CALIB_K_GATE", 12) == 0) {
    int on = 1;
    if (sscanf(cmd + 12, "%d", &on) == 1) calib_.SetGates(on != 0);
    ReportCalibKGate();
    return true;
  }
  if (strncmp(cmd, "CALIB_K_START", 13) == 0) {
    int ch = -1;
    if (sscanf(cmd + 13, "%d", &ch) != 1) return true;
    // The mirror of the refusal in StartFit(), and it has to live here rather
    // than in IntrinsicsCalib: that object does not know the mapper exists, and
    // pointing it at one to answer a single question would tie the two together
    // in the direction the port deliberately kept clear. This handler already
    // holds both.
    //
    // Opening a board session over a running collection does not slow it down,
    // it stops it dead and leaves it that way: the collection's counters and
    // its 200-frame ceiling are both driven by frames it stops receiving, so it
    // neither progresses nor times out. See StartFit() for the same reasoning
    // from the other side.
    if (homography_.FitCollecting(ch)) {
      const char* why = "이 렌즈는 호모그래피 수집 중입니다 "
                        "— 호모그래피 탭에서 끝난 뒤에 시작하세요";
      calib_.NoteMessage(why);
      if (ValidCh(ch)) ReportCalibKConfig(ch, "CALIB_K_STATUS", false, why);
      return true;
    }
    if (!calib_.Start(ch, &reason)) {
      calib_.NoteMessage(reason);
      if (ValidCh(ch)) ReportCalibKConfig(ch, "CALIB_K_STATUS", false, reason);
    } else if (ValidCh(ch)) {
      ReportCalibKAck(ch);
    }
    return true;
  }
  if (strncmp(cmd, "CALIB_K_STOP", 12) == 0) {
    // Named lens, because there can be several open. No argument stops them
    // all — an operator abandoning a calibration run usually means all of it,
    // and making them type four commands to do it invites leaving one open.
    int ch = -1;
    if (sscanf(cmd + 12, "%d", &ch) == 1) {
      calib_.Stop(ch);
    } else {
      for (int c = 0; c < kMaxChannels; ++c) calib_.Stop(c);
    }
    return true;
  }
  if (strncmp(cmd, "CALIB_K_CAPTURE", 15) == 0) {
    // Only flags the request. The frame is taken by ProcessRawVideo when the
    // active lens next delivers one — this handler runs on an HTTP or TCP
    // event and has no image in hand, and capturing a stale copy would grade
    // a pose the operator has already moved out of.
    // "CALIB_K_CAPTURE <ch>" arms ONLY that lens; bare, it arms every open
    // session so a single board pose is banked by every lens that can see it.
    // Each lens judges it separately when its own frame arrives — see
    // IntrinsicsCalib::RequestCapture. The per-lens accept/reject report
    // (CALIB_K_PROGRESS) fires later, from ProcessRawVideo, once
    // TakePendingCapture() actually has a verdict.
    //
    // The <ch> form is the one the dashboard sends (2026-08-11). The bare form
    // stays because it is what fills several overlapping lenses from one board
    // pose, and because older callers send it.
    int ch = -1;
    if (sscanf(cmd + 15, "%d", &ch) != 1) ch = -1;
    if (!calib_.RequestCapture(ch, &reason)) calib_.NoteMessage(reason);
    return true;
  }
  if (strncmp(cmd, "CALIB_K_UNDO", 12) == 0) {
    int ch = -1;
    if (sscanf(cmd + 12, "%d", &ch) != 1) return true;
    const bool ok = calib_.UndoLast(ch);
    if (ValidCh(ch))
      ReportCalibKConfig(ch, "CALIB_K_UNDO", ok, ok ? "" : "되돌릴 뷰가 없습니다");
    return true;
  }
  if (strncmp(cmd, "CALIB_K_COMPUTE", 15) == 0) {
    // Seconds of blocking work on the scheduler thread, and that is the right
    // call: it is explicit, one-shot, and the alternative (a worker thread)
    // would need every calibration field locked for the rest of the app's life.
    int ch = -1;
    if (sscanf(cmd + 15, "%d", &ch) != 1) {
      calib_.NoteMessage("사용법: CALIB_K_COMPUTE <ch>");
      return true;
    }
    if (ValidCh(ch)) ReportCalibKComputing(ch);
    const CalibState result = calib_.Compute(ch);
    if (ValidCh(ch)) ReportCalibKResult(ch, result == CS_DONE_OK);
    return true;
  }
  if (strncmp(cmd, "CALIB_K_CLEAR", 13) == 0) {
    int ch = -1;
    if (sscanf(cmd + 13, "%d", &ch) != 1) return true;
    calib_.NoteMessage(calib_.Clear(ch) ? "이 렌즈의 K/dist를 지웠습니다"
                                        : "채널 번호 범위 초과");
    return true;
  }
  // CALIB_K_REVERT <ch> — put back the value this lens had before the last
  // save, and make the current one the previous. Reversible by running it
  // again; see IntrinsicsCalib::Revert().
  if (strncmp(cmd, "CALIB_K_REVERT", 14) == 0) {
    int ch = -1;
    if (sscanf(cmd + 14, "%d", &ch) != 1) return true;
    // Revert() sets its own reason on every failure path, so only success needs
    // saying here — otherwise the button that worked and the button that found
    // nothing to revert to look identical.
    if (calib_.Revert(ch)) calib_.NoteMessage("이전 K/dist로 되돌렸습니다 (다시 누르면 원래대로)");
    return true;
  }
  if (strncmp(cmd, "CALIB_K_SAVE", 12) == 0) {
    int ch = -1;
    if (sscanf(cmd + 12, "%d", &ch) != 1) return true;
    // Save() sets its own reason when the write fails; a success has to say so
    // too, or the button looks identical either way.
    const bool ok = calib_.Save(ch);
    if (ok) calib_.NoteMessage("이 렌즈의 K/dist를 저장했습니다");
    if (ValidCh(ch)) ReportCalibKSave(ch, ok);
    return true;
  }
  // CALIB_K_STATUS [ch] / CALIB_K_QUERY [ch] — on-demand re-sync for a
  // dashboard that just (re)connected or switched channels, so it does not
  // have to wait for the next state-changing command to see where things
  // stand. No <ch> reports every channel — the RPi dashboard asks this way at
  // page load (before it has picked a channel), and cctv_app-era code sent it
  // bare too.
  if (strncmp(cmd, "CALIB_K_STATUS", 14) == 0) {
    int ch = -1;
    if (sscanf(cmd + 14, "%d", &ch) == 1 && ValidCh(ch)) {
      ReportCalibKConfig(ch, "CALIB_K_STATUS", true, "");
    } else {
      for (int c = 0; c < kMaxChannels; ++c) ReportCalibKConfig(c, "CALIB_K_STATUS", true, "");
    }
    return true;
  }
  if (strncmp(cmd, "CALIB_K_QUERY", 13) == 0) {
    int ch = -1;
    if (sscanf(cmd + 13, "%d", &ch) == 1 && ValidCh(ch)) {
      ReportCalibKQuery(ch);
    } else {
      for (int c = 0; c < kMaxChannels; ++c) ReportCalibKQuery(c);
    }
    return true;
  }
  if (strncmp(cmd, "K_LOAD", 6) == 0) {
    int ch = -1;
    double fx = 0, fy = 0, cx = 0, cy = 0, d[5] = {0, 0, 0, 0, 0};
    const int n = sscanf(cmd + 6, "%d %lf %lf %lf %lf %lf %lf %lf %lf %lf", &ch, &fx, &fy,
                         &cx, &cy, &d[0], &d[1], &d[2], &d[3], &d[4]);
    if (n < 5) return true;
    // Says so either way. Without the success line the previous rejection
    // stayed on screen after a load that had actually worked.
    calib_.NoteMessage(calib_.LoadValues(ch, fx, fy, cx, cy, d)
        ? "K/dist를 적용했습니다 — 저장해야 재부팅 후에도 남습니다"
        : "K/dist 값이 올바르지 않습니다 (fx·fy>0, cx·cy>=0, 유한한 수)");
    return true;
  }
  return false;
}

// --- K/dist reporting (see the doc comments in sample_component.h) --------

void SampleComponent::ReportCalibKConfig(int ch, const char* type, bool ok,
                                         const char* reason) const {
  const CharucoBoardConfig b = calib_.Board();
  const double board_w = b.squares_x * b.square_length_mm + 2.0 * b.outer_margin_x_mm;
  const double board_h = b.squares_y * b.square_length_mm + 2.0 * b.outer_margin_y_mm;
  char json[512];
  snprintf(json, sizeof(json),
           "{\"type\":\"%s\",\"ch\":%d,\"ok\":%s,\"reason\":\"%s\","
           "\"views\":%d,\"target\":%d,"
           "\"squares_x\":%d,\"squares_y\":%d,\"square_mm\":%.2f,\"marker_mm\":%.2f,"
           "\"dictionary\":%d,\"margin_x_mm\":%.2f,\"margin_y_mm\":%.2f,"
           "\"board_w_mm\":%.1f,\"board_h_mm\":%.1f,\"gates\":%s}",
           type, ch, ok ? "true" : "false", reason ? reason : "",
           calib_.Views(ch), calib_.TargetViews(),
           b.squares_x, b.squares_y, b.square_length_mm, b.marker_length_mm,
           b.dictionary_id, b.outer_margin_x_mm, b.outer_margin_y_mm,
           board_w, board_h, calib_.Gates() ? "true" : "false");
  pose_sender_send_control_line(json);
}

void SampleComponent::ReportCalibKAck(int ch) const {
  const CharucoBoardConfig b = calib_.Board();
  char json[320];
  snprintf(json, sizeof(json),
           "{\"type\":\"CALIB_K_ACK\",\"ch\":%d,\"target\":%d,"
           "\"squares_x\":%d,\"squares_y\":%d,\"square_mm\":%.2f,\"marker_mm\":%.2f,"
           "\"dictionary\":%d}",
           ch, calib_.TargetViews(), b.squares_x, b.squares_y,
           b.square_length_mm, b.marker_length_mm, b.dictionary_id);
  pose_sender_send_control_line(json);
}

void SampleComponent::ReportCalibKProgress(int ch) const {
  const CalibViewQuality& q = calib_.LastQuality(ch);
  const bool rejected = calib_.LastCapture(ch) == CS_CAPTURE_REJECTED;
  const bool ready = calib_.Views(ch) >= calib_.TargetViews();
  char json[448];
  snprintf(json, sizeof(json),
           "{\"type\":\"CALIB_K_PROGRESS\",\"ch\":%d,\"rejected\":%s,\"reason\":\"%s\","
           "\"corners\":%d,\"corners_total\":%d,\"coverage\":%.4f,\"sharpness\":%.2f,"
           "\"move_px\":%.2f,\"views\":%d,\"target\":%d,\"ready\":%s}",
           ch, rejected ? "true" : "false", q.reason ? q.reason : "",
           q.corners_found, q.corners_total, q.coverage_ratio, q.sharpness,
           q.mean_move_px, calib_.Views(ch), calib_.TargetViews(),
           (!rejected && ready) ? "true" : "false");
  pose_sender_send_control_line(json);
}

/**
 * The held ChArUco corner viewfinder, for the calibration tab to draw the
 * same way the marker-detection tab draws ArUco corners (2026-08-11).
 *
 * Capped at 40 points -- a 7x5 board (this app's configured default) has 24
 * interior corners, so 40 leaves headroom for a bigger board while keeping
 * the line comfortably under POSE_SENDER_MAX_LINE (1024 B): 40 points at
 * ~18 B each plus the envelope is well under half that.
 */
void SampleComponent::ReportCalibKProbe(int ch) const {
  if (!ValidCh(ch)) return;
  const std::vector<cv::Point2f>& pc = calib_.ProbeCorners(ch);
  const std::vector<int>& pid = calib_.ProbeIds(ch);
  // LastQuality(ch).corners_total is CaptureView()'s number and stays 0 until
  // this channel's first CAPTURE -- wrong source for a viewfinder that is
  // meant to help AIM before that first capture. Board() x/y matches what
  // ProbeIfDue() itself used to fill Session::probe_total (private, no
  // accessor), so recompute the same way rather than adding one just for
  // this.
  const CharucoBoardConfig b = calib_.Board();
  const int corners_total = (b.squares_x - 1) * (b.squares_y - 1);
  char json[800];
  int n = snprintf(json, sizeof(json),
           "{\"type\":\"CALIB_K_PROBE\",\"ch\":%d,\"corners_total\":%d,\"probe\":[",
           ch, corners_total);
  const size_t np = pc.size() < 40 ? pc.size() : 40;
  for (size_t i = 0; i < np && n > 0 && n < (int)sizeof(json) - 32; ++i)
    n += snprintf(json + n, sizeof(json) - n, "%s[%.1f,%.1f,%d]", i ? "," : "",
                  pc[i].x, pc[i].y, (i < pid.size()) ? pid[i] : -1);
  if (n > 0 && n < (int)sizeof(json) - 2) snprintf(json + n, sizeof(json) - n, "]}");
  pose_sender_send_control_line(json);
}

void SampleComponent::ReportCalibKComputing(int ch) const {
  char json[96];
  snprintf(json, sizeof(json), "{\"type\":\"CALIB_K_COMPUTING\",\"ch\":%d,\"views\":%d}",
           ch, calib_.Views(ch));
  pose_sender_send_control_line(json);
}

void SampleComponent::ReportCalibKResult(int ch, bool ok) const {
  char json[512];
  if (ok) {
    double fx = 0, fy = 0, cx = 0, cy = 0, dist[5] = {0, 0, 0, 0, 0};
    calib_.Get(ch, &fx, &fy, &cx, &cy, dist);
    snprintf(json, sizeof(json),
             "{\"type\":\"CALIB_K_RESULT\",\"ch\":%d,\"ok\":true,\"rms\":%.4f,"
             "\"views\":%d,\"pruned\":%d,\"fx\":%.4f,\"fy\":%.4f,\"cx\":%.4f,\"cy\":%.4f,"
             "\"dist\":[%.10e,%.10e,%.10e,%.10e,%.10e]}",
             ch, calib_.Rms(ch), calib_.Views(ch), calib_.PrunedViews(ch),
             fx, fy, cx, cy, dist[0], dist[1], dist[2], dist[3], dist[4]);
  } else {
    snprintf(json, sizeof(json),
             "{\"type\":\"CALIB_K_RESULT\",\"ch\":%d,\"ok\":false,\"reason\":\"%s\"}",
             ch, calib_.FailReason() ? calib_.FailReason() : "");
  }
  pose_sender_send_control_line(json);
}

void SampleComponent::ReportCalibKSave(int ch, bool ok) const {
  char json[192];
  snprintf(json, sizeof(json), "{\"type\":\"CALIB_K_SAVE\",\"ch\":%d,\"ok\":%s,\"reason\":\"%s\"}",
           ch, ok ? "true" : "false", ok ? "" : calib_.FailReason());
  pose_sender_send_control_line(json);
}

void SampleComponent::ReportCalibKBoardSave(bool ok) const {
  char json[192];
  snprintf(json, sizeof(json), "{\"type\":\"CALIB_K_BOARD_SAVE\",\"ok\":%s,\"reason\":\"%s\"}",
           ok ? "true" : "false", ok ? "" : calib_.FailReason());
  pose_sender_send_control_line(json);
}

void SampleComponent::ReportCalibKGate() const {
  char json[96];
  snprintf(json, sizeof(json), "{\"type\":\"CALIB_K_GATE\",\"enabled\":%s}",
           calib_.Gates() ? "true" : "false");
  pose_sender_send_control_line(json);
}

void SampleComponent::ReportCalibKQuery(int ch) const {
  char json[384];
  if (calib_.Available(ch)) {
    double fx = 0, fy = 0, cx = 0, cy = 0, dist[5] = {0, 0, 0, 0, 0};
    calib_.Get(ch, &fx, &fy, &cx, &cy, dist);
    snprintf(json, sizeof(json),
             "{\"type\":\"CALIB_K_QUERY\",\"ch\":%d,\"available\":true,"
             "\"fx\":%.4f,\"fy\":%.4f,\"cx\":%.4f,\"cy\":%.4f,"
             "\"dist\":[%.10e,%.10e,%.10e,%.10e,%.10e]}",
             ch, fx, fy, cx, cy, dist[0], dist[1], dist[2], dist[3], dist[4]);
  } else {
    snprintf(json, sizeof(json),
             "{\"type\":\"CALIB_K_QUERY\",\"ch\":%d,\"available\":false}", ch);
  }
  pose_sender_send_control_line(json);
}

/**
 * HG_* — homography, in cctv_app's and the RPi dashboard's vocabulary.
 *
 * Commands (this slice):
 *   HG_SET   <ch> <h00> <h01> ... <h22> [<space>]   inject a 3x3, RAM only
 *   HG_SAVE  <ch>                                   persist it
 *   HG_CLEAR <ch>                                   forget it, RAM and disk
 *   HG_QUERY <ch>                                   ack on the pose link
 *
 * Still to come (see docs/HOMOGRAPHY_HANDOFF_PROMPT.md): ANCHOR_SET_ALL,
 * CALIB_START, HG_COORD_MODE, MARKER_HEIGHT/CAMERA_HEIGHT, MARKER_PLANE_*.
 *
 * Exact prefixes, like HandleCalibK() and for the same reason: these names
 * overlap heavily (HG_SET vs HG_SAVE, and MARKER_PLANE_SAVE vs MARKER_HEIGHT
 * once those land), so substring matching would pick whichever was tested
 * first. Longest-first ordering is NOT relied on here — every strncmp uses the
 * full command name, so no name can swallow another.
 *
 * `<space>` on HG_SET is optional and defaults to 1 (undistorted). It records
 * THE SPACE THE MATRIX WAS FITTED IN, which is a property of the sender's
 * data, not of this camera — see HomographyMapper::Set().
 */
bool SampleComponent::HandleHomography(const char* cmd) {
  if (strncmp(cmd, "HG_SET", 6) == 0) {
    int ch = -1, space = 1;
    double h[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    const int n = sscanf(cmd + 6, "%d %lf %lf %lf %lf %lf %lf %lf %lf %lf %d", &ch,
                         &h[0], &h[1], &h[2], &h[3], &h[4], &h[5], &h[6], &h[7], &h[8],
                         &space);
    if (n < 10) {
      printf("[ArucoPosePNM] HG_SET: 인자 부족 (ch + 9개 필요, %d개 받음)\n", n);
      fflush(stdout);
      return true;
    }
    const bool ok = homography_.Set(ch, h, space != 0);
    printf("[ArucoPosePNM] HG_SET ch%d %s%s%s\n", ch, ok ? "적용" : "거부",
           ok ? " — 저장해야 재부팅 후에도 남습니다" : " — ", ok ? "" : homography_.FailReason());
    fflush(stdout);
    if (ok) ReportHomography(ch);
    return true;
  }

  if (strncmp(cmd, "HG_SAVE", 7) == 0) {
    int ch = -1;
    if (sscanf(cmd + 7, "%d", &ch) != 1) return true;
    const bool ok = homography_.Save(ch);
    printf("[ArucoPosePNM] HG_SAVE ch%d %s %s\n", ch, ok ? "성공" : "실패",
           ok ? "" : homography_.FailReason());
    fflush(stdout);
    return true;
  }

  if (strncmp(cmd, "HG_CLEAR", 8) == 0) {
    int ch = -1;
    if (sscanf(cmd + 8, "%d", &ch) != 1) return true;
    const bool ok = homography_.Clear(ch);
    printf("[ArucoPosePNM] HG_CLEAR ch%d %s\n", ch, ok ? "완료" : homography_.FailReason());
    fflush(stdout);
    return true;
  }

  if (strncmp(cmd, "HG_QUERY", 8) == 0) {
    int ch = -1;
    if (sscanf(cmd + 8, "%d", &ch) != 1) return true;
    ReportHomography(ch);
    // The fit summary + per-point residuals alongside it, not just on
    // completion — an operator revisiting a channel (or a dashboard that
    // just reconnected) has no other way to ask "how good was that fit"
    // again without re-running the whole collection.
    ReportHomographyFit(ch);
    ReportHomographyFitPoints(ch);
    return true;
  }

  if (strncmp(cmd, "HG_COORD_MODE", 13) == 0) {
    int ch = -1, on = 1;
    if (sscanf(cmd + 13, "%d %d", &ch, &on) != 2) return true;
    const bool ok = homography_.SetCoordMode(ch, on != 0);
    printf("[ArucoPosePNM] HG_COORD_MODE ch%d -> %s : %s\n", ch,
           (on != 0) ? "undistort" : "raw",
           ok ? "적용" : homography_.FailReason());
    fflush(stdout);
    if (ok) ReportHomography(ch);
    return true;
  }

  // CALIB_START <ch> — open the collection session that fits this lens's H
  // from its registered markers.
  //
  // Named without the HG_ prefix because cctv_app and the RPi dashboard already
  // call it this; the only change is the <ch>. It cannot be confused with
  // CALIB_K_START — every prefix in HandleCalibK() begins "CALIB_K_", and that
  // handler runs first.
  //
  // Nothing is saved on success. The matrix and its residuals live in RAM and
  // HG_SAVE persists them, for the same reason K/dist works that way: a
  // calibration that saves itself is a calibration whose save failure nobody
  // notices until the next reboot.
  if (strncmp(cmd, "CALIB_START", 11) == 0) {
    int ch = -1;
    if (sscanf(cmd + 11, "%d", &ch) != 1) return true;
    const bool ok = homography_.StartFit(ch);
    printf("[ArucoPosePNM] CALIB_START ch%d %s%s%s\n", ch, ok ? "수집 시작" : "거부",
           ok ? "" : " — ", ok ? "" : homography_.FailReason());
    fflush(stdout);
    // Immediate ack so a TCP-only dashboard sees "collecting" right away
    // rather than only finding out once the fit finishes (ReportHomography /
    // ReportHomographyFit, called from FeedFrame on completion — see there).
    if (ok) ReportHgAck(ch);
    return true;
  }

  // MARKER_HEIGHT <mm> — how far the robot's marker sits above the floor.
  //
  // SHARED, no <ch>: one robot carries one marker at one height. Four copies
  // would be four things to keep equal, and the lens that disagreed would
  // report the robot in a slightly different place than its neighbour — a
  // discrepancy that looks like a calibration error rather than a typo.
  //
  // Applying is not saving, as everywhere else here. A mistyped height is then
  // undone by a restart rather than by remembering what it used to be.
  if (strncmp(cmd, "MARKER_HEIGHT", 13) == 0) {
    double mm = -1.0;
    if (sscanf(cmd + 13, "%lf", &mm) != 1) {
      printf("[ArucoPosePNM] MARKER_HEIGHT: 사용법 MARKER_HEIGHT <mm>\n");
      fflush(stdout);
      return true;
    }
    const bool ok = homography_.SetMarkerHeightMm(mm);
    printf("[ArucoPosePNM] MARKER_HEIGHT %.1f mm %s%s\n", mm, ok ? "적용" : "거부 — ",
           ok ? "" : homography_.FailReason());
    fflush(stdout);
    return true;
  }

  // CAMERA_HEIGHT <ch> <mm> — the tape-measured height of THIS lens. 0 clears
  // it and returns to the height the H decomposition implies.
  //
  // Per lens, unlike the marker height: the four sensors share a housing but
  // not a tilt, so their effective heights above the floor differ.
  //
  // What this buys: the correction is governed by the ratio h/Cz, and the Cz
  // that comes out of the decomposition carries whatever scale error residual
  // distortion left in t. A measured Cz removes that term. What it must NOT do
  // is move H_floor — see RefreshMarkerPlane(), which scales the marker height
  // instead for exactly that reason.
  if (strncmp(cmd, "CAMERA_HEIGHT", 13) == 0) {
    int ch = -1;
    double mm = -1.0;
    if (sscanf(cmd + 13, "%d %lf", &ch, &mm) != 2) {
      printf("[ArucoPosePNM] CAMERA_HEIGHT: 사용법 CAMERA_HEIGHT <ch> <mm>  (0 = 미측정)\n");
      fflush(stdout);
      return true;
    }
    const bool ok = homography_.SetCameraHeightMm(ch, mm);
    printf("[ArucoPosePNM] CAMERA_HEIGHT ch%d %.1f mm %s%s\n", ch, mm,
           ok ? "적용" : "거부 — ", ok ? "" : homography_.FailReason());
    fflush(stdout);
    return true;
  }

  // MARKER_PLANE_SAVE <ch> — persist the shared marker height and this lens's
  // camera height. Two files; see HomographyMapper::SaveMarkerPlane().
  if (strncmp(cmd, "MARKER_PLANE_SAVE", 17) == 0) {
    int ch = -1;
    if (sscanf(cmd + 17, "%d", &ch) != 1) return true;
    const bool ok = homography_.SaveMarkerPlane(ch);
    printf("[ArucoPosePNM] MARKER_PLANE_SAVE ch%d %s %s\n", ch, ok ? "성공" : "실패",
           ok ? "" : homography_.FailReason());
    fflush(stdout);
    return true;
  }

  // ODOM_PREFER <ch> <0|1> — 로봇 측위에 주행 캘리의 **측정** H_marker 를 쓸지,
  // 체커보드에서 시차 보정으로 얻은 **파생** H_marker 를 쓸지 고른다.
  //
  // 이 명령이 없는 동안 PixelToWorldMarker() 의 use_measured 분기
  // (homography_mapper.cc:559)는 영영 false 였다 — 주행 캘리를 성공해도 로봇
  // 좌표는 계속 파생값으로 나갔고, 측정값은 /status 에 보이기만 했다.
  //
  // 기본이 파생인 것은 그대로 둔다. 주행 방식은 균일 스케일 오차를 스스로
  // 검증하지 못하므로(폐합오차도 그건 못 잡는다 — 네 변이 같은 비율로 짧으면
  // 궤적도 닫힌다) 검증된 쪽을 기본으로 두고 전환을 의도적인 행동으로 만든다.
  // 전환하기 전에 CompareMarkerPlanes() 의 scale 을 보라는 뜻이기도 하다.
  // 공백까지 포함해 비교한다. "ODOM_PREFER" 11자로만 보면 아래의
  // ODOM_PREFER_QUERY 도 여기 걸려서 "인자 이상"으로 끝나고 조회가 영영 안 된다.
  if (strncmp(cmd, "ODOM_PREFER ", 12) == 0) {
    int ch = -1, on = -1;
    if (sscanf(cmd + 12, "%d %d", &ch, &on) != 2 || !ValidCh(ch) || (on != 0 && on != 1)) {
      printf("[ArucoPosePNM] ODOM_PREFER 인자 이상 — 사용법: ODOM_PREFER <ch> <0|1>\n");
      fflush(stdout);
      return true;
    }
    const bool ok = homography_.SetPreferMeasured(ch, on != 0);
    ReportOdomPrefer(ch, ok);
    printf("[ArucoPosePNM] ODOM_PREFER ch%d %s %s%s\n", ch, on ? "측정값" : "파생값",
           ok ? "적용" : "거부 — ", ok ? "" : homography_.FailReason());
    fflush(stdout);
    return true;
  }

  // ODOM_SETTLE <spread_px> [frames] — 정지 판정 임계값. 채널 인자가 없는
  // 몇 안 되는 명령이다(전역, g_odom_settle_* 선언부 주석 참고).
  //
  // 현장에서 조정할 수 있어야 하는 값이다: 너무 빡세면 로봇이 실제로 멈춰
  // 있는데도 not_settled 로 지점이 계속 버려지고, 너무 헐거우면 감속 중
  // 프레임이 섞여 그 오차가 그대로 캘리에 박힌다. 어느 쪽인지는 회신에 실려
  // 오는 spread_px 실측값을 보고 판단한다 — 그래서 실패 응답에도 그 값을 싣는다.
  if (strncmp(cmd, "ODOM_SETTLE ", 12) == 0) {
    double px = 0.0;
    int frames = 0;
    const int got = sscanf(cmd + 12, "%lf %d", &px, &frames);
    // 상한 50px: 그보다 크면 사실상 판정을 끄는 것이고, 그럴 거면 이 방식을
    // 쓸 이유가 없다. 하한 0.1px: 코너 검출 자체의 양자화 잡음보다 작다.
    if (got < 1 || !(px >= 0.1 && px <= 50.0)) {
      printf("[ArucoPosePNM] ODOM_SETTLE 인자 이상 — 사용법: ODOM_SETTLE <0.1..50 px> [2..%d 프레임]\n",
             ODOM_SETTLE_FRAMES);
      fflush(stdout);
      ReportOdomSettle();
      return true;
    }
    g_odom_settle_spread_px = (float)px;
    if (got >= 2) {
      if (frames < 2) frames = 2;
      if (frames > ODOM_SETTLE_FRAMES) frames = ODOM_SETTLE_FRAMES;  // 버퍼가 고정
      g_odom_settle_frames = frames;
    }
    printf("[ArucoPosePNM] ODOM_SETTLE %.2fpx %d프레임\n",
           g_odom_settle_spread_px, g_odom_settle_frames);
    fflush(stdout);
    ReportOdomSettle();
    return true;
  }

  if (strncmp(cmd, "ODOM_SETTLE_QUERY", 17) == 0) {
    ReportOdomSettle();
    return true;
  }

  // ODOM_PREFER_QUERY <ch> — 지금 어느 쪽을 쓰고 있는지. 측위 결과가 조용히
  // 달라지는 설정이라 조회 수단이 따로 있어야 한다.
  if (strncmp(cmd, "ODOM_PREFER_QUERY", 17) == 0) {
    int ch = -1;
    if (sscanf(cmd + 17, "%d", &ch) != 1 || !ValidCh(ch)) return true;
    ReportOdomPrefer(ch, true);
    return true;
  }

  // ODOM_RESEND <ch> — 주행 캘리로 만든 번들을 지금 다시 서버로 보낸다.
  //
  // 주행이 성공하면 CentralCalibDone() 이 자동으로 한 번 보낸다. 이건 그때
  // 링크가 끊겨 있었거나 서버가 재시작해 저장분을 잃은 경우를 위한 재전송이다.
  //
  // 대시보드가 JSON 을 조립해 CENTRAL_HMATRIX 로 보내던 예전 방식을 이걸로
  // 대체한다 (2026-08-13). 그 경로는 대시보드 캐시(hgHfloor/mpPlane)에서 값을
  // 끌어오는데, 그 캐시는 **정적 앵커(floor) 탭 흐름에서만 채워진다** — 주행
  // 캘리 결과는 카메라가 서버로 직접 보내고 대시보드는 저장하지 않는다. 그래서
  // Odometry 탭에서 그 버튼을 눌러도 실제로는 체커보드 값이 나갔다:
  // H_marker 가 "측정값"이 아니라 "floor H 에서 파생된 값"이었고, 역산 방향도
  // 자동 경로와 정반대였다. 같은 탭의 버튼이 자동 경로와 다른 데이터를 보내는
  // 상태였던 셈이다.
  //
  // 카메라가 자기 상태에서 다시 조립하므로 자동 경로와 **정의상 같은 번들**이
  // 나간다. 대시보드가 알아야 할 것은 "나갔는가/왜 안 나갔는가" 둘뿐이다.
  if (strncmp(cmd, "ODOM_RESEND", 11) == 0) {
    int ch = -1;
    if (sscanf(cmd + 11, "%d", &ch) != 1 || !ValidCh(ch)) {
      printf("[ArucoPosePNM] ODOM_RESEND 인자 이상 — 사용법: ODOM_RESEND <ch>\n");
      fflush(stdout);
      return true;
    }
    const char* why = "";
    const bool ok = SendCalibBundle(ch, "manual-resend", &why);
    char json[320];
    snprintf(json, sizeof(json),
             "{\"type\":\"ODOM_RESEND\",\"ch\":%d,\"ok\":%s,\"reason\":\"%s\"}",
             ch, ok ? "true" : "false", ok ? "" : why);
    pose_sender_send_control_line(json);
    return true;
  }

  // FLOOR_RESEND <ch> — 정적 앵커 캘리로 만든 번들을 지금 다시 서버로 보낸다.
  // ODOM_RESEND 와 같은 역할이고, 대시보드가 hgHfloor/mpPlane/kCalib 캐시로
  // 번들을 조립해 CENTRAL_HMATRIX 로 보내던 경로를 이걸로 대체한다 (2026-08-14).
  //
  // 그 캐시 조립이 실제로 틀렸던 값들: image_size 는 드롭다운 추정값,
  // coord_mode 는 HG_COORD_MODE 명령 이력에서 유추(새로고침하면 소실),
  // K/D 는 CALIB_K_QUERY 를 놓치면 자리표시자 [[1400,0,960],…]. 셋 다 카메라만
  // 아는 사실인데 대시보드가 추측하고 있었다.
  if (strncmp(cmd, "FLOOR_RESEND", 12) == 0) {
    int ch = -1;
    if (sscanf(cmd + 12, "%d", &ch) != 1 || !ValidCh(ch)) {
      printf("[ArucoPosePNM] FLOOR_RESEND 인자 이상 — 사용법: FLOOR_RESEND <ch>\n");
      fflush(stdout);
      return true;
    }
    const char* why = "";
    const bool ok = SendFloorBundle(ch, "manual-resend", &why);
    char json[320];
    snprintf(json, sizeof(json),
             "{\"type\":\"FLOOR_RESEND\",\"ch\":%d,\"ok\":%s,\"reason\":\"%s\"}",
             ch, ok ? "true" : "false", ok ? "" : why);
    pose_sender_send_control_line(json);
    return true;
  }

  // MARKER_PLANE_QUERY <ch> — the derived plane and the camera pose it implies,
  // on the pose link. /status carries the same thing; this exists because the
  // RPi drives the vocabulary over TCP and has no /status to poll.
  if (strncmp(cmd, "MARKER_PLANE_QUERY", 18) == 0) {
    int ch = -1;
    if (sscanf(cmd + 18, "%d", &ch) != 1) return true;
    ReportMarkerPlane(ch);
    return true;
  }

  // HG_MAP <ch> <px> <py> — map one raw sensor pixel through this lens's
  // coordinate mode and H, and report the world millimetres.
  //
  // Not in cctv_app: there, the mapping was only ever visible through the pose
  // packets, which need a marker physically in view. Here the undistort path
  // and H have to be checkable on their own, before anything is fitted from
  // real anchors — otherwise the first time the whole chain runs is also the
  // first time it could be wrong, with a marker and a tape measure in the way
  // of finding out which half broke.
  if (strncmp(cmd, "HG_MAP", 6) == 0) {
    int ch = -1;
    double px = 0.0, py = 0.0;
    if (sscanf(cmd + 6, "%d %lf %lf", &ch, &px, &py) != 3) return true;
    double wx = 0.0, wy = 0.0;
    char json[256];
    hg_map_.ch = ch;
    hg_map_.px = px;
    hg_map_.py = py;
    // The reason comes from the mapper, not from guessing here: "no H", "no
    // K/dist" and "undistortion did not converge at this pixel" are three
    // different problems with three different fixes, and only the mapper knows
    // which one it hit.
    const char* why = "";
    hg_map_.ok = homography_.PixelToWorld(ch, (float)px, (float)py, &wx, &wy, &why);
    hg_map_.wx = hg_map_.ok ? wx : 0.0;
    hg_map_.wy = hg_map_.ok ? wy : 0.0;
    hg_map_.reason = hg_map_.ok ? "" : (why && *why ? why : "지평선 (매핑 불가)");
    if (hg_map_.ok) {
      snprintf(json, sizeof(json),
               "{\"type\":\"HG_MAP\",\"ch\":%d,\"px\":%.2f,\"py\":%.2f,"
               "\"ok\":true,\"wx_mm\":%.4f,\"wy_mm\":%.4f,\"undistorted\":%s}",
               ch, px, py, wx, wy,
               homography_.CoordModeUndistorted(ch) ? "true" : "false");
    } else {
      snprintf(json, sizeof(json),
               "{\"type\":\"HG_MAP\",\"ch\":%d,\"px\":%.2f,\"py\":%.2f,"
               "\"ok\":false,\"reason\":\"%s\"}",
               ch, px, py, hg_map_.reason);
    }
    printf("[ArucoPosePNM] %s\n", json);
    fflush(stdout);
    pose_sender_send_control_line(json);
    return true;
  }

  // IVA_SYNC <ch> — the coverage of whichever calibration this lens's H
  // last came from, as a raw-pixel convex hull, for pushing to WiseAI's own
  // IVA area API.
  //
  // Floor and odometry describe "the calibrated area" with two genuinely
  // different point sets, and there is no single blended answer that is
  // right for both, so this follows LastHomographySource(ch) rather than
  // preferring one:
  //   - floor: the REGISTERED ANCHORS (tape-measured points the operator
  //     spread across the floor), not the last fit's detected centres —
  //     those drift frame to frame and would mean something different every
  //     time this command ran. Pixels come from WorldToPixel(anchor world
  //     mm), because floor anchors only exist as world coordinates.
  //   - odometry: the DRIVEN STOP POINTS, whose pixels were never derived
  //     from a world coordinate through H at all — AddOdomPoint() was handed
  //     the pixel directly, so OdomPointRawPixel() only has to undo the
  //     undistort step, not invert H.
  //
  // In RAW sensor pixels either way, because that is the space WiseAI's
  // detector and its IVA area both work in — this lens may fit H in
  // undistorted space, and handing that space to WiseAI unconverted would
  // place the area wrong by exactly the lens distortion.
  //
  // Deliberately does NOT call WiseAI's PUT itself: that endpoint sits behind
  // this camera's own HTTP digest auth, same as this app's own /cmd and
  // /status do, and there is no HTTP client in this codebase to drive it
  // with. This command only computes the polygon and hands it back over
  // /cmd; tools/iva_push.sh does the actual PUT, the same way
  // build_install.sh/calib_backup.sh already drive this app's own endpoints.
  //
  // Fixed-size arrays throughout, NOT std::vector<cv::Point2f>, and the hull
  // is ConvexHull2f() (above), NOT cv::convexHull() — two independent
  // triggers of the identical symptom, both found by bisecting this handler
  // command-by-command on 2026-08-18: a locally constructed
  // std::vector<cv::Point2f> (build, push_back, let it go out of scope), and
  // any linked call site to cv::convexHull() at all (crashed even with the
  // call never actually reached — see ConvexHull2f()'s comment). Both
  // reliably crashed this app at component load, before any of this
  // function could possibly run. cv::Point2f itself, and cv::Mat headers
  // wrapping existing memory (no allocation), are both already used safely
  // elsewhere in this file (PreparePixel's round-trip check) — it is
  // specifically that vector specialisation's construction and that one
  // OpenCV algorithm entry point that are not safe on this platform, for a
  // reason not fully root-caused. Anchors are capped at kMaxAnchors anyway,
  // so a fixed buffer costs nothing here.
  // IVA_ZONE_SET <ch> <n> <x1> <y1> ... — arm a channel's foot-point zone
  // from an explicit polygon instead of deriving one from anchors.
  //
  // Exists because IVA_SYNC cannot survive a restart: it builds the zone from
  // the registered anchor list, and anchors live only in memory unless
  // ANCHOR_SAVE was run, so every reinstall disarms every channel with no way
  // back short of redoing the calibration. This command re-arms in one line --
  // the sender (the Pi, which already keeps the last IVA_SYNC reply) can
  // replay it on reconnect.
  //
  // It also decouples the zone from calibration entirely: a channel with no
  // homography can still have a zone, since judging foot-in-polygon is pure
  // pixel geometry. Only the world-mm fields of ZONE_EVENT need H, and those
  // are already optional.
  //
  // Points are RAW SENSOR pixels -- the same space IVA_SYNC reports and
  // WiseAI's own area API uses, so a polygon can be moved between them
  // unchanged. A declared `n` for the same reason ANCHOR_SET_ALL has one: a
  // truncated coordinate list is still well-formed, so only a count makes the
  // sender's intent checkable.
  if (strncmp(cmd, "IVA_ZONE_SET", 12) == 0) {
    int ch = -1, n = -1, consumed = 0;
    if (sscanf(cmd + 12, "%d %d%n", &ch, &n, &consumed) != 2 || !ValidCh(ch)) {
      printf("[ArucoPosePNM] IVA_ZONE_SET 인자 이상 — 사용법: "
             "IVA_ZONE_SET <ch> <n> <x1> <y1> ...\n");
      fflush(stdout);
      return true;
    }
    if (n < 3 || n > HomographyMapper::kMaxAnchors) {
      printf("[ArucoPosePNM] IVA_ZONE_SET 거부 — 점 개수 %d, 3..%d 여야 함\n", n,
             HomographyMapper::kMaxAnchors);
      fflush(stdout);
      return true;
    }
    float xs[HomographyMapper::kMaxAnchors], ys[HomographyMapper::kMaxAnchors];
    const char* p = cmd + 12 + consumed;
    int got = 0;
    for (; got < n; ++got) {
      int used = 0;
      if (sscanf(p, "%f %f%n", &xs[got], &ys[got], &used) != 2) break;
      p += used;
    }
    if (got != n) {
      printf("[ArucoPosePNM] IVA_ZONE_SET 거부 — %d개를 선언했는데 %d개만 읽힘\n", n, got);
      fflush(stdout);
      return true;
    }
    for (int i = 0; i < n; ++i) {
      iva_zone_[ch].px[i] = xs[i];
      iva_zone_[ch].py[i] = ys[i];
    }
    iva_zone_[ch].n = n;
    // Same reset as IVA_SYNC's: every remembered side belongs to the old zone.
    for (int s = 0; s < kMaxRecentWiseAiObjects; ++s)
      recent_wiseai_obj_[ch][s].zone_state = -1;
    FillZoneWorld(ch);
    SendZoneBands(ch);

    char json[900];
    int w = snprintf(json, sizeof(json),
                     "{\"type\":\"IVA_ZONE_SET\",\"ch\":%d,\"ok\":true,\"n\":%d,\"points\":[",
                     ch, n);
    for (int i = 0; i < n && w > 0 && w < (int)sizeof(json); ++i) {
      w += snprintf(json + w, sizeof(json) - w, "%s{\"x\":%.1f,\"y\":%.1f}",
                    (i == 0) ? "" : ",", xs[i], ys[i]);
    }
    if (w > 0 && w < (int)sizeof(json) - 4) snprintf(json + w, sizeof(json) - w, "]}");
    printf("[ArucoPosePNM] %s\n", json);
    fflush(stdout);
    pose_sender_send_control_line(json);
    return true;
  }

  if (strncmp(cmd, "IVA_SYNC", 8) == 0) {
    int ch = -1;
    if (sscanf(cmd + 8, "%d", &ch) != 1 || !ValidCh(ch)) {
      printf("[ArucoPosePNM] IVA_SYNC 인자 이상 — 사용법: IVA_SYNC <ch>\n");
      fflush(stdout);
      return true;
    }
    iva_sync_.ch = ch;
    iva_sync_.ok = false;
    iva_sync_.n = 0;
    iva_sync_.reason[0] = '\0';

    const HomographyMapper::LastHSource src = homography_.LastHomographySource(ch);
    if (src == HomographyMapper::LastHSource::kNone) {
      CopyUtf8(iva_sync_.reason, sizeof(iva_sync_.reason),
               "이 채널은 아직 호모그래피가 없습니다 (floor 또는 odometry 캘리를 먼저 완료할 것)");
    } else {
      cv::Point2f pts_buf[HomographyMapper::kMaxAnchors];
      int n_pts = 0;
      const char* why = "";
      int total = 0;  // for the "N/total 성공" message below
      if (src == HomographyMapper::LastHSource::kFloor) {
        total = homography_.AnchorCount(ch);
        for (int i = 0; i < total && n_pts < HomographyMapper::kMaxAnchors; ++i) {
          Anchor a;
          if (!homography_.AnchorAt(ch, i, &a)) continue;
          float px = 0.0f, py = 0.0f;
          if (homography_.WorldToPixel(ch, a.wx_mm, a.wy_mm, &px, &py, &why)) {
            pts_buf[n_pts++] = cv::Point2f(px, py);
          }
        }
      } else {  // kOdom
        total = homography_.OdomCount(ch);
        for (int i = 0; i < total && n_pts < HomographyMapper::kMaxAnchors; ++i) {
          float px = 0.0f, py = 0.0f;
          if (homography_.OdomPointRawPixel(ch, i, &px, &py, &why)) {
            pts_buf[n_pts++] = cv::Point2f(px, py);
          }
        }
      }
      if (n_pts < 3) {
        snprintf(iva_sync_.reason, sizeof(iva_sync_.reason),
                 "%s: 픽셀 변환 가능한 지점이 3개 미만입니다 (%d/%d 성공) — %s",
                 (src == HomographyMapper::LastHSource::kFloor) ? "floor" : "odometry",
                 n_pts, total, (why && *why) ? why : "");
      } else {
        cv::Point2f hull_buf[HomographyMapper::kMaxAnchors];
        iva_sync_.n = ConvexHull2f(pts_buf, n_pts, hull_buf, HomographyMapper::kMaxAnchors);
        for (int i = 0; i < iva_sync_.n; ++i) {
          iva_sync_.px[i] = hull_buf[i].x;
          iva_sync_.py[i] = hull_buf[i].y;
        }
        iva_sync_.ok = (iva_sync_.n > 0);
        // Arm this channel's own foot-point zone from the same hull we are
        // about to hand the operator to push to WiseAI, so the two agree by
        // construction. Only on success: a failed sync leaves the previous
        // zone standing rather than disarming the channel silently.
        if (iva_sync_.ok) {
          iva_zone_[ch].n = iva_sync_.n;
          for (int i = 0; i < iva_sync_.n; ++i) {
            iva_zone_[ch].px[i] = iva_sync_.px[i];
            iva_zone_[ch].py[i] = iva_sync_.py[i];
          }
          // Every track's remembered side belongs to the OLD zone; keeping it
          // would let the first frame after a re-sync read as a crossing that
          // the person never made.
          for (int s = 0; s < kMaxRecentWiseAiObjects; ++s)
            recent_wiseai_obj_[ch][s].zone_state = -1;
          FillZoneWorld(ch);
          SendZoneBands(ch);
        }
      }
    }

    char json[900];
    int w = snprintf(json, sizeof(json),
                     "{\"type\":\"IVA_SYNC\",\"ch\":%d,\"ok\":%s,\"n\":%d,\"points\":[",
                     ch, iva_sync_.ok ? "true" : "false", iva_sync_.n);
    for (int i = 0; i < iva_sync_.n && w > 0 && w < (int)sizeof(json); ++i) {
      w += snprintf(json + w, sizeof(json) - w, "%s{\"x\":%.1f,\"y\":%.1f}",
                    (i == 0) ? "" : ",", iva_sync_.px[i], iva_sync_.py[i]);
    }
    if (w > 0 && w < (int)sizeof(json) - 16) {
      snprintf(json + w, sizeof(json) - w, "],\"reason\":\"%s\"}", iva_sync_.reason);
    }
    printf("[ArucoPosePNM] %s\n", json);
    fflush(stdout);
    pose_sender_send_control_line(json);
    return true;
  }

  return false;
}

/**
 * ANCHOR_* — the registered marker list.
 *
 *   ANCHOR_SET_ALL <ch> <count> <id> <wx> <wy> [<id> <wx> <wy> ...]
 *   ANCHOR_QUERY   <ch>
 *   ANCHOR_SAVE    <ch>
 *
 * `<count>` is NOT in the port plan's command list (§4.1), and is the one
 * deliberate addition. It exists because this is the only command in the app
 * whose payload can exceed a transport's line budget, and every layer that
 * budget passes through truncates SILENTLY: POST /cmd resized the body, and
 * pose_sender_poll_command() clips a long line to the caller's buffer. Both are
 * now sized to hold 24 markers and both report an oversized line rather than
 * trimming it — but a cut still cannot be detected from the text alone, because
 * chopping "... 11 1234.5 -6789.0" mid-number leaves "... 11 1234.5 -6" which
 * is a perfectly well-formed triple. The result would be a marker list that is
 * complete-looking, one point short or one point wrong, and fitted from without
 * complaint. A declared count makes the sender's intent checkable: parse
 * exactly `count` triples, and refuse if the line ran out early or ran on late.
 *
 * ANCHOR_SET_ALL 0 0 empties a lens's list — the count makes that unambiguous,
 * where a bare "ANCHOR_SET_ALL 0" could equally be a command whose arguments
 * were lost.
 *
 * Saving is a separate command, as it is for K/dist and H: a value that
 * persists itself on success is a value whose save failure nobody sees.
 */
bool SampleComponent::HandleAnchors(const char* cmd) {
  if (strncmp(cmd, "ANCHOR_SET_ALL", 14) == 0) {
    const char* p = cmd + 14;
    char* end = NULL;

    const long ch = strtol(p, &end, 10);
    if (end == p) {
      SetAnchorResult(-1, false, "ANCHOR_SET_ALL: 채널 번호가 없습니다");
      return true;
    }
    p = end;
    const long want = strtol(p, &end, 10);
    if (end == p) {
      SetAnchorResult((int)ch, false,
                      "ANCHOR_SET_ALL: 마커 개수가 없습니다 "
                      "(ANCHOR_SET_ALL <ch> <개수> <id> <wx> <wy> ...)");
      return true;
    }
    p = end;
    if (want < 0 || want > HomographyMapper::kMaxAnchors) {
      SetAnchorResult((int)ch, false,
                      "ANCHOR_SET_ALL: 마커 개수가 0..24 범위를 벗어났습니다");
      return true;
    }

    Anchor list[HomographyMapper::kMaxAnchors];
    int got = 0;
    while (got < (int)want) {
      const long id = strtol(p, &end, 10);
      if (end == p) break;
      p = end;
      const double wx = strtod(p, &end);
      if (end == p) break;
      p = end;
      const double wy = strtod(p, &end);
      if (end == p) break;
      p = end;
      list[got].id = (int)id;
      list[got].wx_mm = wx;
      list[got].wy_mm = wy;
      ++got;
    }

    if (got != (int)want) {
      // The likely cause is a line that got cut, so say so — the alternative
      // ("bad argument") sends the operator hunting for a typo in numbers that
      // were all correct when they were sent.
      char why[160];
      snprintf(why, sizeof(why),
               "ANCHOR_SET_ALL: %ld개를 받기로 했는데 %d개에서 끊겼습니다 "
               "— 명령줄이 잘렸을 수 있습니다",
               want, got);
      SetAnchorResult((int)ch, false, why);
      return true;
    }
    while (*p == ' ' || *p == '\t' || *p == '\r') ++p;
    if (*p != '\0') {
      // More data than promised. The count and the payload disagree, and there
      // is no way to tell which one the operator meant.
      char why[160];
      snprintf(why, sizeof(why),
               "ANCHOR_SET_ALL: %ld개라고 했는데 뒤에 내용이 더 있습니다 "
               "— 개수와 목록이 어긋납니다",
               want);
      SetAnchorResult((int)ch, false, why);
      return true;
    }

    const bool ok = homography_.SetAnchors((int)ch, list, got);
    SetAnchorResult((int)ch, ok,
                    ok ? "" : homography_.FailReason());
    if (ok) ReportAnchors((int)ch);
    return true;
  }

  if (strncmp(cmd, "ANCHOR_QUERY", 12) == 0) {
    int ch = -1;
    if (sscanf(cmd + 12, "%d", &ch) != 1) return true;
    ReportAnchors(ch);
    return true;
  }

  if (strncmp(cmd, "ANCHOR_SAVE", 11) == 0) {
    int ch = -1;
    if (sscanf(cmd + 11, "%d", &ch) != 1) return true;
    const bool ok = homography_.SaveAnchors(ch);
    SetAnchorResult(ch, ok,
                    ok ? "" : homography_.FailReason());
    return true;
  }

  return false;
}

// Record an ANCHOR_* outcome for /status, and echo it to stdout. One place so
// that no path can report to the log and forget the status object, which is the
// only one an operator driving this by curl will ever see.
void SampleComponent::SetAnchorResult(int ch, bool ok, const char* reason) {
  anchor_cmd_.ch = ch;
  // Read here rather than passed in. Every call site was computing the same
  // AnchorCount(ch), including one whose ternary existed only to restate it —
  // and the struct's contract is "what this lens HAS now", which a caller
  // passing the count it TRIED to set would silently break.
  anchor_cmd_.n = homography_.AnchorCount(ch);
  anchor_cmd_.ok = ok;
  snprintf(anchor_cmd_.reason, sizeof(anchor_cmd_.reason), "%s", reason ? reason : "");
  printf("[ArucoPosePNM] ANCHOR ch%d %s (%d개)%s%s\n", ch, ok ? "적용" : "거부", anchor_cmd_.n,
         anchor_cmd_.reason[0] ? " — " : "", anchor_cmd_.reason);
  fflush(stdout);
}

/**
 * ODOM_SETTLE / ODOM_SETTLE_QUERY 의 답.
 *
 * `max_frames` 를 같이 싣는 이유: 상한이 컴파일 시점의 버퍼 크기라 UI 가
 * 스스로 알 방법이 없다. 이걸 안 주면 대시보드가 자기 마음대로 상한을 정하고,
 * 그 값이 펌웨어와 갈리면 "입력은 받았는데 조용히 잘리는" 상태가 된다.
 */
void SampleComponent::ReportOdomSettle() const {
  char json[192];
  snprintf(json, sizeof(json),
           "{\"type\":\"ODOM_SETTLE\",\"spread_px\":%.2f,\"frames\":%d,\"max_frames\":%d}",
           g_odom_settle_spread_px, g_odom_settle_frames, ODOM_SETTLE_FRAMES);
  pose_sender_send_control_line(json);
}

/**
 * ODOM_PREFER / ODOM_PREFER_QUERY 의 답. 포즈 링크로 나간다.
 *
 * 지금 어느 슬롯이 로봇 측위에 쓰이는지를 한 줄로 말한다. `measured_ready` 를
 * 같이 싣는 이유: 측정값이 없으면 SetPreferMeasured() 가 거부하는데, 그 사유를
 * "왜 안 켜지지"로 되묻지 않게 하려는 것이다.
 *
 * stale 두 가지도 싣는다 — K 가 교체됐거나 마커 높이가 바뀌었으면 그 행렬은
 * 잴 당시와 다른 조건의 것이다. 버리지는 않고 표시만 하는 것이 이 앱의 규칙
 * (CalibStale() 과 같은 판단)이므로, 켜기 전에 이 두 값을 보라는 뜻이다.
 */
void SampleComponent::ReportOdomPrefer(int ch, bool ok) const {
  if (!ValidCh(ch)) return;
  char json[256];
  snprintf(json, sizeof(json),
           "{\"type\":\"ODOM_PREFER\",\"ch\":%d,\"ok\":%s,\"preferred\":%s,"
           "\"measured_ready\":%s,\"k_stale\":%s,\"height_stale\":%s,\"reason\":\"%s\"}",
           ch, ok ? "true" : "false",
           homography_.PreferMeasured(ch) ? "true" : "false",
           homography_.MeasuredMarkerPlaneReady(ch) ? "true" : "false",
           homography_.MeasuredKStale(ch) ? "true" : "false",
           homography_.MeasuredHeightStale(ch) ? "true" : "false",
           ok ? "" : homography_.FailReason());
  pose_sender_send_control_line(json);
}

/**
 * ANCHOR_QUERY's answer on the pose link.
 *
 * %.1f: these are tape measurements, so a tenth of a millimetre is already
 * finer than the thing being reported. The wide format matters for H, which is
 * defined up to scale — it does not matter here, and at 24 markers the
 * difference is most of a line budget.
 */
void SampleComponent::ReportAnchors(int ch) const {
  if (ch < 0 || ch >= HomographyMapper::kChannels) return;
  char json[POSE_SENDER_MAX_LINE];
  int w = snprintf(json, sizeof(json),
                   "{\"type\":\"ANCHORS\",\"ch\":%d,\"n\":%d,\"markers\":[", ch,
                   homography_.AnchorCount(ch));
  for (int i = 0; i < homography_.AnchorCount(ch) && w > 0 && w < (int)sizeof(json); ++i) {
    Anchor a;
    if (!homography_.AnchorAt(ch, i, &a)) break;
    w += snprintf(json + w, sizeof(json) - w, "%s{\"id\":%d,\"wx\":%.1f,\"wy\":%.1f}",
                  (i == 0) ? "" : ",", a.id, a.wx_mm, a.wy_mm);
  }
  if (w > 0 && w < (int)sizeof(json) - 3) {
    snprintf(json + w, sizeof(json) - w, "]}");
    pose_sender_send_control_line(json);
  }
}

void SampleComponent::ReportMarkerPlane(int ch) const {
  if (ch < 0 || ch >= HomographyMapper::kChannels) return;
  char json[768];
  double hm[9];
  double cz = 0.0, nx = 0.0, ny = 0.0;
  const char* why = "";
  const bool pose_ok = homography_.CameraPose(ch, &cz, &nx, &ny, &why);
  const bool have = homography_.GetMarkerPlane(ch, hm);

  if (!have) {
    snprintf(json, sizeof(json),
             "{\"type\":\"MARKER_PLANE\",\"ch\":%d,\"ready\":false,"
             "\"height_mm\":%.1f,\"reason\":\"%s\"}",
             ch, homography_.MarkerHeightMm(), homography_.MarkerPlaneReason(ch));
  } else {
    // camera_z_mm is what was TAPE-MEASURED (0 = nobody measured); derived_z_mm
    // is what the matrix implies. Reporting both is the point — they should
    // agree, and the size of the disagreement is the size of the scale error
    // the measurement is there to remove.
    snprintf(json, sizeof(json),
             "{\"type\":\"MARKER_PLANE\",\"ch\":%d,\"ready\":true,"
             "\"height_mm\":%.1f,\"camera_z_mm\":%.1f,"
             "\"derived_z_mm\":%.1f,\"nadir_mm\":[%.1f,%.1f],\"pose_ok\":%s,"
             "\"H_marker\":[%.9e,%.9e,%.9e,%.9e,%.9e,%.9e,%.9e,%.9e,%.9e]}",
             ch, homography_.MarkerHeightMm(), homography_.CameraHeightMm(ch),
             pose_ok ? cz : -1.0, pose_ok ? nx : 0.0, pose_ok ? ny : 0.0,
             pose_ok ? "true" : "false",
             hm[0], hm[1], hm[2], hm[3], hm[4], hm[5], hm[6], hm[7], hm[8]);
  }
  printf("[ArucoPosePNM] %s\n", json);
  fflush(stdout);
  pose_sender_send_control_line(json);
}

/**
 * One HG line on the pose link, mirroring ReportDynRoi().
 *
 * The on-camera page does not need this — /status already carries every
 * matrix. It exists because the RPi drives the same command vocabulary over
 * TCP and has no /status to poll, and because an injection that produced no
 * reply is indistinguishable from one that never arrived.
 *
 * %.9e per entry: H is defined only up to scale, so the exponent range within
 * one matrix is wide and a fixed-point format would flatten h20/h21 to zero.
 */
void SampleComponent::ReportHomography(int ch) const {
  double h[9];
  char json[512];
  if (!homography_.Get(ch, h)) {
    snprintf(json, sizeof(json),
             "{\"type\":\"HG\",\"ch\":%d,\"available\":false,\"mode_undistorted\":%s}",
             ch, homography_.CoordModeUndistorted(ch) ? "true" : "false");
  } else {
    snprintf(json, sizeof(json),
             "{\"type\":\"HG\",\"ch\":%d,\"available\":true,\"undistorted\":%s,"
             "\"mappable\":%s,\"camera_z_mm\":%.1f,"
             "\"H\":[%.9e,%.9e,%.9e,%.9e,%.9e,%.9e,%.9e,%.9e,%.9e]}",
             ch, homography_.FittedUndistorted(ch) ? "true" : "false",
             homography_.Mappable(ch) ? "true" : "false",
             homography_.CameraHeightMm(ch),
             h[0], h[1], h[2], h[3], h[4], h[5], h[6], h[7], h[8]);
  }
  pose_sender_send_control_line(json);
}

void SampleComponent::ReportHomographyFit(int ch) const {
  const HomographyMapper::FitResult& fr = homography_.Residuals(ch);
  char json[256];
  if (!fr.have) {
    snprintf(json, sizeof(json), "{\"type\":\"HG_FIT\",\"ch\":%d,\"have\":false}", ch);
  } else {
    snprintf(json, sizeof(json),
             "{\"type\":\"HG_FIT\",\"ch\":%d,\"have\":true,\"n\":%d,"
             "\"loo_valid\":%s,\"advisory\":%s,"
             "\"rmse_in_mm\":%.1f,\"rmse_loo_mm\":%.1f,"
             "\"max_loo_mm\":%.1f,\"max_loo_id\":%d}",
             ch, fr.n, fr.loo_valid ? "true" : "false", fr.advisory ? "true" : "false",
             fr.rmse_in_mm, fr.rmse_loo_mm, fr.max_loo_mm, fr.max_loo_id);
  }
  pose_sender_send_control_line(json);
}

// One line per point, not one line for all of them: kMaxAnchors (24) points
// at ~35 B each plus the ch/id/in/loo fields would run past
// POSE_SENDER_MAX_LINE (1024 B) and get silently cut mid-object by the
// transport, which is worse than not sending it — a truncated line is
// invalid JSON with no indication of where it broke. Each of these fits in a
// fraction of the budget on its own.
void SampleComponent::ReportHgAck(int ch) const {
  char json[64];
  snprintf(json, sizeof(json), "{\"type\":\"CALIB_ACK\",\"ch\":%d}", ch);
  pose_sender_send_control_line(json);
}

// Terminal signal for a CALIB_START session — see the header. `ok` is whether an
// H is now available on this lens (a session that ran out of frames without one
// leaves ok=false), and `result` is the same human-readable outcome /status
// shows. last_result carries only text this code wrote (no JSON escaping needed,
// same as the /status session block) and fits well inside json[] here.
void SampleComponent::ReportHgDone(int ch) const {
  const HomographyMapper::FitProgress& fp = homography_.Fit(ch);
  char json[384];
  snprintf(json, sizeof(json),
           "{\"type\":\"HG_DONE\",\"ch\":%d,\"ok\":%s,\"good\":%d,\"total\":%d,\"result\":\"%s\"}",
           ch, homography_.Available(ch) ? "true" : "false", fp.good, fp.total, fp.last_result);
  pose_sender_send_control_line(json);
}

void SampleComponent::ReportHomographyFitPoints(int ch) const {
  const HomographyMapper::FitResult& fr = homography_.Residuals(ch);
  if (!fr.have) return;
  char json[96];
  for (int i = 0; i < fr.n; ++i) {
    snprintf(json, sizeof(json),
             "{\"type\":\"HG_FIT_PT\",\"ch\":%d,\"id\":%d,\"in_mm\":%.1f,\"loo_mm\":%.1f}",
             ch, fr.pt[i].id, fr.pt[i].in_mm, fr.pt[i].loo_mm);
    pose_sender_send_control_line(json);
  }
}

#if ENABLE_CENTRAL_TLS_STREAM
/**
 * CENTRAL_LINK <0|1>          -- drop or restore the TLS session itself.
 * CENTRAL_POS <0|1>           -- stop sending POS but stay registered with the server.
 * CENTRAL_ID <n>              -- retarget the POS stream (which marker id is "the robot").
 * CENTRAL_QUERY               -- redraw the dashboard's central-server tab from camera state.
 * CENTRAL_HMATRIX <payload>   -- forward a calibration bundle to the server as H_MATRIX.
 *
 * Same vocabulary cctv_app's dashboard tab already speaks, ported so the tab
 * does not need a second implementation to talk to this camera.
 */
bool SampleComponent::HandleCentral(const char* cmd) {
  if (strncmp(cmd, "CENTRAL_LINK", 12) == 0) {
    int on = 0;
    if (sscanf(cmd + 12, "%d", &on) == 1) central_tls_sender_set_enabled(on);
    ReportCentral("link", central_tls_sender_enabled() ? "reconnecting" : "disconnected");
    return true;
  }
  if (strncmp(cmd, "CENTRAL_POS", 11) == 0) {
    int on = 0;
    if (sscanf(cmd + 11, "%d", &on) == 1) g_central_pos_enabled = (on != 0);
    ReportCentral("pos", g_central_pos_enabled ? "streaming" : "stopped");
    return true;
  }
  if (strncmp(cmd, "CENTRAL_ID", 10) == 0) {
    int id = 0;
    if (sscanf(cmd + 10, "%d", &id) == 1) g_central_marker_id = id;
    ReportCentral("id", "applied");
    return true;
  }
  if (strncmp(cmd, "CENTRAL_QUERY", 13) == 0) {
    ReportCentral("query", "");
    return true;
  }
  // CENTRAL_HMATRIX <payload json> -- forward a calibration bundle verbatim.
  //
  // This is a straight passthrough, same as cctv_app: the dashboard operator
  // supplies the whole payload (including "ch" — the central protocol reads
  // it from payload's top level, see docs/08.06/CCTV_ACTION_ITEMS_20260806.md
  // C-3) and this app does not re-serialise it from homography_/calib_'s own
  // state. This app, unlike cctv_app, actually HAS K/D/H_marker per channel
  // (homography_mapper.h) and could assemble the bundle itself — deliberately
  // not done here: docs/PROTOCOL.md's mm-vs-m field-by-field convention
  // (H_floor/H_marker stay mm, only marker_height_m converts) needs its own
  // pass, and it's not yet decided whether that should fire automatically on
  // every calibration change or stay operator-triggered like this. Passthrough
  // first, auto-assembly is a follow-up.
  if (strncmp(cmd, "CENTRAL_HMATRIX", 15) == 0) {
    const char* p = cmd + 15;
    while (*p == ' ' || *p == '\t') ++p;
    if (*p != '{') {
      ReportCentral("hmatrix", "payload must be a JSON object");
      return true;
    }
    // Refuse a bundle that contradicts what this app knows about the lens it
    // claims to describe. Still a passthrough — the dashboard owns the format —
    // but the dashboard cannot see fitted_undistorted_ or marker_plane.ready,
    // and the SERVER cannot see them either. Every field checked here is one
    // the receiver takes on trust, so a wrong value is not an error anywhere
    // downstream: Qt reads coord_mode:"undistort", switches lens correction on,
    // and draws pixels that were already corrected. Silent, plausible, wrong.
    //
    // 2026-08-11: CH2 carried a raw-fitted H for hours while the page reported
    // it as calibrated. Nothing in the send path would have stopped that bundle.
    char why[192];
    if (!ValidateHMatrixPayload(p, why, sizeof(why))) {
      ReportCentral("hmatrix", why);
      return true;
    }
    const int rc = central_tls_sender_send_typed("H_MATRIX", p);
    ReportCentral("hmatrix", rc == 0 ? "sent" : "send failed (link down or too long)");
    return true;
  }
  return false;
}

/**
 * Cross-check a CENTRAL_HMATRIX payload against this app's own state.
 *
 * Returns NULL when the bundle may be sent, or a reason to report and drop it.
 *
 * Scanned by hand rather than parsed: the payload is the dashboard's format,
 * not ours, and a parser here would quietly become a second definition of it
 * that has to be kept in step. What this needs is narrower — find a handful of
 * literal keys and compare them with values this app already holds. Same
 * approach ANCHOR_SET_ALL takes with strtol/strtod, for the same reason.
 *
 * Deliberately NOT checked: anything only the dashboard knows (canvas_mm,
 * origin_mm, axis, calib_id, request_id). Refusing on those would make this
 * app an authority on a format it does not own.
 *
 * Absent keys pass. A bundle that omits coord_mode is a format question for
 * whoever consumes it; this function only catches the case where the payload
 * ASSERTS something this app can see is false.
 */
bool SampleComponent::ValidateHMatrixPayload(const char* p, char* reason,
                                             size_t reason_size) const {
  #define HM_REJECT(...) do { snprintf(reason, reason_size, __VA_ARGS__); return false; } while (0)
  // "ch" is 1-based in the envelope (QT_CCTV_SERVER_CALIBRATION_FORMAT §3), the
  // same convention POS uses. Without it there is no lens to check against, and
  // the central protocol requires it regardless.
  const char* q = strstr(p, "\"ch\"");
  if (q == NULL) HM_REJECT("ch 필드가 없습니다 (envelope 최상위, 1-based)");
  q = strchr(q, ':');
  if (q == NULL) HM_REJECT("ch 필드를 읽을 수 없습니다");
  const long ch1 = strtol(q + 1, NULL, 10);
  if (ch1 < 1 || ch1 > kMaxChannels) HM_REJECT("ch 는 1..4 여야 합니다 (1-based)");
  const int ch = (int)ch1 - 1;

  if (!homography_.Available(ch)) HM_REJECT("이 렌즈에 H가 없습니다 — 먼저 계산하세요");

  // coord_mode is read from its own key rather than by looking for the words
  // anywhere in the payload: "undistort" and "raw" are ordinary enough to turn
  // up in a comment or an id, and a guard that refuses a correct bundle is worse
  // than no guard — it teaches the operator to route around it.
  const char* cm = strstr(p, "\"coord_mode\"");
  if (cm != NULL) {
    const char* colon = strchr(cm, ':');
    // The value, bounded so a missing quote cannot walk into the next field.
    const char* val = (colon != NULL) ? strchr(colon, '"') : NULL;
    const char* end = (val != NULL) ? strchr(val + 1, '"') : NULL;
    if (val != NULL && end != NULL) {
      const size_t n = (size_t)(end - val - 1);
      const bool says_undistort = (n == 9 && strncmp(val + 1, "undistort", 9) == 0);
      const bool says_raw       = (n == 3 && strncmp(val + 1, "raw", 3) == 0);
      // The claim the receiver acts on: Qt turns lens correction ON when it sees
      // undistort. A raw-fitted H shipped with this label is corrected twice.
      if (says_undistort && !homography_.FittedUndistorted(ch))
        HM_REJECT("coord_mode=undistort 인데 이 렌즈의 H는 raw 픽셀로 피팅돼 있습니다 "
                  "— HG_CLEAR 후 HG_COORD_MODE 1 로 다시 계산하세요");
      // The reverse is just as wrong: an undistort fit shipped as raw makes the
      // receiver skip a correction the numbers already assume.
      if (says_raw && homography_.FittedUndistorted(ch))
        HM_REJECT("coord_mode=raw 인데 이 렌즈의 H는 undistort 공간에서 피팅됐습니다");
    }
  }

  // H_marker is derived from the fit plus the marker height; if the plane is not
  // ready this app has no such matrix, so whatever is in the payload came from
  // somewhere else.
  if (strstr(p, "\"H_marker\"") != NULL && !homography_.MarkerPlaneReady(ch))
    HM_REJECT("H_marker 가 실려 있는데 이 렌즈의 마커 평면이 준비되지 않았습니다");

  // Both matrices present and different: §3 "H 중복 필드 사용 금지" — Qt may
  // prefer one and the server the other, so the drawing and the robot end up in
  // different places. Compared as text because that is what actually ships; two
  // spellings of the same number are a formatting question, not a safety one,
  // and this only fires when the payload carries both.
  const char* hf = strstr(p, "\"H_floor\"");
  const char* hp = strstr(p, "\"H\"");
  if (hf != NULL && hp != NULL) {
    const char* a = strchr(hf, '[');
    const char* b = strchr(hp, '[');
    if (a != NULL && b != NULL) {
      // Compare to the end of the outer array on each side.
      int da = 0, db = 0;
      const char* ea = a;
      const char* eb = b;
      for (; *ea != '\0'; ++ea) { if (*ea == '[') ++da; else if (*ea == ']' && --da == 0) { ++ea; break; } }
      for (; *eb != '\0'; ++eb) { if (*eb == '[') ++db; else if (*eb == ']' && --db == 0) { ++eb; break; } }
      const size_t la = (size_t)(ea - a), lb = (size_t)(eb - b);
      if (la != lb || memcmp(a, b, la) != 0)
        HM_REJECT("H 와 H_floor 가 함께 있고 값이 다릅니다 — H_floor 하나만 보내세요");
    }
  }

  // image_size must be the frame this lens actually delivers. A bundle fitted on
  // one size and consumed against another is off by that ratio everywhere, and
  // the receiver has no way to notice. Only checked once a frame has arrived.
  const char* is = strstr(p, "\"image_size\"");
  if (is != NULL && last_w_[ch] > 0 && last_h_[ch] > 0) {
    const char* a = strchr(is, '[');
    if (a != NULL) {
      char* end = NULL;
      const long w = strtol(a + 1, &end, 10);
      long h = 0;
      if (end != NULL) {
        const char* c = strchr(end, ',');
        if (c != NULL) h = strtol(c + 1, NULL, 10);
      }
      if (w > 0 && h > 0 && (w != last_w_[ch] || h != last_h_[ch]))
        HM_REJECT("image_size %ldx%ld 가 이 렌즈의 실제 프레임 %dx%d 와 다릅니다",
                  w, h, last_w_[ch], last_h_[ch]);
    }
  }
  reason[0] = '\0';
  return true;
  #undef HM_REJECT
}

void SampleComponent::ReportCentral(const char* action, const char* detail) const {
  char json[512];
  snprintf(json, sizeof(json),
           "{\"type\":\"CENTRAL_STATUS\",\"link\":\"%s\",\"link_on\":%s,"
           "\"pos_on\":%s,\"marker_id\":%d,\"server\":\"%s:%d\","
           "\"action\":\"%s\",\"detail\":\"%s\"}",
           central_tls_sender_state(), central_tls_sender_enabled() ? "true" : "false",
           g_central_pos_enabled ? "true" : "false", g_central_marker_id,
           CENTRAL_TLS_SERVER_IP, CENTRAL_TLS_SERVER_PORT,
           action ? action : "", detail ? detail : "");
  pose_sender_send_control_line(json);
}

/**
 * Commands the central server itself sends over the TLS link -- a much
 * smaller vocabulary than the RPi dashboard's, and not funnelled through
 * DispatchCommand()/HandleCentral() above: those answer commands from the
 * OTHER transport (pose_sender), and CALIB_START/SELECT_CHANNEL arrive bare
 * (no "CENTRAL_" prefix, per docs/PROTOCOL.md) so mixing the two tables
 * would risk a collision with an unrelated dashboard command.
 *
 * SELECT_CHANNEL is accepted and acknowledged but otherwise a no-op here:
 * this app already streams every channel's POS tagged with its own "ch"
 * (see SendPosePackets), so there is no single "active channel" for the app
 * itself to switch -- the server is the one deciding which channel's POS it
 * currently trusts. See docs/08.06/CCTV_ACTION_ITEMS_20260806.md C-0/C-2 for
 * the multi-instance case this app does not have.
 */
void SampleComponent::PollCentralCommands() {
  char cmd[CENTRAL_TLS_MAX_LINE];
  while (central_tls_sender_poll_command(cmd, sizeof(cmd))) {
    if (strcmp(cmd, "CALIB_START") == 0) {
      // Not wired to a calibration action yet -- this app's own K/homography
      // sessions are already addressable per-channel from the RPi dashboard
      // (CALIB_K_START <ch>, HG_*), and it is not yet decided what a
      // channel-less central CALIB_START should trigger here. Logged, not
      // silently dropped, so the gap is visible instead of looking like a
      // dead command.
      printf("[ArucoPosePNM] central CALIB_START received (not wired to an action)\n");
    } else if (strncmp(cmd, "SELECT_CHANNEL ", 15) == 0) {
      printf("[ArucoPosePNM] central SELECT_CHANNEL %s (no-op: all channels stream already)\n",
             cmd + 15);
    } else if (strcmp(cmd, "SELECT_CHANNEL_BAD") == 0) {
      printf("[ArucoPosePNM] central SELECT_CHANNEL with missing/out-of-range ch\n");
    } else if (strncmp(cmd, "CALIB_CAPTURE ", 14) == 0) {
      // "CALIB_CAPTURE <ch> <point_index> <x_mm> <y_mm> <request_id>"
      // request_id 는 맨 뒤이고 공백이 없다(central_cmd_parse 가 '_' 로 치환).
      int wire_ch = 0, idx = -1;
      double wx = 0.0, wy = 0.0;
      char rid[64] = "";
      const int got = sscanf(cmd + 14, "%d %d %lf %lf %63s",
                             &wire_ch, &idx, &wx, &wy, rid);
      // 파서가 이미 걸렀지만 한 번 더 본다 — 이 값들은 그대로 캘리 데이터가 된다.
      if (got < 4 || !ValidCh(wire_ch - 1)) {
        printf("[ArucoPosePNM] central CALIB_CAPTURE 인자 이상 (got=%d ch=%d)\n", got, wire_ch);
      } else {
        CentralCalibCapture(wire_ch - 1, idx, wx, wy, rid);   // 1-based -> 0-based
      }
    } else if (strncmp(cmd, "CALIB_DONE ", 11) == 0) {
      // "CALIB_DONE <ch> <m_mm> <n_mm> <request_id>"
      int wire_ch = 0;
      double m = 0.0, n = 0.0;
      char rid[64] = "";
      const int got = sscanf(cmd + 11, "%d %lf %lf %63s", &wire_ch, &m, &n, rid);
      if (got < 3 || !ValidCh(wire_ch - 1)) {
        printf("[ArucoPosePNM] central CALIB_DONE 인자 이상 (got=%d ch=%d)\n", got, wire_ch);
      } else {
        CentralCalibDone(wire_ch - 1, m, n, rid);
      }
    } else if (strncmp(cmd, "CALIB_CANCEL ", 13) == 0) {
      // "CALIB_CANCEL <ch> <request_id>"
      int wire_ch = 0;
      char rid[64] = "";
      const int got = sscanf(cmd + 13, "%d %63s", &wire_ch, rid);
      // ch 가 이상해도 취소는 수행한다 — 여기서 그냥 로그만 남기고 빠지면
      // 서버가 5초를 기다린 뒤 cancel_failed 로 접는다.
      if (got < 1 || !ValidCh(wire_ch - 1)) {
        printf("[ArucoPosePNM] central CALIB_CANCEL 채널 이상 (got=%d ch=%d) — 전체 취소로 처리\n",
               got, wire_ch);
        CentralCalibCancel(-1, rid);
      } else {
        CentralCalibCancel(wire_ch - 1, rid);        // 1-based -> 0-based
      }
    } else if (strcmp(cmd, "CALIB_CANCEL_BAD") == 0) {
      // 파서가 ch 를 못 읽은 취소. 어느 렌즈인지 모르므로 열려 있는 주행
      // 세션을 전부 접고 ack 한다 — 헤더 CentralCalibCancel() 주석 참고.
      printf("[ArucoPosePNM] central CALIB_CANCEL — ch 누락/범위 밖, 전체 취소로 처리\n");
      CentralCalibCancel(-1, "");
    } else if (strcmp(cmd, "CALIB_CAPTURE_BAD") == 0 ||
               strcmp(cmd, "CALIB_DONE_BAD") == 0) {
      // 파서가 필드 부족/범위 밖으로 판정한 줄. 어느 지점인지조차 모르므로
      // 서버에 개별 실패를 돌려줄 수 없다 — 서버는 자기 타임아웃으로 접는다.
      printf("[ArucoPosePNM] central %s — 필드 누락 또는 범위 밖\n", cmd);
    }
    fflush(stdout);
  }
}

/**
 * 주행 캘리 한 지점의 캡처 요청.
 *
 * 여기서는 요청 시각(t_capture 기준선)을 잡고 프레임 경로를 무장시키는 데까지
 * 한다. 정지 판정과 저장은 OdomFeedFrame() 이 다음 프레임부터 이어받는다.
 *
 * 어느 경로로 끝나든 **반드시 답한다** — 서버는 ack 를 받아야 다음 지점으로
 * GO 를 보내므로(계획서 §3-1), 조용히 빠지면 세션이 타임아웃까지 멈춰 서고
 * 조작자에게는 "로봇이 안 움직인다"로만 보인다.
 */
void SampleComponent::CentralCalibCapture(int ch, int point_index,
                                          double wx_mm, double wy_mm,
                                          const char* request_id) {
  const long now = epoch_ms();

  // 첫 지점이 세션을 연다. 서버가 CALIB_START 를 따로 보내지만 그건 로봇 경로를
  // 만들라는 신호이기도 해서, 캡처가 먼저 도착해도 동작하게 해 둔다.
  if (!homography_.OdomActive(ch)) {
    const char* wire_reason = NULL;
    if (!homography_.StartOdom(ch, &wire_reason)) {
      printf("[ArucoPosePNM] 주행 캘리 시작 거부 ch=%d: %s\n", ch, homography_.FailReason());
      // wire_reason 이 세분화된 값(no_intrinsics/session_conflict)이면 그걸
      // 쓰고, 아니면(정적 앵커 수집 중 / raw 좌표 모드) 기본값으로 뭉뚱그린다
      // — 서버팀 8/13 회신 §4, wire §7 표에 이 두 사유는 대응 항목이 없다.
      ReportCentralCapture(ch, point_index, false, 0.0, 0.0, -1.0,
                           wire_reason ? wire_reason : "session_refused", request_id);
      return;
    }
    printf("[ArucoPosePNM] 주행 캘리 세션 시작 ch=%d(CH%d)\n", ch, ch + 1);
  }
  odom_session_deadline_ms_[ch] = now + ODOM_SESSION_TIMEOUT_MS;

  // 이 렌즈의 검출이 꺼져 있으면 프레임이 오지 않아 영영 판정이 안 된다.
  // 조용히 기다리지 않고 즉시 알린다.
  if (!detect_enabled_[ch]) {
    ReportCentralCapture(ch, point_index, false, 0.0, 0.0, -1.0,
                         "detect_off", request_id);
    return;
  }

  OdomPending& p = odom_pending_[ch];
  p.armed = true;
  p.t0_ms = now;
  p.point_index = point_index;
  p.wx_mm = wx_mm;
  p.wy_mm = wy_mm;
  // 복귀 지점 판정: 이미 같은 월드 좌표를 받은 적이 있으면 그게 폐합 지점이다.
  // 서버가 따로 표시해 주지 않으므로 라벨 일치로 알아낸다(계획서 §3 의 idx 8 은
  // idx 0 과 좌표가 같다).
  p.closing = homography_.OdomHasLabel(ch, wx_mm, wy_mm);
  snprintf(p.request_id, sizeof(p.request_id), "%s", request_id ? request_id : "");
  p.n = 0;

  printf("[ArucoPosePNM] CALIB_CAPTURE ch=%d(CH%d) idx=%d world=(%.1f,%.1f)mm%s\n",
         ch, ch + 1, point_index, wx_mm, wy_mm, p.closing ? " [복귀·폐합용]" : "");
}

/**
 * 프레임마다 호출 — 무장된 지점이 있으면 표본을 모으고 정지 판정을 한다.
 *
 * 판정이 끝나면(성공이든 타임아웃이든) 무장을 풀고 서버에 결과를 보낸다.
 * 서버는 그 ack 를 받아야 다음 지점으로 GO 를 보낸다.
 */
void SampleComponent::OdomFeedFrame(int ch, const std::vector<ArucoProcessor::Detection>& dets,
                                    long t_capture, long now_ms) {
  OdomPending& p = odom_pending_[ch];
  if (!p.armed) return;

  // 요청보다 먼저 촬영된 프레임은 버린다. 이동 중 화면일 수 있고, 섞이면
  // 평균이 조용히 오염된다. 파이프라인 지연이 클수록 이 필터가 중요해진다.
  if (t_capture > 0 && t_capture < p.t0_ms) return;

  if (now_ms - p.t0_ms > ODOM_CAPTURE_TIMEOUT_MS) {
    p.armed = false;
    ReportCentralCapture(ch, p.point_index, false, 0.0, 0.0, -1.0,
                         p.n > 0 ? "not_settled" : "marker_not_found", p.request_id);
    return;
  }

  // 추적 대상은 중앙 링크가 쓰는 그 마커다(POS 와 같은 id). 중심 정의도
  // MarkerPoints() 로 통일한다 — 정의가 어긋나면 캘리 전체가 그 차이만큼 편향된다.
  const ArucoProcessor::Detection* target = NULL;
  for (size_t i = 0; i < dets.size(); ++i) {
    if (dets[i].id == g_central_marker_id && dets[i].corners2d.size() >= 4) {
      target = &dets[i];
      break;
    }
  }
  if (target == NULL) { p.n = 0; return; }   // 놓치면 처음부터 다시 센다

  float cx, cy;
  MarkerPoints(*target, &cx, &cy, NULL, NULL);
  // 창 크기는 런타임 값이지만 버퍼는 컴파일 상수다. 명령 처리에서 이미 상한을
  // 걸었어도 여기서 한 번 더 조인다 — 이 배열을 넘기면 스택을 밟는다.
  const int win = (g_odom_settle_frames < 2) ? 2
                : (g_odom_settle_frames > ODOM_SETTLE_FRAMES) ? ODOM_SETTLE_FRAMES
                : g_odom_settle_frames;
  if (p.n < win) {
    p.us[p.n] = cx;
    p.vs[p.n] = cy;
    ++p.n;
  } else {
    // 창이 줄어든 직후엔 p.n 이 win 보다 클 수 있다. 그때도 최근 win 개만
    // 남도록 앞을 버린다.
    const int drop = p.n - win + 1;
    for (int i = drop; i < p.n; ++i) { p.us[i - drop] = p.us[i]; p.vs[i - drop] = p.vs[i]; }
    p.n = win - 1;
    p.us[p.n] = cx;
    p.vs[p.n] = cy;
    ++p.n;
  }
  if (p.n < win) return;

  double mu = 0.0, mv = 0.0;
  for (int i = 0; i < win; ++i) { mu += p.us[i]; mv += p.vs[i]; }
  mu /= win; mv /= win;
  double var = 0.0;
  for (int i = 0; i < win; ++i) {
    const double du = p.us[i] - mu, dv = p.vs[i] - mv;
    var += du * du + dv * dv;
  }
  const double spread = std::sqrt(var / win);
  if (spread > g_odom_settle_spread_px) return;   // 아직 움직인다 — 계속 모은다

  // 정지 확인. 피팅 공간으로 옮겨서 저장한다 — 저장 시점에 한 번만 하면
  // 피팅 때 다시 할 필요가 없고, 여기서 실패하면 그 사유를 바로 보고할 수 있다.
  cv::Point2f prepared;
  const char* why = "";
  if (!homography_.PreparePixel(ch, (float)mu, (float)mv, &prepared, &why)) {
    p.armed = false;
    ReportCentralCapture(ch, p.point_index, false, mu, mv, spread, "unmappable", p.request_id);
    printf("[ArucoPosePNM] 주행 캡처 unmappable ch=%d idx=%d: %s\n", ch, p.point_index, why);
    return;
  }

  p.armed = false;
  if (!homography_.AddOdomPoint(ch, p.point_index, p.wx_mm, p.wy_mm,
                                prepared.x, prepared.y, p.closing)) {
    ReportCentralCapture(ch, p.point_index, false, mu, mv, spread, "store_failed", p.request_id);
    return;
  }
  ReportCentralCapture(ch, p.point_index, true, mu, mv, spread, "", p.request_id);
  printf("[ArucoPosePNM] 주행 캡처 ch=%d idx=%d px=(%.1f,%.1f) spread=%.2fpx%s\n",
         ch, p.point_index, mu, mv, spread, p.closing ? " [복귀]" : "");
}

/**
 * 캡처·세션 타임아웃의 벽시계 안전망 (헤더 주석 참고).
 *
 * OdomFeedFrame() 의 검사가 정상 경로이고 이건 그물이다. 판정에 쓸 표본이 없어
 * 픽셀은 실을 게 없으므로 (0,0)/spread=-1 로 보낸다 — 서버는 실패 ack 에서
 * pixel_uv 를 읽지 않는다(router_odocalib.cpp 의 ok 분기에서만 읽는다).
 *
 * 이 함수는 **어느 채널이든** 프레임/이벤트가 오면 도는 자리에서 불린다. 문제의
 * 렌즈가 굶어도 다른 렌즈가 살아 있으면 ack 는 나간다. 4채널이 전부 멎으면
 * 그건 캘리 이전에 앱이 죽은 것이라 여기서 다룰 문제가 아니다.
 */
void SampleComponent::OdomSweepTimeouts() {
  const long now = epoch_ms();
  for (int c = 0; c < kMaxChannels; ++c) {
    OdomPending& p = odom_pending_[c];
    if (p.armed && now - p.t0_ms > ODOM_CAPTURE_TIMEOUT_MS) {
      p.armed = false;
      // 표본이 하나도 없었으면 마커를 아예 못 본 것이고, 모으다 말았으면
      // 안 멈춘 것이다 — 프레임 경로와 같은 구분을 쓴다.
      const char* why = p.n > 0 ? "not_settled" : "marker_not_found";
      ReportCentralCapture(c, p.point_index, false, 0.0, 0.0, -1.0, why, p.request_id);
      printf("[ArucoPosePNM] 주행 캡처 타임아웃(프레임 없음) ch=%d idx=%d — %s\n",
             c, p.point_index, why);
      fflush(stdout);
    }
    if (odom_session_deadline_ms_[c] > 0 && now > odom_session_deadline_ms_[c]) {
      printf("[ArucoPosePNM] 주행 캘리 세션 시간 초과 ch=%d — 폐기\n", c);
      fflush(stdout);
      homography_.AbortOdom(c);
      p.armed = false;
      odom_session_deadline_ms_[c] = 0;
    }
  }
}

/**
 * 수집 종료 — 여기서 피팅하고 H_MATRIX 를 서버로 보내게 된다.
 *
 * m/n 은 이번 주행 사각형의 크기이고, 번들의 canvas_mm 을 채우는 데 쓴다.
 * 받은 지점들의 바운딩 박스로 역산할 수도 있지만 서버가 명시해 주기로 했다
 * (CCTV 2차 회신 §3 -> 서버 회신 §1-5).
 */
void SampleComponent::CentralCalibDone(int ch, double m_mm, double n_mm,
                                       const char* request_id) {
  odom_pending_[ch].armed = false;
  odom_session_deadline_ms_[ch] = 0;

  const int n = homography_.OdomCount(ch);
  printf("[ArucoPosePNM] CALIB_DONE ch=%d(CH%d) m=%.1f n=%.1f mm 지점=%d\n",
         ch, ch + 1, m_mm, n_mm, n);

  // 하한을 여기서 한 번 더 본다. 피팅 자체는 4점이면 되지만 그러면 LOO 가 안
  // 돌아 품질 지표가 사라진다 — "성공했는지 알 수 없는 H" 가 나가는 것이
  // 이 방식에서 가장 피해야 할 결과다.
  if (n < ODOM_MIN_POINTS) {
    char why[96];
    snprintf(why, sizeof(why), "유효 지점 %d개 (최소 %d)", n, ODOM_MIN_POINTS);
    ReportOdomResult(ch, false, why, request_id);
    // 서버로도 종결 응답을 보낸다 (2026-08-13, 서버팀 회신 §1). ADMIN 개시
    // 세션은 CALIB_DONE 시점에 이미 닫혀 있어 이게 없어도 무해했지만, Qt가
    // 개시한 세션은 calib_odo_result_wait_ms(서버 60초) 동안 H_MATRIX 나
    // CALIB_FAIL 을 기다린다 — 안 보내면 서버가 timeout 으로 접어 조작자에게
    // "카메라 응답 없음"으로 잘못 뜬다(실제 원인은 유효점 부족).
    ReportCentralCalibFail(ch, "too_few_points", request_id);
    homography_.AbortOdom(ch);
    return;
  }
  if (!homography_.FinishOdom(ch)) {
    ReportOdomResult(ch, false, homography_.FailReason(), request_id);
    ReportCentralCalibFail(ch, "fit_failed", request_id);
    return;
  }

  odom_m_mm_[ch] = m_mm;
  odom_n_mm_[ch] = n_mm;
  ReportOdomResult(ch, true, "", request_id);
  SendCalibBundle(ch, request_id);
}

/**
 * CALIB_CANCEL — 수집을 멈추고 CALIB_STOPPED 로 답한다 (Wire 스펙 §9).
 *
 * 로봇은 이 시점에 이미 서 있다. 서버가 ABORT_DRAW 를 먼저 보내고(로봇이
 * 동기 처리, ack 없음) 이쪽 ack 하나만 기다리는 구조라, 여기서 답하지 않으면
 * 세션이 5초 뒤 cancel_failed 로 닫힌다 — 실제로는 카메라도 멈췄는데 기록만
 * 실패로 남는다.
 *
 * 접을 세션이 없어도 답한다. 늦게 도착한 취소(이미 CALIB_DONE 으로 끝난 세션,
 * 또는 카메라 쪽 10분 데드라인이 먼저 걸린 세션)가 정확히 이 경우인데, 그건
 * 오류가 아니라 정상적인 경합이다. 그 사실은 침묵이 아니라 payload 의
 * `aborted` 개수로 전한다 — 0 이면 "받았고, 접을 게 없었다"는 뜻이다.
 *
 * 측정된 H_marker 슬롯은 건드리지 않는다. 취소는 **이번 수집**을 접는 것이고,
 * 지난번 주행으로 얻어 둔 행렬까지 버릴 이유가 없다(AbortOdom 도 같은 선을
 * 긋는다).
 */
void SampleComponent::CentralCalibCancel(int ch, const char* request_id) {
  const int lo = (ch < 0) ? 0 : ch;
  const int hi = (ch < 0) ? kMaxChannels - 1 : ch;

  int aborted = 0;
  int first = -1;
  for (int c = lo; c <= hi; ++c) {
    if (!ValidCh(c)) continue;
    // 정적 앵커 수집은 여기서 못 멈춘다 — HomographyMapper 에 취소가 없고,
    // 그건 의도된 설계다(homography_mapper.h kFitMaxFrames 주석: 취소가
    // 필요한 상황의 답은 "가린 마커를 치우는 것"이다). 그 세션은 200프레임
    // 한도로 스스로 끝나고, 끝나도 중앙 서버로는 아무것도 나가지 않는다.
    // 조용히 넘기지 않고 로그로 남긴다.
    if (homography_.FitCollecting(c)) {
      printf("[ArucoPosePNM] CALIB_CANCEL ch=%d — 정적 앵커 수집은 계속됩니다"
             " (취소 명령 없음, 프레임 한도로 자동 종료)\n", c);
    }
    const bool busy = homography_.OdomActive(c) || odom_pending_[c].armed ||
                      odom_session_deadline_ms_[c] > 0;
    if (!busy) continue;
    homography_.AbortOdom(c);
    odom_pending_[c].armed = false;
    odom_session_deadline_ms_[c] = 0;
    ++aborted;
    if (first < 0) first = c;
  }

  // ch 는 와이어로 나갈 때 1-based. 실제로 접은 렌즈를 싣되, 접을 게 없었으면
  // 취소가 지목한 렌즈를 그대로 되돌려 준다("무엇에 대한 ack 인가"가 서버
  // 로그에서 읽혀야 한다). 둘 다 없는 경우(전체 취소인데 돌던 게 없음)만 0 이다.
  const int ack_ch = (first >= 0) ? first + 1 : (ch >= 0 ? ch + 1 : 0);
  char json[192];
  snprintf(json, sizeof(json),
           "{\"ch\":%d,\"request_id\":\"%s\",\"aborted\":%d}",
           ack_ch, request_id ? request_id : "", aborted);
  const int rc = central_tls_sender_send_typed("CALIB_STOPPED", json);

  printf("[ArucoPosePNM] CALIB_CANCEL 처리 — 세션 %d개 중단, CALIB_STOPPED %s\n",
         aborted, rc == 0 ? "전송" : "전송 실패(링크 다운)");
  fflush(stdout);
}

/**
 * 세션 수준 실패를 서버로. 헤더 주석 참고 — Qt 개시 세션의 종결 응답이다.
 *
 * reason 은 이 파일이 만든 고정 문자열만 들어온다("too_few_points",
 * "fit_failed") — ReportCentralCapture() 와 같은 이유로 이스케이프가
 * 필요 없다.
 */
void SampleComponent::ReportCentralCalibFail(int ch, const char* reason,
                                             const char* request_id) const {
  char json[192];
  snprintf(json, sizeof(json),
           "{\"ch\":%d,\"request_id\":\"%s\",\"reason\":\"%s\"}",
           ch + 1, request_id ? request_id : "", reason ? reason : "");
  const int rc = central_tls_sender_send_typed("CALIB_FAIL", json);
  printf("[ArucoPosePNM] CALIB_FAIL ch=%d(CH%d) reason=%s %s\n",
         ch, ch + 1, reason ? reason : "",
         rc == 0 ? "전송" : "전송 실패(링크 다운)");
  fflush(stdout);
}

/**
 * 주행 캘리 결과를 대시보드(포즈 링크)로. 서버로 가는 것은 H_MATRIX 다.
 *
 * 폐합오차를 같이 싣는다: 복귀 지점은 피팅에서 제외됐으므로, 그 픽셀을 새 H 로
 * 월드에 보내 출발점 라벨과 비교한 값은 **out-of-sample 검증**이다. LOO 와 같은
 * 성격이면서 "한 바퀴 도는 동안 오도메트리가 얼마나 밀렸는가"를 mm 로 준다.
 *
 * 다만 이 값도 균일 스케일 오차는 못 잡는다 — 네 변이 같은 비율로 짧으면 실제
 * 궤적도 닫힌 직사각형이라 정확히 제자리로 돌아온다. 그건 compare 쪽만 드러낸다.
 */
void SampleComponent::ReportOdomResult(int ch, bool ok, const char* reason,
                                       const char* request_id) const {
  const HomographyMapper::FitResult& fr = homography_.OdomResiduals(ch);
  double closure = -1.0;
  homography_.OdomClosureMm(ch, &closure);
  double scale = 0.0, cmp_rmse = -1.0, cmp_max = -1.0;
  const char* cmp_why = "";
  const bool cmp_ok = homography_.CompareMarkerPlanes(ch, &scale, &cmp_rmse, &cmp_max, &cmp_why);

  char json[512];
  snprintf(json, sizeof(json),
           "{\"type\":\"ODOM_DONE\",\"ch\":%d,\"ok\":%s,\"reason\":\"%s\","
           "\"n\":%d,\"loo_valid\":%s,\"rmse_in_mm\":%.1f,\"rmse_loo_mm\":%.1f,"
           "\"max_loo_mm\":%.1f,\"max_loo_id\":%d,\"closure_mm\":%.1f,"
           "\"cmp_ok\":%s,\"cmp_scale\":%.5f,\"cmp_rmse_mm\":%.1f,\"cmp_max_mm\":%.1f,"
           "\"cmp_reason\":\"%s\",\"request_id\":\"%s\"}",
           ch, ok ? "true" : "false", reason ? reason : "",
           fr.n, fr.loo_valid ? "true" : "false", fr.rmse_in_mm, fr.rmse_loo_mm,
           fr.max_loo_mm, fr.max_loo_id, closure,
           cmp_ok ? "true" : "false", scale, cmp_rmse, cmp_max,
           cmp_ok ? "" : (cmp_why ? cmp_why : ""),
           request_id ? request_id : "");
  pose_sender_send_control_line(json);

  printf("[ArucoPosePNM] 주행 캘리 ch=%d %s n=%d loo=%.1fmm 폐합=%.1fmm",
         ch, ok ? "성공" : "실패", fr.n, fr.rmse_loo_mm, closure);
  if (cmp_ok) printf(" 체커보드대비 scale=%.4f rmse=%.1fmm", scale, cmp_rmse);
  printf("\n");
  fflush(stdout);
}

/**
 * 두 캘리 방식이 공유하는 번들 조립·전송.
 *
 * 지금까지 H_MATRIX 는 대시보드가 만든 payload 를 카메라가 통과만 시켰다
 * (CENTRAL_HMATRIX). 그 자리 주석이 "이 앱은 K/D/H_marker 를 다 갖고 있어서
 * 직접 조립할 수 있지만 mm<->m 규약 정리가 안 끝나 통과 방식으로 뒀다,
 * auto-assembly 는 follow-up" 이라고 적어 둔 그 follow-up 이다. 주행 캘리가
 * 먼저 넘어왔고(2026-08-13), 정적 앵커도 같은 자리로 옮겼다(2026-08-14).
 *
 * 단위 규약: H 는 둘 다 mm 유지(pixel -> world mm). 높이도 mm 다 —
 * marker_height_mm 이 유일하게 m 로 나가던 잔재는 8/13 에 정리했다.
 */
bool SampleComponent::EmitCalibBundle(int ch, const char* request_id,
                                      const char* method, const char* calib_id,
                                      bool undistorted,
                                      const double Hm[9], const double Hf[9],
                                      double canvas_w_mm, double canvas_h_mm,
                                      const char* extra_json,
                                      const char** why) const {
  if (why) *why = "";
  // K/D 가 없으면 보내지 않는다. 서버의 왜곡 보정은 K/D 존재 여부로만 켜지므로
  // (Calib::hasKD), 빠진 번들이 가면 서버가 보정을 조용히 건너뛰고 H 를 raw
  // 픽셀에 적용한다 — 에러 없이 좌표만 틀어진다. 서버팀 회신 §3-2 의 요청이고,
  // 이 프로젝트에서 실제로 한 번 겪은 실패다(대시보드 조립 시절, 90건).
  double fx, fy, cx, cy, dist[5];
  if (!calib_.Get(ch, &fx, &fy, &cx, &cy, dist)) {
    printf("[ArucoPosePNM] 번들 조립 거부 ch=%d: K/dist 없음 (서버가 왜곡 보정을 건너뛴다)\n", ch);
    if (why) *why = "K/dist 가 없습니다 — 서버가 왜곡 보정을 건너뛰므로 보내지 않습니다";
    return false;
  }
  // image_size 도 없으면 보내지 않는다 (2026-08-13, 서버팀 회신 §5). 예전에는
  // 미수신 시 [2592,1520] 로 채워 보냈는데, 이건 실제 해상도가 다른데도 그럴듯한
  // 값을 자신 있게 실어 보내는 경로였다 — K/D 90건 누락이 아무도 모르게 쌓였던
  // 것과 같은 실패 양상. image_size 가 틀리면 Qt 디코딩 해상도와의 비율만큼
  // 좌표가 통째로 어긋나는데 값 자체는 그럴듯해 아무도 눈치채지 못한다. K/D와
  // 같은 원칙으로 통일한다: 없으면 폴백하지 말고 거부한다.
  if (last_w_[ch] <= 0 || last_h_[ch] <= 0) {
    printf("[ArucoPosePNM] 번들 조립 거부 ch=%d: image_size 미수신 (프레임 크기를 아직 모름)\n", ch);
    if (why) *why = "이 렌즈의 프레임 크기를 아직 모릅니다 (검출이 꺼져 있거나 첫 프레임 전)";
    return false;
  }

  // created_at 은 규격 필드다(서버 calib.hpp 의 번들 주석 목록). 파싱 게이트는
  // hasKD 뿐이라 없어도 저장은 되지만, 저장된 번들만 보고 "언제 잰 것인가"를
  // 알 수 있어야 두 방식(체커보드/주행)의 값이 갈릴 때 어느 쪽이 최신인지
  // 가릴 수 있다. UTC 로 낸다 — 카메라와 서버의 로컬 시간대 설정이 다르다.
  const time_t now_s = (time_t)(epoch_ms() / 1000);
  struct tm utc;
  char created[32] = "";
  if (gmtime_r(&now_s, &utc) != NULL)
    strftime(created, sizeof(created), "%Y-%m-%dT%H:%M:%SZ", &utc);

  char json[CENTRAL_TLS_MAX_LINE];
  const int n = snprintf(json, sizeof(json),
      "{\"ch\":%d,\"request_id\":\"%s\",\"calib\":{"
      "\"calib_id\":\"%s\",\"created_at\":\"%s\",\"method\":\"%s\",\"unit\":\"mm\","
      "\"coord_mode\":\"%s\",\"image_size\":[%d,%d],"
      "\"K\":[[%.4f,0,%.4f],[0,%.4f,%.4f],[0,0,1]],"
      "\"D\":[%.8f,%.8f,%.8f,%.8f,%.8f],"
      "\"marker_height_mm\":%.1f,\"origin_mm\":[0,0],\"canvas_mm\":[%.0f,%.0f],"
      "\"axis\":\"x_right_y_up\","
      "\"H_marker\":[[%.9e,%.9e,%.9e],[%.9e,%.9e,%.9e],[%.9e,%.9e,%.9e]],"
      "\"H_floor\":[[%.9e,%.9e,%.9e],[%.9e,%.9e,%.9e],[%.9e,%.9e,%.9e]]%s}}",
      ch + 1, request_id ? request_id : "", calib_id, created, method,
      undistorted ? "undistort" : "raw",
      last_w_[ch], last_h_[ch],
      fx, cx, fy, cy,
      dist[0], dist[1], dist[2], dist[3], dist[4],
      homography_.MarkerHeightMm(), canvas_w_mm, canvas_h_mm,
      Hm[0], Hm[1], Hm[2], Hm[3], Hm[4], Hm[5], Hm[6], Hm[7], Hm[8],
      Hf[0], Hf[1], Hf[2], Hf[3], Hf[4], Hf[5], Hf[6], Hf[7], Hf[8],
      extra_json ? extra_json : "");
  if (n <= 0 || n >= (int)sizeof(json)) {
    printf("[ArucoPosePNM] 번들이 한 줄 상한(%d B)을 넘었습니다 — 전송 안 함\n",
           CENTRAL_TLS_MAX_LINE);
    if (why) *why = "번들이 한 줄 상한을 넘었습니다";
    return false;
  }
  const int rc = central_tls_sender_send_typed("H_MATRIX", json);
  printf("[ArucoPosePNM] H_MATRIX ch=%d %s (%s, %d B, coord_mode=%s)\n",
         ch + 1, rc == 0 ? "전송" : "전송 실패(링크 다운)", method, n,
         undistorted ? "undistort" : "raw");
  fflush(stdout);
  if (rc != 0 && why) *why = "중앙 서버 링크가 끊겨 있습니다";
  return rc == 0;
}

/**
 * 주행 캘리 번들. 측정한 것은 H_marker 이고 H_floor 를 역산한다.
 *
 * 사람이 개입할 틈 없이 카메라가 바로 보내야 하는 경로다 — 서버가 로봇을 몰고
 * 있는 동안 대시보드는 열려 있지 않을 수도 있다.
 */
bool SampleComponent::SendCalibBundle(int ch, const char* request_id,
                                      const char** why) const {
  if (why) *why = "";
  double Hm[9];
  if (!homography_.GetMeasuredMarkerPlane(ch, Hm)) {
    printf("[ArucoPosePNM] 번들 조립 실패 ch=%d: 측정된 H_marker 가 없습니다\n", ch);
    if (why) *why = "측정된 H_marker 가 없습니다 — 주행 캘리를 먼저 완주하세요";
    return false;
  }

  // H_floor 역산. 두 평면은 나디르를 중심으로 한 균일 스케일 관계다:
  //   marker = nadir + (1 - h/Zc) * (floor - nadir)
  // 이므로 역방향은 나디르 기준으로 1/(1 - h/Zc) 만큼 확대하는 것이다.
  // 이걸 안 하고 H_floor 자리에 H_marker 를 그대로 넣으면 서버 pose 는 멀쩡한데
  // Qt top-view 만 조용히 틀어진다(서버는 H_floor 가 없으면 H_marker 를 복사한다).
  //
  // 나디르와 카메라 높이는 **측정 H_marker 를 직접 분해해서** 얻는다
  // (MeasuredCameraPose). floor 탭의 CameraPose 를 쓰지 않는 이유가 둘이다:
  //   - 그건 바닥 H 가 있어야만 나오므로 주행 캘리 단독으로는 늘 실패한다.
  //   - 더 중요하게, 바닥 H 의 월드 프레임은 주행 캘리의 프레임과 원점이 다르다
  //     (주행 원점 = 그 세션 로봇 출발점). 남의 프레임 나디르로 확대하면
  //     그럴듯하고 조용히 틀린 H_floor 가 나간다.
  // 분해로 나오는 높이는 마커평면 위 높이(zc_m)이므로 바닥 위 높이는 zc_m + h.
  //   s = 1/(1 - h/(zc_m + h)) = 1 + h/zc_m
  double Hf[9];
  memcpy(Hf, Hm, sizeof(Hf));
  bool floor_ok = false;
  double zc_m = 0.0, nx = 0.0, ny = 0.0;
  const char* pose_why = "";
  const double h_mm = homography_.MarkerHeightMm();
  if (h_mm <= 0.0) {
    // 마커가 바닥에 붙어 있으면 두 평면이 같다 — 보정할 것이 없고, 그건 실패가
    // 아니다. H_marker 를 그대로 H_floor 로 싣는 것이 맞는 답이다.
    floor_ok = true;
  } else if (homography_.MeasuredCameraPose(ch, &zc_m, &nx, &ny, &pose_why) && zc_m > 0.0) {
    const double s = 1.0 + h_mm / zc_m;           // 확대 계수
    // 나디르 중심 스케일 S 를 월드 쪽에 곱한다: H_floor = S * H_marker
    //   S(p) = nadir + s * (p - nadir)
    for (int i = 0; i < 3; ++i) {
      Hf[0 * 3 + i] = s * Hm[0 * 3 + i] + (1.0 - s) * nx * Hm[2 * 3 + i];
      Hf[1 * 3 + i] = s * Hm[1 * 3 + i] + (1.0 - s) * ny * Hm[2 * 3 + i];
    }
    printf("[ArucoPosePNM] H_floor 역산 ch=%d: 카메라 마커평면 위 %.0fmm "
           "(바닥 위 %.0fmm), 나디르=(%.0f,%.0f)mm, 확대 %.4f\n",
           ch, zc_m, zc_m + h_mm, nx, ny, s);
    floor_ok = true;
  } else {
    // 역산에 실패하면 H_floor 를 싣지 않는 편이 낫지만, 서버는 없으면
    // H_marker 를 복사하므로 결과가 같다. 최소한 로그로는 남긴다.
    printf("[ArucoPosePNM] H_floor 역산 불가 ch=%d (%s) — 번들에 H_marker 를 그대로 싣는다\n",
           ch, pose_why && pose_why[0] ? pose_why : "측정 H_marker 분해 실패");
  }

  // 세 경우를 구분해서 남긴다 — "복사"가 마커 높이 0(정상)인지 분해 실패(문제)인지
  // 로그만 보고 갈릴 수 있어야 한다.
  printf("[ArucoPosePNM] 주행 번들 ch=%d H_floor %s\n", ch + 1,
         (h_mm <= 0.0) ? "H_marker 와 동일 (마커 높이 0)"
         : floor_ok    ? "역산"
                       : "H_marker 복사 (역산 실패)");

  // 폐합오차 + 체커보드 대비 스케일 (2026-08-13, 서버팀 회신 §4). 지금까지는
  // ODOM_DONE 으로 대시보드에만 실어 서버 로그·Qt 어디에도 안 남았다. 서버는
  // H_MATRIX 번들을 값 변경 없이 raw 로 저장·중계하므로, 필드로 얹기만 하면
  // 새 메시지 없이 그대로 도착한다 — ReportOdomResult() 와 같은 계산을 그대로
  // 재사용한다(값이 갈릴 이유가 없다).
  //
  // 둘 다 균일 스케일 오차는 못 잡는다는 한계가 있다: 네 변이 같은 비율로
  // 짧으면 궤적도 정확히 닫히므로 closure_mm 은 그걸 못 보고, closure_mm 은
  // out-of-sample 검증일 뿐 체커보드라는 별도 기준과 비교하는 게 아니다.
  // cmp_scale 만 그 오차(예: 로봇 바퀴 지름)를 드러낸다 — 1.000 에서 벗어나면
  // 유일한 신호다.
  double closure_mm = -1.0;
  homography_.OdomClosureMm(ch, &closure_mm);
  double cmp_scale = 0.0, cmp_rmse_mm = -1.0, cmp_max_mm = -1.0;
  const char* cmp_why = "";
  const bool cmp_ok = homography_.CompareMarkerPlanes(ch, &cmp_scale, &cmp_rmse_mm,
                                                       &cmp_max_mm, &cmp_why);
  if (!cmp_ok) cmp_scale = 0.0;  // 0 = "비교 불가" (체커보드 H 없음 등)

  char extra[96];
  snprintf(extra, sizeof(extra), ",\"closure_mm\":%.1f,\"cmp_scale\":%.5f",
           closure_mm, cmp_scale);
  char calib_id[48];
  snprintf(calib_id, sizeof(calib_id), "odom-%ld", epoch_ms());

  // coord_mode 는 참으로 고정한다. StartOdom() 이 K/dist 없는 렌즈의 세션을
  // 아예 거부하므로(no_intrinsics), 측정 H_marker 는 정의상 undistort 공간이다.
  return EmitCalibBundle(ch, request_id, ODOM_METHOD_NAME, calib_id, true,
                         Hm, Hf, odom_m_mm_[ch], odom_n_mm_[ch], extra, why);
}

/**
 * 정적 앵커 캘리 번들. 주행 캘리와 방향만 반대다 — 측정한 것이 H_floor 이고
 * H_marker 를 마커 높이만큼 파생시킨다(RefreshMarkerPlane 의 3D 평면 이동).
 *
 * coord_mode 를 FittedUndistorted(ch) 에서 그대로 읽는 것이 이 경로를 카메라로
 * 옮긴 가장 큰 이유다. 대시보드는 이 값을 알 방법이 없어 "HG_COORD_MODE 를 보낸
 * 이력"으로 유추했는데, 그건 설정이지 사실이 아니다 — 새로고침만 해도 이력이
 * 날아가고, 2026-08-11 에는 raw 로 피팅된 CH2 의 H 가 undistort 라고 라벨링돼
 * 몇 시간 동안 나가 있었다. Qt 는 이 라벨을 보고 왜곡 보정을 켜므로 이미 보정된
 * 픽셀을 두 번 보정한다 — 에러 없이 그림만 틀어진다.
 */
bool SampleComponent::SendFloorBundle(int ch, const char* request_id,
                                      const char** why) const {
  if (why) *why = "";
  double Hf[9];
  if (!homography_.Get(ch, Hf)) {
    printf("[ArucoPosePNM] 번들 조립 실패 ch=%d: 바닥 H 가 없습니다\n", ch + 1);
    if (why) *why = "이 렌즈의 바닥 H 가 없습니다 — 앵커 캘리를 먼저 완주하세요";
    return false;
  }
  // 파생 H_marker. 준비가 안 됐으면(마커 높이 0, K 없음 등) H_floor 를 그대로
  // 싣는다 — 주행 경로가 역산 실패 시 H_marker 를 복사하는 것과 같은 처리이고,
  // 서버도 H_marker 가 없으면 floor 를 복사하므로 결과가 같다.
  double Hm[9];
  const bool marker_ok = homography_.GetMarkerPlane(ch, Hm);
  if (!marker_ok) {
    memcpy(Hm, Hf, sizeof(Hm));
    printf("[ArucoPosePNM] 앵커 번들 ch=%d H_marker 파생 불가 (%s) — H_floor 를 그대로 싣는다\n",
           ch + 1, homography_.MarkerPlaneReason(ch));
  }

  // canvas_mm 은 앵커들이 실제로 덮는 사각형이다. 대시보드의 guessCanvasMm()
  // 이 하던 추정과 같은 계산인데, 여기서는 추정이 아니라 카메라가 등록해 둔
  // 앵커표 원본을 그대로 읽는다. 앵커가 2개 미만이면 0 으로 두고(서버는 이 값을
  // 그리기 영역 힌트로만 쓴다) 틀린 크기를 자신 있게 보내지 않는다.
  double canvas_w = 0.0, canvas_h = 0.0;
  const int na = homography_.AnchorCount(ch);
  if (na >= 2) {
    double x0 = 0, x1 = 0, y0 = 0, y1 = 0;
    for (int i = 0; i < na; ++i) {
      Anchor a;
      if (!homography_.AnchorAt(ch, i, &a)) break;
      if (i == 0) { x0 = x1 = a.wx_mm; y0 = y1 = a.wy_mm; continue; }
      if (a.wx_mm < x0) x0 = a.wx_mm;
      if (a.wx_mm > x1) x1 = a.wx_mm;
      if (a.wy_mm < y0) y0 = a.wy_mm;
      if (a.wy_mm > y1) y1 = a.wy_mm;
    }
    canvas_w = x1 - x0;
    canvas_h = y1 - y0;
  }

  // 품질 지표. 주행 캘리의 closure_mm/cmp_scale 과 같은 자리다 — 지금까지 이
  // 숫자들은 HG_FIT 으로 대시보드에만 갔고 서버·Qt 어디에도 안 남았다. 저장된
  // 번들만 보고 "이 캘리를 믿어도 되는가"를 판단할 수 있어야 두 방식 중 어느
  // 쪽을 택할지 서버가 근거를 갖는다.
  const HomographyMapper::FitResult& fr = homography_.Residuals(ch);
  char extra[160];
  if (fr.have)
    snprintf(extra, sizeof(extra),
             ",\"n_points\":%d,\"rmse_in_mm\":%.1f,\"rmse_loo_mm\":%.1f,"
             "\"max_loo_mm\":%.1f,\"loo_valid\":%s",
             fr.n, fr.rmse_in_mm, fr.rmse_loo_mm, fr.max_loo_mm,
             fr.loo_valid ? "true" : "false");
  else
    extra[0] = '\0';  // HG_SET 으로 주입된 H 에는 잔차가 없다

  char calib_id[48];
  snprintf(calib_id, sizeof(calib_id), "floor-%ld", epoch_ms());

  return EmitCalibBundle(ch, request_id, FLOOR_METHOD_NAME, calib_id,
                         homography_.FittedUndistorted(ch),
                         Hm, Hf, canvas_w, canvas_h, extra, why);
}

void SampleComponent::ReportCentralCapture(int ch, int point_index, bool ok,
                                           double u, double v, double spread_px,
                                           const char* reason,
                                           const char* request_id) const {
  char json[320];
  // ch 는 와이어로 나갈 때 다시 1-based (docs/PROTOCOL.md "채널 규약").
  // reason 은 이 파일이 만든 고정 문자열만 들어온다 — 네트워크에서 온 문자열을
  // 그대로 되돌려 보내지 않는다(JSON 이스케이프가 필요해진다).
  snprintf(json, sizeof(json),
           "{\"ch\":%d,\"point_index\":%d,\"ok\":%s,"
           "\"pixel_uv\":[%.2f,%.2f],\"spread_px\":%.2f,"
           "\"reason\":\"%s\",\"request_id\":\"%s\"}",
           ch + 1, point_index, ok ? "true" : "false", u, v, spread_px,
           reason ? reason : "", request_id ? request_id : "");
  central_tls_sender_send_typed(ok ? "CALIB_CAPTURE_OK" : "CALIB_CAPTURE_FAIL", json);
}
#endif  // ENABLE_CENTRAL_TLS_STREAM

void SampleComponent::PollDashboardCommands() {
  // One budget for every transport — see POSE_SENDER_MAX_LINE. Lines that still
  // do not fit are dropped with a message by pose_sender_poll_command() rather
  // than trimmed; see there.
  char cmd[POSE_SENDER_MAX_LINE];
  while (pose_sender_poll_command(cmd, sizeof(cmd))) {
    if (DispatchCommand(cmd)) continue;
    printf("[ArucoPosePNM] unknown command: %s\n", cmd);
    fflush(stdout);
  }
}

extern "C" {
SampleComponent* create_component(void* mem_manager) {
  Component::allocator = decltype(Component::allocator)(mem_manager);
  Event::allocator = decltype(Event::allocator)(mem_manager);
  return new ("SampleComponent") SampleComponent();
}

void destroy_component(SampleComponent* ptr) { delete ptr; }
}
