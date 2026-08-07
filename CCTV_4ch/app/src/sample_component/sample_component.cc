#include "sample_component.h"

#include <dirent.h>
#include <limits.h>
#include <stdarg.h>
#include <math.h>     // atan2 — CAM_POSE heading
#include <stdlib.h>   // strtol/strtod — ANCHOR_SET_ALL parses by hand
#include <string.h>
#include <sys/stat.h>
#include <malloc.h>          // mallinfo2, mallopt, malloc_trim — see heap_bytes()
#include <sys/resource.h>
#include <sys/time.h>
#include <unistd.h>

#include <opencv2/core.hpp>

#include "app_config.h"
#include "dispatcher_serialize.h"
#include "i_app_dispatcher.h"
#include "i_log_manager.h"
#include "i_p_open_platform_manager.h"
#include "i_p_stream_provider_manager_video_raw.h"
#include "i_p_video_frame_raw.h"   // IPVideoFrameRaw, RawImage, RAW_FMT_*, RAW_PLANE_*
#include "i_pl_video_frame_raw.h"  // IPLVideoFrameRaw (concrete)
#include "pose_sender.h"

namespace {
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
      detect_enabled_[c] = true;
      search_scale_[c] = SEARCH_DOWNSCALE;
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
                             "CALIB_K_*|K_LOAD|HG_*|ANCHOR_*)");
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
           (c == 0) ? "" : ",", c, detect_enabled_[c] ? "true" : "false", search_scale_[c],
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
  std::shared_ptr<RawImage> img(raw_frame->GetRawImage());
  if (!img) {
    ++raw_no_image_;
    return;
  }

  const long t_frame_ms = epoch_ms();

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
      } else {
        // Throttled board probe so the page can say "board visible, N corners"
        // and draw where it is. The calibration UI has no photo, so without
        // this, aiming the board would be blind. See CALIB_PROBE_MS.
        calib_.ProbeIfDue(ch, gray, t_frame_ms);
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
    // The dashboard's entire footprint on the hot path: four stores, no
    // branches, no allocation, no clock read (t_frame_ms was taken above for
    // the pose packets). Everything else /status reports is derived from state
    // that already exists.
    last_frame_ms_[ch] = t_frame_ms;
    last_markers_[ch] = (int)dets.size();
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

#if ENABLE_STATUS_PAGE
    last_dets_[ch] = dets;
    last_dets_approx_[ch] = approximate;
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
      homography_.FeedFrame(ch, ids, cxs, cys, nd);
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

/** "DYNROI <0|1> [maxMargin] [maxMiss]" -- toggle/tune the tracker at runtime. */
bool SampleComponent::HandleDynRoi(const char* cmd) {
  const char* p = strstr(cmd, "DYNROI");
  if (p == NULL) return false;

  int on = 0, margin = dynroi_[0].margin(), maxMiss = dynroi_[0].maxMiss();
  sscanf(p + 6, "%d %d %d", &on, &margin, &maxMiss);  // 6 = strlen("DYNROI")
  if (margin < 0) margin = 0;
  if (margin > 960) margin = 960;
  if (maxMiss < 0) maxMiss = 0;
  if (maxMiss > 60) maxMiss = 60;

  for (int c = 0; c < kMaxChannels; ++c) {
    if (!dynroi_[c].configure(on != 0, margin, maxMiss) && aruco_[c])
      aruco_[c]->setRoi(manual_roi_[c]);  // tracker off -> manual ROI back
  }

  ReportDynRoi();
  printf("[ArucoPosePNM] dynroi %s margin=%d maxMiss=%d (all channels)\n",
         dynroi_[0].enabled() ? "ON" : "OFF", dynroi_[0].margin(), dynroi_[0].maxMiss());
  fflush(stdout);
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
  // Ordering matters: these match on substrings, so a longer name that
  // CONTAINS a shorter one has to be tried first. DYNROI_IDS before DYNROI is
  // the existing case — otherwise "DYNROI" swallows it and parses "_IDS ..."
  // as its own numeric arguments.
  if (HandleCalibK(cmd)) return true;   // CALIB_K_* — all matched in one place
  if (HandleHomography(cmd)) return true;  // HG_* — likewise
  if (HandleAnchors(cmd)) return true;     // ANCHOR_*
  if (HandleScale(cmd)) return true;
  if (HandleDuty(cmd)) return true;
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
 *   CALIB_K_CAPTURE             — capture the active lens's next frame
 *   CALIB_K_UNDO
 *   CALIB_K_COMPUTE
 *   CALIB_K_SAVE <ch>
 *   CALIB_K_REVERT <ch>         — swap back to the value before the last save
 *   K_LOAD <ch> fx fy cx cy d0 d1 d2 d3 d4
 *
 * Everything answers through /status; there is no separate reply, on purpose —
 * the page repaints from the same object either way, and the pose link's
 * CALIB_K_* status lines can be added later without changing any of this.
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
    calib_.NoteMessage(calib_.SetBoard(c, &reason) ? "보드 설정을 적용했습니다 (RAM)" : reason);
    return true;
  }
  if (strncmp(cmd, "CALIB_K_BOARD_SAVE", 18) == 0) {
    calib_.NoteMessage(calib_.SaveBoard() ? "보드 설정 저장됨" : "보드 설정 저장 실패");
    return true;
  }
  if (strncmp(cmd, "CALIB_K_SET", 11) == 0) {
    int views = 0;
    double rms = 0.0;
    sscanf(cmd + 11, "%d %lf", &views, &rms);
    calib_.SetParams(views, rms);
    return true;
  }
  if (strncmp(cmd, "CALIB_K_GATE", 12) == 0) {
    int on = 1;
    if (sscanf(cmd + 12, "%d", &on) == 1) calib_.SetGates(on != 0);
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
      calib_.NoteMessage("이 렌즈는 호모그래피 수집 중입니다 "
                         "— 호모그래피 탭에서 끝난 뒤에 시작하세요");
      return true;
    }
    if (!calib_.Start(ch, &reason)) calib_.NoteMessage(reason);
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
    // One press arms EVERY open session, so a single board pose is banked by
    // every lens that can see it. Each lens judges it separately when its own
    // frame arrives — see IntrinsicsCalib::RequestCapture.
    if (!calib_.RequestCapture(&reason)) calib_.NoteMessage(reason);
    return true;
  }
  if (strncmp(cmd, "CALIB_K_UNDO", 12) == 0) {
    int ch = -1;
    if (sscanf(cmd + 12, "%d", &ch) != 1) return true;
    calib_.UndoLast(ch);
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
    calib_.Compute(ch);
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
    if (calib_.Save(ch)) calib_.NoteMessage("이 렌즈의 K/dist를 저장했습니다");
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