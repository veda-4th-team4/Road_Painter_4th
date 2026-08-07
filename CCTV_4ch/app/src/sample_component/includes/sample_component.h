#pragma once

#include <memory>

#include "app_config.h"  // ENABLE_STATUS_PAGE gates the members below
#include "aruco_processor.h"
#include "component.h"
#include "dyn_roi.h"
#include "homography_mapper.h"
#include "i_sample_component.h"
#include "intrinsics_calib.h"

class SampleComponent : public Component, public ISampleComponent {
 public:
  SampleComponent();
  SampleComponent(ClassID id, const char* name);
  virtual ~SampleComponent();
  bool ProcessAEvent(Event* event) override;

 protected:
  bool Initialize() override;

 private:
  bool HandleHttpRequest(Event* event);
  void RegisterURI();
  std::string GetCurrentTimeToString();

  // GET /status — everything app/html/index.html shows, as one JSON object.
  // Runs on the scheduler thread, the same one as ProcessRawVideo(), so every
  // value it reads is quiescent and none of it needs a lock.
  std::string BuildStatusJson();
  // Roll every rate the page shows (app CPU, per-core CPU, per-lens delivered
  // fps) over one shared window. Measurement, kept out of the JSON builder.
  void SampleRates(long now_ms, double cpu_s);

  // PNM-C16083RVQ is a 4-sensor camera: SPMgrVideoRaw_0 .. _3.
  static const int kMaxChannels = 4;

  // Raw-video path: NV12 Y plane -> ArUco detect -> CAM_POSE over TCP.
  void ProcessRawVideo(Event* event);
  // t_capture / queue_ms travel with every packet so the server can see the
  // frame's true age, not just this channel's detect time.
  void SendPosePackets(int ch, const std::vector<ArucoProcessor::Detection>& fresh,
                       long t_frame_ms, int frame_w, int frame_h,
                       long t_capture, long queue_ms);

  // THE command surface. Every control the app exposes is one text line, and
  // both transports go through here:
  //
  //   pose link (TCP)  ->  PollDashboardCommands() -> DispatchCommand()
  //   on-camera page   ->  POST /cmd               -> DispatchCommand()
  //
  // One language and one handler chain, deliberately: the plan is to drive all
  // of this from the RPi server as well, and a control that exists on only one
  // transport is a control the other side cannot have. Adding a command means
  // adding one Handle*() and one line in DispatchCommand() — never two parsers
  // that can drift apart.
  //
  // Returns false when nothing recognised the line.
  bool DispatchCommand(const char* cmd);

  // Commands arrive on the same TCP link the poses go out on.
  void PollDashboardCommands();
  bool HandleDynRoi(const char* cmd);
  bool HandleDynRoiIds(const char* cmd);
  bool HandleDetect(const char* cmd);
  bool HandleScale(const char* cmd);
  bool HandleDynRoiCh(const char* cmd);
  // DUTY <pct> — the detection budget, at runtime. Named DUTY and not
  // DETECT_DUTY because HandleDetect() matches "DETECT" with strstr, so any
  // name containing it is swallowed there and silently does nothing.
  bool HandleDuty(const char* cmd);
  void ReportDynRoi() const;
  void ReportDetect() const;
  void ReportDuty() const;

  // Intrinsics (K/dist) calibration. Same command vocabulary the RPi dashboard
  // already speaks (CALIB_K_*), so the page here and the server there drive it
  // with identical strings — the one difference is that START now names a lens,
  // because this camera has four.
  bool HandleCalibK(const char* cmd);
  // Homography (HG_*) — same vocabulary cctv_app and the RPi dashboard use,
  // plus the <ch> argument every one of them now needs.
  bool HandleHomography(const char* cmd);
  // HG_QUERY's answer on the pose link, mirroring ReportDynRoi(). The page
  // itself reads /status; this exists because the RPi asks over TCP.
  void ReportHomography(int ch) const;
  // ANCHOR_* — the registered marker list H gets fitted from. Separate from
  // HandleHomography() because these commands carry a variable-length payload
  // and parse by hand, where every HG_* command is one sscanf.
  bool HandleAnchors(const char* cmd);
  // ANCHOR_QUERY's answer on the pose link. Same argument as
  // ReportHomography(): the page reads /status, the RPi has no /status.
  void ReportAnchors(int ch) const;
  // MARKER_PLANE_QUERY's answer on the pose link: the derived H_marker plus the
  // camera pose the floor H implies. The pose is the diagnostic half — an
  // installer who knows the camera is 1.5 m up can see instantly whether the
  // decomposition is sane, and a nadir in the wrong place means a bad K.
  void ReportMarkerPlane(int ch) const;
  // Record an ANCHOR_* outcome into anchor_cmd_ and the log, in one place.
  void SetAnchorResult(int ch, bool ok, const char* reason);
  // Appends the calibration block to /status.
  void AppendCalibJson(void* jsonbuf, long now_ms);

  // Turn one lens's marker search on or off. Returns false if `ch` is out of
  // range. Reachable two ways on purpose — POST /detect from the on-camera
  // page, and "DETECT <ch> <0|1>" on the pose link — because the HTTP route
  // stops answering exactly when the app is too loaded, which is when turning
  // a channel off is most needed.
  bool SetDetectEnabled(int ch, bool on);
  bool SetSearchScale(int ch, int n);

 private:
  std::string setting_changed_time_;

  // Everything below is PER CHANNEL. The v5 app kept one global detector and
  // one global tracker; with four lenses that model does not hold — each lens
  // has its own field of view, so its own ROI box and its own frame counter.
  std::unique_ptr<ArucoProcessor> aruco_[kMaxChannels];
  DynRoiTracker dynroi_[kMaxChannels];
  cv::Rect manual_roi_[kMaxChannels];   // operator ROI; empty = full frame
  unsigned long seq_[kMaxChannels];     // frame counter, shared by same-frame packets

  // Per-lens master switch for the marker search. Default on.
  //
  // This is not just a convenience. All four channels share ONE scheduler
  // thread, and a lens with no marker in view cannot use the dynamic ROI, so
  // it pays a full-frame scan every frame — measured 180..300 ms at 1080p on
  // .13 (2026-08-04). Four of those saturate the thread completely: video
  // events pile up and HTTP requests wait behind them until nginx gives up
  // (502 after ~20 s). Switching off the lenses that have nothing to look at
  // is what keeps the app inside its one-thread budget.
  bool detect_enabled_[kMaxChannels];

  // K/dist for all four lenses plus the one capture session that may be open.
  // Deliberately a single object rather than four: the board description and
  // the session are shared by construction (one printed sheet, one operator),
  // and only the RESULT is per lens. See intrinsics_calib.h.
  IntrinsicsCalib calib_;

  // Pixel -> world (mm) per lens. One object holding four matrices, for the
  // same reason calib_ holds four K's: the thing that is per lens is the
  // result, and routing every access through "which lens" is what stops the
  // four from quietly becoming one. See homography_mapper.h.
  HomographyMapper homography_;

  // Result of the most recent HG_MAP, carried in /status.
  //
  // Not a control-line-only reply, for the reason every other readout here is
  // in /status: POST /cmd answers with the whole status object, so a command
  // and its result are one round trip, and the page never has to correlate an
  // asynchronous ack with the request that caused it. A reply that exists only
  // on the pose link is also invisible to anyone driving the camera by curl —
  // which is how this command gets used.
  struct HgMapResult {
    int    ch;        // -1 = never run
    double px, py;
    bool   ok;
    double wx, wy;
    const char* reason;  // literal; empty when ok
  } hg_map_;

  // Result of the most recent ANCHOR_SET_ALL / ANCHOR_SAVE, in /status for the
  // same reason hg_map_ is — but here it is closer to required than convenient.
  // ANCHOR_SET_ALL carries up to 24 hand-entered triples, so it is the one
  // command an operator WILL get wrong, and every rejection reason names the
  // offending id. With the reason only on stdout, a refused command and an
  // accepted one look identical to whoever typed it: the marker list in /status
  // simply does not change, which reads as "the camera ignored me".
  //
  // The reason is copied rather than pointed at: HomographyMapper::FailReason()
  // returns a formatted buffer that the next rejection overwrites. Contains
  // only digits and punctuation this code printf'd, so it needs no JSON
  // escaping — see the note on JsonBuf.
  struct AnchorCmdResult {
    int  ch;        // -1 = never run
    // Always what this lens HAS now, not what the command tried to set. On a
    // rejection the operator's next question is "so what is registered?", and
    // answering with the count from the command that just failed would say the
    // rejection had taken effect.
    int  n;
    bool ok;
    char reason[192];
  } anchor_cmd_;

  // Shrink factor used while this channel is SEARCHING (1 = full resolution).
  // Per channel and runtime-settable because the usable value depends on how
  // far that particular lens is from its markers — the limit is apparent
  // marker size in pixels, which differs per lens even on one camera.
  int  search_scale_[kMaxChannels];

  // Duty-cycle governor (DETECT_DUTY_PCT), PER CHANNEL. No frame on a channel
  // is detected before that channel's instant; after each search the slot is
  // pushed out in proportion to what the search cost, so an expensive channel
  // throttles itself rather than the thread.
  //
  // Per channel and not shared: one shared deadline meant an expensive channel
  // blocked every OTHER channel too, and whichever frame happened to arrive
  // when the block lifted won the slot. That starved whole lenses for seconds
  // at a time (7..13 s observed on ch2/ch3, 2026-08-04) while a cheap tracking
  // channel kept winning the race. Each channel now owns its own share of the
  // budget, so a lens can only slow itself down.
  long          detect_budget_until_ms_[kMaxChannels];

  // The budget itself, as a percentage of wall clock, settable by DUTY <pct>.
  //
  // Runtime rather than the compile-time constant it started as, because the
  // right value depends on what the lenses are looking at that day, and finding
  // it meant a 6-8 minute rebuild-reinstall per attempt. DETECT_DUTY_PCT is now
  // the starting value only.
  //
  // NOT persisted, deliberately, unlike K/dist and the anchors. Those are
  // measurements of the world that took a person with a tape measure; this is a
  // load-tuning knob whose right value follows the load. A saved 95 from an
  // afternoon of debugging would come back after a reboot as an app that cannot
  // serve its own page, with nothing on screen to say why. A restart returning
  // to a known-good default is the more useful behaviour.
  int           detect_duty_pct_;
  // Bounds for DUTY, and both of them are about staying reachable.
  //
  // The high end matters more than it looks: the wait is
  // cost * (100*n/DUTY - 1), so at DUTY=100 with one channel active the wait is
  // exactly zero and the search may run back-to-back forever. That is the state
  // that took HTTP down and, with it, the ability to undo it. 95 leaves a
  // sliver that keeps the event queue from growing without bound.
  //
  // The low end is not a safety limit but an honesty one: below ~5 a 250 ms
  // scan waits over 20 s, which is indistinguishable from a broken lens. DETECT
  // <ch> 0 already says "off" clearly, so there is no need for a duty that says
  // it obscurely.
  static const int kDutyMin = 5;
  static const int kDutyMax = 95;
  // Frames the governor dropped, per channel. Per channel and not one total,
  // because the number is only meaningful next to that channel's `frames`:
  // together they give the lens's real duty (detected vs offered) — one shared
  // total cannot say which lens is being throttled.
  unsigned long detect_skipped_[kMaxChannels];

  // The same duty, over the last kRecentWindow frames the camera OFFERED this
  // lens rather than since start-up.
  //
  // Cumulative counters answer "what has this lens done all day", which is the
  // wrong question for a live page: after an hour of running, turning dynROI on
  // barely moves a lifetime ratio, so the number stays wrong for minutes
  // exactly when someone is watching it to see whether the change worked. A
  // sliding window is current by construction — at 5 fps it spans ~20 s.
  //
  // A ring of one byte per frame, and a running sum kept in step with it, so
  // recording a frame is a store, an add and a subtract — no scan.
  static const int kRecentWindow = 100;
  void NoteFrame(int ch, bool skipped);
  unsigned char recent_skip_[kMaxChannels][kRecentWindow];  // 1 = governor dropped it
  int recent_head_[kMaxChannels];     // next slot to overwrite
  int recent_n_[kMaxChannels];        // frames recorded so far, capped at the window
  int recent_skipped_[kMaxChannels];  // 1s currently inside the window

  // How long the last frame sat in the scheduler queue before this app got to
  // it, per lens.
  //
  // This is the latency that was previously invisible. A pose packet's t_frame
  // is stamped when ProcessRawVideo() STARTS, so everything the frame spent
  // waiting behind other channels had already elapsed — "proc" told the server
  // 250 ms while the true age of the image could be well over a second.
  //
  // Derived rather than read: pts and the wall clock have an unknown constant
  // offset, so the smallest (t_frame_ms - pts) ever seen is taken as that
  // offset (a frame that waited for nothing), and anything above it is queue
  // time. Slow drift between the two clocks would inflate this over a long
  // run; for a debug figure that is a fair trade for needing no clock sync.
  // Smallest (wall clock - pts) ever seen, PER CHANNEL. The four lenses are not
  // assumed to share a pipeline delay — see the use site.
  long          pts_offset_min_[kMaxChannels];

  // Raw frames the SDK has delivered on each lens, counted before this app
  // decides anything about them. See ProcessRawVideo for why the existing
  // frames/skipped pair could not answer the question this does.
  unsigned long delivered_[kMaxChannels];
  // Previous reading, for differencing into a rate. Same shape as cpu_pct:
  // cumulative counters only mean something when subtracted. The instant it was
  // taken is stats_sample_ms_, shared with every other rate on the page.
  unsigned long delivered_prev_[kMaxChannels];
  // When each lens last delivered, for the detection governor's channel count.
  // Separate from last_frame_ms_ (which is dashboard-only and compiled out with
  // the status page) because the governor has to work either way, and it is set
  // for EVERY delivered frame — including ones a switched-off channel drops
  // immediately — since the question is whether the stream is alive, not
  // whether we did anything with it. 0 = nothing yet.
  long          last_delivery_ms_[kMaxChannels];
  // What the governor last divided the duty by, reported as-is rather than
  // recomputed for /status. n decides every lens's share, so when a lens drops
  // out of the count the only visible effect is the OTHERS speeding up — which
  // reads as a performance change with no cause on screen. Recomputing it in
  // BuildStatusJson would answer "what would n be if asked now", and the
  // question is what the governor actually used. 0 = no frame charged yet.
  int           governor_active_;
  // How long a silent lens stays in the governor's denominator. Generous on
  // purpose: it only has to outlast a real delivery gap, and a lens limping
  // along at 1 fps is still consuming the thread, so timing it out would speed
  // the others up into work that is genuinely still there.
  static const long kActiveIdleMs = 2000;
  long          last_queue_ms_[kMaxChannels];

  // Raw-video EVENTS, counted before the frame is taken apart.
  //
  // delivered_ above cannot answer the question this does, because it is
  // indexed by a channel number that only exists after DeserializeBaseObject()
  // and GetRawImage() have both succeeded — an event that arrives and yields no
  // image is dropped without being counted anywhere, and reads exactly like an
  // event that never arrived.
  //
  // That ambiguity is the whole reason these exist. GroupSPMgrVideoRaw3
  // (DEF_FULL_RAW) has now been tried in three manifest shapes and produced
  // 0 fps every time (RAW_VIDEO_LIMITS §2.4, §2.4.1, and 2026-08-05 with
  // SPMgrVideoRaw_0 added to ReceiverNames). "The firmware does not publish
  // that group" and "it publishes frames this app fails to unpack" call for
  // completely different next moves — one closes the raw path, the other is a
  // bug in this app — and until now nothing here could tell them apart.
  //
  // Cheap enough to keep afterwards: two increments per event, and they stay
  // meaningful for any future source change.
  unsigned long raw_events_;     // eVideoRawData callbacks entered
  unsigned long raw_no_image_;   // ...of which produced no RawImage at all

  // NOTE: the eVideoConnect / SensorInfo probe that used to live here is gone
  // (2026-08-04). It was an experiment to make the camera state its own raw
  // fps ceiling instead of us inferring it; the event never fires on this
  // model, so it only ever reported zeros. The ceiling is settled by other
  // means and written up in docs/RAW_VIDEO_LIMITS.md — that document is the
  // record, so keeping dead members to display 0 bought nothing.

#if ENABLE_STATUS_PAGE
  // The ONLY state the dashboard adds to the per-frame path, and the reason it
  // is this short: frame count (seq_), detect cost, ROI and tracker state are
  // all readable from what already exists, so nothing here duplicates them.
  // Plain ints, written once per frame, read only from the same thread.
  long          last_frame_ms_[kMaxChannels];  // epoch ms; 0 = no frame yet
  int           last_markers_[kMaxChannels];   // detections in that frame
  int           last_w_[kMaxChannels];
  int           last_h_[kMaxChannels];

  // Corners of the last frame's detections, kept ONLY so /status can draw a
  // coordinate overlay in the browser (raw corner quads + the dynROI box, no
  // photo — same idea as the RPi dashboard's 마커검출 tab, minus the
  // background image). Ported from that tab; see index.html for the undistort
  // math (client-side, needs no K/D on the camera).
  //
  // Deliberately includes SEARCH-stage approximate corners (scale > 1), unlike
  // the wire packets which suppress them — a debug overlay showing "roughly
  // here, about to refine" is honest and useful; a robot silently accepting an
  // approximate coordinate is not. BuildStatusJson marks each one "approx".
  std::vector<ArucoProcessor::Detection> last_dets_[kMaxChannels];
  bool          last_dets_approx_[kMaxChannels];

  long          start_ms_;            // app start, for uptime
  // Previous CPU reading, so CPU can be reported as a percentage over a recent
  // interval rather than as a meaningless average since boot. The instant it
  // was taken is stats_sample_ms_ below, shared with every other rate.
  double        cpu_sample_cpu_s_;

  // Per-core load of the WHOLE CAMERA, from /proc/stat, differenced between
  // /status calls the same way cpu_pct is.
  //
  // Separate from cpu_pct because the two answer different questions and get
  // mistaken for each other. cpu_pct is THIS APP against wall clock, so on a
  // two-core box it can legitimately read 130%. The figures here are each
  // core's total occupancy including the firmware's own work, so each is capped
  // at 100. "app 83%" next to "cpu0 61% / cpu1 34%" is the pair that says the
  // app's one scheduler thread is being moved between both cores rather than
  // pinned to one, and that neither core is saturated. Neither number says
  // that on its own.
  //
  // /proc/stat counters are cumulative since boot, so one reading is an
  // uptime-wide average and useless for "is it busy now" — only the difference
  // between two readings means anything, which is why the previous one is kept.
  static const int kMaxCores = 8;
  int                core_n_;                     // 0 = never sampled
  unsigned long long core_busy_[kMaxCores];
  unsigned long long core_total_[kMaxCores];

  // ONE sampling window for every rate on the page (cpu_pct, core_pct, in_fps),
  // rolled at most once a second, with the results cached until it rolls again.
  //
  // Each of those numbers is a difference between two readings, so each needs a
  // baseline and an interval to divide by. Until 2026-08-05 each kept its own
  // baseline and rolled it ON EVERY RENDER, which made the interval whatever
  // the gap between two /status requests happened to be. That is not a property
  // of the camera — it is a property of who else is polling. With the dashboard
  // open at 2 s and a curl arriving 40 ms behind it, the curl measured over
  // 40 ms.
  //
  // At 5 fps a 0.2 s window can hold either 0 or 1 frames, so the answer
  // quantises to 0 or 5 with nothing in between. Measured on .13: a healthy
  // lens reported 0.0 fps, and this app reported 2.5% CPU while it was really
  // near 60%. Those readings were not noisy, they were unanswerable — the
  // window was too short to contain the question. Caching means requests
  // arriving inside a window all receive the same still-valid answer instead of
  // each destroying the next one's baseline; readers stop interacting at all.
  //
  // One shared epoch rather than three also makes the three figures describe
  // the SAME interval, so "app 60% / cpu0 71% / 4.9 fps" is a single coherent
  // snapshot and not three measurements of three different moments.
  //
  // Kept here and not on the frame path because delivery STOPPING has to show
  // up too: nothing runs in ProcessRawVideo when no frame arrives, so a rate
  // maintained there would freeze at its last healthy value forever.
  static const long kStatsWindowMs = 1000;
  // How often the allocator is asked to return its spare pages. See the call
  // site for why this is minutes-scale rather than per window.
  static const long kTrimIntervalMs = 30000;
  long          last_trim_ms_;

  // Where the per-frame heap growth is coming from: inside the marker search,
  // or in what this app does afterwards. Sampled every kHeapProbeEvery frames
  // so the mallinfo2 calls stay off the per-frame cost. See the probe sites.
  static const unsigned long kHeapProbeEvery = 10;
  unsigned long heap_probe_seq_;   // frames offered since start, for the modulo
  unsigned long heap_probe_n_;     // probes actually completed
  long long     heap_detect_bytes_;
  long long     heap_rest_bytes_;
  long          stats_sample_ms_;     // 0 = no baseline yet; rates read -1
  double        stats_window_s_;      // interval the cached figures cover
  double        cpu_pct_;             // cached; -1 = not measurable yet
  double        in_fps_[kMaxChannels];
  double        core_pct_[kMaxCores];
#endif
};
