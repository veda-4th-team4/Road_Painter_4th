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
  void ReportDynRoi() const;
  void ReportDetect() const;

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
  // Record an ANCHOR_* outcome into anchor_cmd_ and the log, in one place.
  void SetAnchorResult(int ch, int n, bool ok, const char* reason);
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
  long          pts_offset_min_;
  long          last_queue_ms_[kMaxChannels];

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
  // Previous /status sample, so CPU can be reported as a percentage over the
  // interval between two presses of [Refresh] rather than as a meaningless
  // average since boot.
  long          cpu_sample_wall_ms_;
  double        cpu_sample_cpu_s_;
#endif
};
