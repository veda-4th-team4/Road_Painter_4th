#pragma once

#include <memory>

#include "app_config.h"  // ENABLE_STATUS_PAGE gates the members below
#include "aruco_processor.h"
#include "component.h"
#include "dyn_roi.h"
#include "homography_mapper.h"
#include "i_sample_component.h"
#include "intrinsics_calib.h"
#include "proximity_guard.h"
#include "wiseai_metadata.h"

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
  // WiseAI bbox 이벤트 — 지금은 원문 XML을 로그로 남기기만 한다 (실측 전).
  void ProcessWiseAiMetadata(Event* event);
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
  // CENTRAL_LINK/CENTRAL_POS/CENTRAL_ID/CENTRAL_QUERY/CENTRAL_HMATRIX —
  // operator control of the central-server TLS link, over the RPi dashboard
  // channel (same pattern as cctv_app). Separate from the CALIB_START/
  // SELECT_CHANNEL commands the central server itself sends — those arrive
  // on the TLS link and are drained in PollCentralCommands(), not here.
  bool HandleCentral(const char* cmd);
  // `action`/`detail` are echoed into the CENTRAL_STATUS line so the
  // dashboard tab can show the result of whatever just ran (e.g. "hmatrix" /
  // "sent") without a separate reply channel. Defaulted so a plain query
  // (CENTRAL_QUERY, or after CENTRAL_LINK/POS/ID) can just redraw current
  // state with no claimed action.
  void ReportCentral(const char* action = "", const char* detail = "") const;
  // Cross-check a CENTRAL_HMATRIX payload against this lens's actual state
  // before it goes out. True = send it. False = `reason` says why not.
  // The payload stays the dashboard's format — this only catches the cases
  // where it asserts something this app can see is false (coord_mode vs the
  // space the H was fitted in, H_marker without a ready marker plane,
  // image_size against the frames this lens really delivers). See the
  // definition for what is deliberately NOT checked.
  bool ValidateHMatrixPayload(const char* payload, char* reason,
                              size_t reason_size) const;
  // Commands the central server (not the RPi dashboard) sends over the TLS
  // link itself: CALIB_START, SELECT_CHANNEL.
  void PollCentralCommands();
  // 주행 캘리(오도메트리) — 서버가 TLS 링크로 보내는 CALIB_CAPTURE/CALIB_DONE 의
  // 진입점. PollCentralCommands() 가 문자열을 뜯어 여기로 넘긴다.
  //
  // `ch` 는 여기서부터 **내부 0-based** 다. 와이어의 1-based -> 내부 변환은
  // PollCentralCommands() 한 곳에서만 한다 — 2026-08-11 에 POS 가 0-based 로
  // 나가 CH1 이 서버 게이트에서 통째로 버려진 적이 있어, 변환 지점을 흩지 않는다.
  void CentralCalibCapture(int ch, int point_index, double wx_mm, double wy_mm,
                           const char* request_id);
  void CentralCalibDone(int ch, double m_mm, double n_mm, const char* request_id);
  // 서버가 세션을 접을 때(조작자 취소·캡처 타임아웃·세션 타임아웃) 보내는
  // CALIB_CANCEL. 수집을 멈추고 CALIB_STOPPED 로 답한다.
  //
  // **답을 반드시 보낸다** — 접을 세션이 없었어도 보낸다. 서버는 이 ack 하나로
  // 취소 절차를 닫고(로봇은 ABORT_DRAW 로 이미 서 있다), 못 받으면
  // calib_cancel_ack_ms(5초)를 기다린 뒤 cancel_failed 로 기록한다.
  // "우리 쪽엔 돌던 게 없었다"는 정보는 침묵이 아니라 payload 의 aborted:0 으로
  // 전한다.
  //
  // `ch` 가 음수면 열려 있는 주행 캘리 세션을 **전부** 접는다. 서버가 보낸
  // ch 를 못 읽은 경우인데(CALIB_CANCEL_BAD), 취소를 못 한 채로 두는 것보다
  // 넓게 접는 편이 안전하다 — 동시에 도는 주행 세션은 어차피 하나다.
  void CentralCalibCancel(int ch, const char* request_id);
  // 캡처 한 지점의 결과를 서버로 돌려준다. ok=true 면 CALIB_CAPTURE_OK,
  // false 면 CALIB_CAPTURE_FAIL{reason}.
  //
  // 실패를 반드시 보내야 하는 이유: 서버는 ack 를 받아야 다음 지점으로 GO 를
  // 보낸다(계획서 §3-1). 아무 것도 안 보내면 세션이 타임아웃까지 멈춰 서고,
  // 운영자에게는 "로봇이 안 움직인다"로만 보인다.
  void ReportCentralCapture(int ch, int point_index, bool ok,
                            double u, double v, double spread_px,
                            const char* reason, const char* request_id) const;
  // 프레임마다 — 무장된 지점이 있으면 표본을 모아 정지 판정을 한다.
  void OdomFeedFrame(int ch, const std::vector<ArucoProcessor::Detection>& dets,
                     long t_capture, long now_ms);
  // 캡처·세션 타임아웃의 벽시계 안전망. 폴링 자리에서 매 이벤트 돈다.
  //
  // OdomFeedFrame() 안의 같은 검사만으로는 부족하다 — 그건 **그 채널의
  // !approximate 프레임**이 와야 실행된다. 동적 ROI 가 추정 좌표만 내거나
  // 거버너에 눌려 그 렌즈의 프레임이 끊기면 카메라는 실패조차 보고하지 못하고
  // 침묵하고, 서버는 15초 뒤 세션을 통째로 접는다(capture_timeout). 조작자에게는
  // "로봇이 한 바퀴도 못 돌고 멈췄다"로만 보이고 사유가 로그 어디에도 안 남는다.
  void OdomSweepTimeouts();
  // 주행 캘리 결과(잔차·폐합·두 방식 비교)를 대시보드로.
  void ReportOdomResult(int ch, bool ok, const char* reason,
                        const char* request_id) const;
  // 세션 수준 실패(too_few_points/fit_failed)를 서버로도 알린다 (2026-08-13).
  //
  // ReportOdomResult() 는 대시보드(포즈 링크) 전용이라 central 링크는 침묵한다.
  // ADMIN 이 연 세션은 CALIB_DONE 시점에 이미 닫혀 있어 그래도 무해했지만, Qt가
  // 연 세션은 서버가 calib_odo_result_wait_ms 동안 H_MATRIX 나 CALIB_FAIL 을
  // 기다린다 — 안 보내면 서버 자체 timeout 으로 접혀 조작자에게 "카메라 응답
  // 없음"으로 잘못 뜬다(서버팀 8/13 회신 §1).
  void ReportCentralCalibFail(int ch, const char* reason,
                              const char* request_id) const;
  // 주행 캘리 번들을 조립해 중앙 서버로. H_marker 가 측정값, H_floor 를 역산한다.
  //
  // 전송에 성공하면 true. `why` 가 NULL 이 아니면 실패 사유(리터럴)를 채운다 —
  // ODOM_RESEND 가 조작자에게 "왜 안 나갔는지"를 그대로 보여주기 위해서다.
  // 주행 직후 자동 경로는 사유를 로그로만 남기면 되므로 기본값 NULL 로 둔다.
  bool SendCalibBundle(int ch, const char* request_id,
                       const char** why = NULL) const;
  // 정적 앵커 캘리 번들을 조립해 중앙 서버로. 위와 방향만 반대다 — H_floor 가
  // 측정값(앵커 피팅 결과)이고 H_marker 를 마커 높이로 파생시킨다.
  //
  // 2026-08-14. 이전까지 floor 결과는 카메라가 서버로 직접 보내지 않았다:
  // 대시보드가 HG_QUERY 로 값을 받아 캐시해 뒀다가 자기가 번들을 조립해
  // CENTRAL_HMATRIX 로 되돌려보내면 카메라가 통과만 시켰다. 그 왕복 때문에
  //   - 대시보드가 안 물어보면 서버는 floor H 를 영영 모르고,
  //   - 대시보드가 아는 값(image_size 는 드롭다운 추정, coord_mode 는 명령
  //     이력에서 유추, K/D 는 놓치면 자리표시자 [[1400,0,960],…])으로 조립되며,
  //   - "조회인지 새 캘리인지"를 값 변화로 추측해야 했다(HG 는 한 타입이다).
  // 셋 다 카메라가 자기 상태로 조립하면 존재하지 않는 문제다. 주행 캘리와 같은
  // 모양으로 맞춘다 — 피팅이 끝나는 그 자리에서 카메라가 직접 보낸다.
  bool SendFloorBundle(int ch, const char* request_id,
                       const char** why = NULL) const;
  // 위 둘의 공통부. 두 방식의 차이는 어느 평면을 재고 어느 쪽을 파생시켰는지와
  // 품질 지표뿐이라, K/D·image_size 게이트와 JSON 조립은 한 벌만 둔다 — 갈라
  // 두면 한쪽만 고쳐진 채 남는다(marker_height 의 _m/_mm 이 그렇게 어긋났다).
  //
  // `extra_json` 은 H_floor 뒤에 그대로 이어 붙일 조각이다(쉼표로 시작하거나
  // 빈 문자열). 방식별 품질 지표가 여기로 들어간다.
  bool EmitCalibBundle(int ch, const char* request_id, const char* method,
                       const char* calib_id, bool undistorted,
                       const double Hm[9], const double Hf[9],
                       double canvas_w_mm, double canvas_h_mm,
                       const char* extra_json, const char** why) const;
  // ODOM_PREFER / ODOM_PREFER_QUERY 의 답 — 지금 로봇 측위가 측정 H_marker 를
  // 쓰는지 파생값을 쓰는지. `ok=false` 면 전환이 거부된 것이고 사유가 실린다.
  void ReportOdomPrefer(int ch, bool ok) const;
  // ODOM_SETTLE / ODOM_SETTLE_QUERY 의 답 — 정지 판정 임계값과 창 크기, 그리고
  // 창의 상한(컴파일 시점 버퍼 크기).
  void ReportOdomSettle() const;
  bool HandleDynRoi(const char* cmd);
  bool HandleDynRoiIds(const char* cmd);
  bool HandleDetect(const char* cmd);
  bool HandleScale(const char* cmd);
  // ARUCO_SCAN <ch> [<passes> [win]] -- per-lens adaptive-threshold sweep
  // (ArucoProcessor::setScanPasses). Ported from cctv_app's single-lens
  // ARUCO_SCAN (2026-08-10) -- channel-scoped here since each lens has its
  // own ArucoProcessor and the speed/robustness trade-off can differ lens to
  // lens. Bare "ARUCO_SCAN" (no ch) reports every lens, same convention as
  // ReportDetect().
  bool HandleArucoScan(const char* cmd);
  // DETECT_PARAM <ch> [<name> <value>] -- one of the four detection-RATE
  // knobs (perim/ecc/thresh/poly, see ArucoProcessor::setDetectParam), for
  // one lens. Ported from cctv_app (2026-08-10), channel-scoped for the same
  // reason as ARUCO_SCAN. Bare "DETECT_PARAM" reports every lens; "DETECT_PARAM
  // <ch>" alone reports just that one.
  bool HandleDetectParam(const char* cmd);
  bool HandleDynRoiCh(const char* cmd);
  // DUTY <pct> — the detection budget, at runtime. Named DUTY and not
  // DETECT_DUTY because HandleDetect() matches "DETECT" with strstr, so any
  // name containing it is swallowed there and silently does nothing.
  bool HandleDuty(const char* cmd);
  // HOLD_MS <ms> — the flicker-hold window, at runtime. See marker_hold_ms_.
  bool HandleHoldMs(const char* cmd);
  void ReportDynRoi() const;
  void ReportDetect() const;
  void ReportDuty() const;
  void ReportHoldMs() const;
  // Pushes CPU_STAT on the pose link, throttled to kCpuReportIntervalMs, from
  // ProcessRawVideo() (2026-08-11) — ported from cctv_app's report_cpu_if_due(),
  // but reusing this file's own richer /status figures (cpu_pct_/core_pct_)
  // instead of re-deriving CPU math from scratch. Not const: it calls
  // SampleRates(), which updates cpu_pct_/core_pct_ (self-throttled to
  // kStatsWindowMs, so calling it from here as well as from /status, if
  // anything ever polls that too, is safe).
  void ReportCpu();
  void ReportArucoScan(int ch) const;
  // `name` must be one of "perim"/"ecc"/"thresh"/"poly" or "" -- never the raw
  // wire buffer. This file's JSON strings are otherwise always compile-time
  // constants (see JsonBuf's "nothing from the network is echoed back" rule
  // in sample_component.cc); HandleDetectParam() whitelists before calling in.
  void ReportDetectParam(int ch, bool ok, const char* name, const char* reason) const;

  // Intrinsics (K/dist) calibration. Same command vocabulary the RPi dashboard
  // already speaks (CALIB_K_*), so the page here and the server there drive it
  // with identical strings — the one difference is that START now names a lens,
  // because this camera has four.
  bool HandleCalibK(const char* cmd);
  // Everything below is new (2026-08-10): until now HandleCalibK() changed
  // calib_'s state but never told the pose link about it — a dashboard driving
  // K/dist over TCP (as opposed to the on-camera page, which just re-reads
  // /status) had no way to see a session start, a capture accept/reject, a
  // compute finish, or a saved value. These push exactly what /status already
  // has, over the wire, in the RPi dashboard's existing message shapes — see
  // each ReportCalibK*'s call site in HandleCalibK()/ProcessRawVideo() for
  // which command triggers which.
  //
  // Shared "board + this lens's session" shape (CALIB_K_CONFIG/STATUS/UNDO —
  // cctv.py's combined handler). `type` picks which of the three; `ok`/`reason`
  // carry the triggering command's own outcome, everything else is the
  // CURRENT truth, not an echo of the request.
  void ReportCalibKConfig(int ch, const char* type, bool ok, const char* reason) const;
  // CALIB_K_START's ack: board description + this lens's target view count.
  void ReportCalibKAck(int ch) const;
  // One accepted or rejected capture, from IntrinsicsCalib::LastQuality(ch) —
  // called once TakePendingCapture() actually resolves in ProcessRawVideo,
  // since HandleCalibK only ever ARMS the capture (see CALIB_K_CAPTURE's
  // comment) and has no frame in hand to grade yet.
  void ReportCalibKProgress(int ch) const;
  void ReportCalibKComputing(int ch) const;
  // The held ChArUco corner viewfinder (IntrinsicsCalib::ProbeCorners/
  // ProbeIds), pushed at the same CALIB_PROBE_MS cadence ProbeIfDue() already
  // throttles itself to (2026-08-11) -- so the dashboard can draw where the
  // board is, the same way the marker-detection tab draws ArUco corners.
  // Until now this only ever reached /status, which nothing on the pose-link
  // side polls (this project's dashboard is push-only, see ReportCpu()'s own
  // doc comment for the same gap found the same day).
  void ReportCalibKProbe(int ch) const;
  // CALIB_K_COMPUTE's result: success (rms/views/pruned/K/dist) or failure
  // (IntrinsicsCalib::FailReason()).
  void ReportCalibKResult(int ch, bool ok) const;
  void ReportCalibKSave(int ch, bool ok) const;
  void ReportCalibKBoardSave(bool ok) const;
  // Global, not per-lens — CALIB_K_GATE applies to every session's quality
  // gates at once (IntrinsicsCalib::Gates() is one flag, not kChannels of them).
  void ReportCalibKGate() const;
  // CALIB_K_QUERY's answer: this lens's currently loaded fx/fy/cx/cy/dist, or
  // "available":false if none. New recognized command (see HandleCalibK) —
  // the RPi dashboard already sent this expecting a reply; nothing answered.
  void ReportCalibKQuery(int ch) const;
  // Homography (HG_*) — same vocabulary cctv_app and the RPi dashboard use,
  // plus the <ch> argument every one of them now needs.
  bool HandleHomography(const char* cmd);
  // HG_QUERY's answer on the pose link, mirroring ReportDynRoi(). The page
  // itself reads /status; this exists because the RPi asks over TCP.
  void ReportHomography(int ch) const;
  // The LOO residual summary (see HomographyMapper::FitResult and its own
  // "no split, cross-validate on everything" design comment) alongside
  // ReportHomography — added 2026-08-10, same reason as the ReportCalibK*
  // block: a TCP-only dashboard could not tell a good fit from a bad one
  // without /status. Summary only (n, rmse_in/loo_mm, max_loo_mm/id) — the
  // per-point breakdown is ReportHomographyFitPoints below, kept SEPARATE
  // because up to kMaxAnchors (24) points would not fit one line under
  // POSE_SENDER_MAX_LINE (1024 B) alongside everything else here.
  void ReportHomographyFit(int ch) const;
  void ReportHomographyFitPoints(int ch) const;
  // CALIB_START's immediate ack — bare, on purpose: it means only "collecting
  // began", and the RPi dashboard's existing handler for it (cctv_app-era,
  // kept as-is on the Pi side) does not read any fields.
  void ReportHgAck(int ch) const;
  // CALIB_START's terminal signal — sent once when a collection session ENDS
  // (target reached or frame budget spent). ReportHomography/ReportHomographyFit
  // above also carry the result, but they double as the HG_QUERY reply, so
  // neither uniquely means "a session just finished"; the RPi dashboard raises a
  // "collecting…" banner on ReportHgAck and had no message to clear it with —
  // this is that message.
  void ReportHgDone(int ch) const;
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
#if ENABLE_SHELL_CMD
  // "SHELL <cmd>" — see ENABLE_SHELL_CMD's warning in app_config.h before
  // touching this. Matched by an anchored prefix (not strstr), and checked
  // FIRST in DispatchCommand(): every other handler here matches by loose
  // substring, and a shell command's own free-text argument (unlike every
  // other command's fixed numeric/enum ones) could otherwise contain
  // something like "DETECT" or "CALIB_K_STATUS" and get intercepted before
  // ever reaching this one.
  bool HandleShell(const char* cmd);
#endif
  // Bounds check for a channel argument parsed off the wire (CALIB_K_*
  // reporting — see the ReportCalibK* block above). Same shape as
  // IntrinsicsCalib::ValidCh(), kept separately because that one is private to
  // a different class and a channel number here has not yet been handed to
  // anything that could validate it for us.
  bool ValidCh(int ch) const { return ch >= 0 && ch < kMaxChannels; }
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

  // Remember one marker's corners (and their mean, for FillRecentMarkers) as
  // "seen just now" on this channel. Called from every EXACT detection,
  // whether or not a homography session is open, so a session's first frame
  // already has recent history. See BuildHeldDets, FillRecentMarkers and
  // MARKER_HOLD_MS.
  void RememberMarker(int ch, const ArucoProcessor::Detection& d, long now_ms);
  // Append any id in the recent-sighting cache that is (a) not already among
  // ids[0..n) and (b) still within MARKER_HOLD_MS, up to `cap` total entries.
  // Returns the new count. Used only to paper over a registered anchor's
  // single-frame miss before handing the set to HomographyMapper::FeedFrame,
  // which otherwise discards a frame missing even one registered marker.
  int FillRecentMarkers(int ch, int* ids, float* cxs, float* cys, int n, int cap,
                        long now_ms) const;
  // What RememberWiseAiObject() reports about this object's zone membership
  // compared with the last frame that saw it. Only the two edges matter --
  // "still inside" is not an event, it is the steady state.
  enum ZoneEdge { kZoneNoChange = 0, kZoneEntered = 1, kZoneExited = 2 };
  // Same idea as RememberMarker(), for WiseAI's ObjectId instead of an ArUco
  // marker id -- see recent_wiseai_obj_'s comment for why this cache exists.
  //
  // `zone_now` is this frame's verdict for the object: 1 inside, 0 outside,
  // -1 unknown (this channel has no zone yet). The cache is what makes edge
  // detection possible at all -- a single frame only says where the object IS,
  // and Enter/Exit are about where it WAS. -1 parks the state rather than
  // clearing it, so syncing a zone mid-track doesn't manufacture an edge.
  ZoneEdge RememberWiseAiObject(int ch, const WiseAiDetection& d, long now_ms,
                                int zone_level);
  // How far this raw-sensor pixel is from the channel's IVA_SYNC zone:
  // 0 inside, >0 the distance to the nearest edge outside, -1 when the
  // channel has no zone (n < 3). Ray casting plus per-edge segment distance,
  // on the fixed arrays in IvaZone -- deliberately not cv::pointPolygonTest,
  // for the same reason ConvexHull2f() exists instead of cv::convexHull();
  // see its comment.
  //
  // A distance rather than a bool because the foot is judged as a disc, not a
  // point (see ZONE_FOOT_RADIUS_DIV): "inside" means the disc reaches the
  // zone, i.e. distance <= radius, and that test needs the number.
  float IvaZoneDistancePx(int ch, float x, float y) const;
  // ZONE_RADIUS <div> -- retune the foot disc without a rebuild. See
  // zone_radius_div_.
  bool HandleZoneRadius(const char* cmd);
  // DET_STREAM <0|1> -- start/stop the per-detection position feed. See
  // det_stream_.
  bool HandleDetStream(const char* cmd);
  // Look up the most recent bbox for `object_id` on this channel, if any and
  // if not older than WISEAI_OBJECT_HOLD_MS. False (out untouched) on a miss
  // -- an IVA_EVENT whose object was never seen in the bbox stream (or whose
  // sighting has since aged out) reports no bbox rather than a stale one.
  bool RecentWiseAiObjectBbox(int ch, const char* object_id, long now_ms,
                              float* left, float* top, float* right, float* bottom) const;
  // /status + the dashboard overlay's flicker hold: `fresh` (this frame's own
  // detections) UNION whatever is left in the recent-sighting cache that is
  // not already in `fresh` and is still within MARKER_HOLD_MS, each shown at
  // its last known corners. Per-marker, unlike a whole-frame hold: a channel
  // that finds a DIFFERENT subset of the same markers on consecutive attempts
  // (routine under the detection governor, which spaces attempts out) would
  // otherwise flicker every marker that dropped out of THIS frame's subset
  // even though `fresh` is never actually empty. Display only, same as
  // last_dets_ — SendPosePackets and FeedFrame already ran on `fresh` itself.
  void BuildHeldDets(int ch, const std::vector<ArucoProcessor::Detection>& fresh,
                     long now_ms, std::vector<ArucoProcessor::Detection>* out) const;

 private:
  std::string setting_changed_time_;

  // Everything below is PER CHANNEL. The v5 app kept one global detector and
  // one global tracker; with four lenses that model does not hold — each lens
  // has its own field of view, so its own ROI box and its own frame counter.
  std::unique_ptr<ArucoProcessor> aruco_[kMaxChannels];
  DynRoiTracker dynroi_[kMaxChannels];
  cv::Rect manual_roi_[kMaxChannels];   // operator ROI; empty = full frame
  unsigned long seq_[kMaxChannels];     // frame counter, shared by same-frame packets

  // Per-lens master switch for the marker search. Default OFF on every lens
  // (changed 2026-08-10; was on).
  //
  // This is not just a convenience. All four channels share ONE scheduler
  // thread, and a lens with no marker in view cannot use the dynamic ROI, so
  // it pays a full-frame scan every frame — measured 180..300 ms at 1080p on
  // .13 (2026-08-04). Four of those saturate the thread completely: video
  // events pile up and HTTP requests wait behind them until nginx gives up
  // (502 after ~20 s). Defaulting every lens ON meant paying that cost from
  // the moment the app started, whether or not anyone was using any of the
  // four lenses yet — measured 80%+ CPU across two cores at boot with dynROI
  // off, before an operator had touched anything. Starting OFF makes turning
  // a lens on (POST /detect, "DETECT <ch> 1") the one action that spends
  // this budget, instead of spending it by default and switching OFF the
  // lenses that turned out not to be needed.
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

  // BISECT 2026-08-18 step 2: member re-enabled (+ constructor init only,
  // command handler + BuildStatusJson emission still #if 0'd in the .cc) to
  // test whether SampleComponent's size/layout alone is the crash trigger.
  //
  // Result of the most recent IVA_SYNC, in /status for the same reason
  // hg_map_/anchor_cmd_ are: the operator drives this camera by curl, and a
  // command whose answer only exists as a stdout line is invisible to them.
  //
  // `px`/`py` are RAW SENSOR pixels (HomographyMapper::WorldToPixel's output
  // space) — the space WiseAI's own IVA area API expects, not the space this
  // lens fits H in. See WorldToPixel()'s comment for why that distinction
  // exists at all.
  struct IvaSyncResult {
    int    ch;    // -1 = never run
    bool   ok;
    int    n;     // convex hull point count; 0 on failure
    float  px[HomographyMapper::kMaxAnchors];
    float  py[HomographyMapper::kMaxAnchors];
    char   reason[192];
  } iva_sync_;

  // The zone each channel judges foot points against itself, in the same raw
  // pixels as iva_sync_ (which is where it is copied from, on every successful
  // IVA_SYNC). Per channel, unlike iva_sync_, which only remembers the LAST
  // sync on any channel because it exists to answer "what did that command
  // do"; this one has to survive a sync on a different lens.
  //
  // Why judge it ourselves when WiseAI already reports Enter/Exit for the very
  // polygon we push it: WiseAI decides membership with the bbox CENTRE, and
  // the centre of a standing person is not on the floor, while this zone is
  // built from floor anchors. Measured 2026-08-19 over 9 real events: every
  // Exit fired while the person's feet were still inside, the centre sitting
  // 162-402 px (median 262) above the foot point -- and varying with range, so
  // no constant offset can correct it. The foot point is the only part of the
  // bbox that lies on the plane this zone was calibrated on, which is what
  // makes PixelToWorld() (and so the zone itself) valid for it. WiseAI's own
  // IVA_EVENT is still forwarded, unchanged, as a coarse cross-check.
  struct IvaZone {
    int   n;  // 0 or <3 = this channel has no usable zone
    float px[HomographyMapper::kMaxAnchors];
    float py[HomographyMapper::kMaxAnchors];
    // The same polygon in world millimetres, filled by projecting each vertex
    // through PixelToWorld() when the zone is armed. Valid only when this
    // channel has a homography AND every vertex mapped -- the vertices come
    // from floor anchors, so they genuinely lie on the plane H was fitted to,
    // which is what makes the mapping meaningful.
    //
    // Why keep both: inside/outside is the same answer in either space (a
    // projective map preserves it), but DISTANCE is not. A margin of "0.3 m"
    // only means anything here, and a circle of fixed pixel radius is an
    // ellipse on the floor -- wider the further from the camera. So the
    // verdict can be pixel-only, but the bands cannot.
    bool  have_world;
    float wx[HomographyMapper::kMaxAnchors];
    float wy[HomographyMapper::kMaxAnchors];
  } iva_zone_[kMaxChannels];

  // Buffer bands outside the zone, in world millimetres (see
  // ZONE_MARGIN_DANGER_MM). Runtime-settable (ZONE_MARGIN) so they can be
  // tuned against the live overlay, and toggleable (ZONE_BANDS) because a
  // channel without a homography cannot draw them at all.
  int  zone_danger_mm_;
  int  zone_warn_mm_;
  bool zone_bands_on_;

  // Which level makes ZONE_EVENT fire: 3 = only inside the polygon, 2 = the
  // no-approach band as well (default), 1 = the caution band too.
  //
  // Default 2 because the point of the bands is to warn BEFORE someone is in
  // the hazard, not to confirm they already are. Settable so the threshold can
  // be moved without touching the band distances -- "where the line is" and
  // "how loud we are about it" are separate decisions.
  int  zone_alarm_level_;

  // Distance from a world point to this channel's world polygon, in mm:
  // 0 inside, >0 outside, -1 when the channel has no world-space zone.
  float IvaZoneWorldDistanceMm(int ch, double wx, double wy) const;
  // Project the freshly-armed zone's vertices to world mm, setting
  // IvaZone::have_world. Called from every place that arms a zone, so the two
  // copies can never disagree about which polygon they describe.
  void FillZoneWorld(int ch);
  // Push the two band outlines (as raw sensor pixels, ready to draw) for one
  // channel. No-op when bands are off or this channel has no world zone.
  void SendZoneBands(int ch);
  bool HandleZoneBands(const char* cmd);   // ZONE_BANDS <0|1>
  bool HandleZoneMargin(const char* cmd);  // ZONE_MARGIN <danger_mm> <warn_mm>
  bool HandleZoneAlarmLevel(const char* cmd);  // ZONE_ALARM_LEVEL <1|2|3>

  // Divisor turning a bbox width into the foot disc's radius -- see
  // ZONE_FOOT_RADIUS_DIV for why the radius scales with the box instead of
  // being a fixed pixel count. Runtime-settable (ZONE_RADIUS) because the
  // right value depends on the lens's field of view, and every rebuild here
  // costs an app restart, which drops the calibration with it.
  int zone_radius_div_;

  // Per-detection position feed for the dashboard overlay: one line per Human
  // per metadata frame, so the operator can watch where people actually are
  // relative to the zone instead of only learning about boundary crossings.
  //
  // OFF by default and runtime-toggled (DET_STREAM) because it is only worth
  // anything while somebody is looking at the overlay. Measured cost when on:
  // ~5 lines/s per channel typically, 13/s peak, ~150 bytes each -- 7.6 KB/s
  // even with four busy channels, which is nothing next to CAM_POSE. The
  // reason it defaults off is not bandwidth but noise: it would otherwise
  // stream continuously into a dashboard nobody has open.
  bool det_stream_;

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

  // How long a marker (ArUco or ChArUco corner/marker) not seen THIS attempt
  // still counts as present, in milliseconds — the flicker hold used by
  // BuildHeldDets/FillRecentMarkers (this file) and IntrinsicsCalib::ProbeIfDue
  // (passed in, so both stay in sync with the one operator-tunable value).
  // Runtime rather than the compile-time MARKER_HOLD_MS it started as, for the
  // same reason DUTY is: the right value depends on how marginal the current
  // print/distance/lighting is, and that is exactly the thing an operator is
  // staring at while deciding it. MARKER_HOLD_MS is the boot default only.
  // --- 주행 캘리: 한 지점의 캡처 대기 상태 (2026-08-12) --------------------
  //
  // 서버는 READY 를 받자마자 고정 대기 없이 CALIB_CAPTURE 를 보내고, "정말
  // 멈췄는가"의 판정은 카메라가 한다(계획서 §3-1). 검출 주기가 채널 수와
  // 거버너에 따라 4배까지 변해서 서버가 셀 수 있는 상수가 없기 때문이다.
  //
  // 판정 방식: 요청이 도착한 시각을 t0 로 잡고 **그 이후에 촬영된** 프레임만
  // 모은다(t_capture 기준). 그 전 프레임은 이동 중일 수 있고, 섞이면 평균이
  // 조용히 오염된다 — 파이프라인이 밀려 있을수록 더 그렇다.
  struct OdomPending {
    bool   armed;
    long   t0_ms;            // 요청 도착 시각. 이보다 이른 촬영분은 버린다.
    int    point_index;
    double wx_mm, wy_mm;
    bool   closing;          // 복귀 지점(피팅 제외, 폐합 진단용)
    char   request_id[64];
    int    n;
    float  us[ODOM_SETTLE_FRAMES];
    float  vs[ODOM_SETTLE_FRAMES];
  };
  OdomPending   odom_pending_[kMaxChannels];
  // 이번 주행 사각형의 크기. 번들의 canvas_mm 을 채운다 — 받은 지점들의 바운딩
  // 박스로 역산할 수도 있지만 서버가 CALIB_DONE 에 명시해 주기로 했다.
  double        odom_m_mm_[kMaxChannels];
  double        odom_n_mm_[kMaxChannels];
  // 세션 자체의 데드라인. CALIB_DONE 이 영영 안 오면(서버 재시작·링크 단절)
  // 그 렌즈가 계속 수집 모드로 남아 다른 캘리를 거부한다.
  long          odom_session_deadline_ms_[kMaxChannels];

  long          marker_hold_ms_;
  // Bounds for HOLD_MS. 0 is a legitimate value (hold off entirely — show
  // only this instant's truth); the high end is an honesty limit like
  // kDutyMax's, not a safety one: past a few seconds a "live" overlay is
  // showing positions old enough to not mean "here right now" any more.
  static const long kHoldMsMin = 0;
  static const long kHoldMsMax = 10000;

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

  // Per-channel, per-marker recent-sighting cache: id, last corners (+ their
  // mean, for FillRecentMarkers), and when. NOT gated by ENABLE_STATUS_PAGE —
  // this feeds FillRecentMarkers(), which affects what actually gets fitted
  // during a homography session, and BuildHeldDets(), which drives the
  // dashboard's flicker hold. Only EXACT detections are ever remembered here
  // (see the `!approximate` guard at the call site) — a SEARCH-stage low-res
  // corner is not trustworthy enough to stand in for a marker gone missing
  // later. See MARKER_HOLD_MS.
  struct RecentMarker {
    int   id;       // -1 = empty slot
    float cx, cy;   // mean of corners[], kept alongside for FillRecentMarkers
    cv::Point2f corners[4];
    long  seen_ms;
  };
  // Comfortably past HomographyMapper::kMaxAnchors (24): every registered
  // anchor plus a few unregistered markers that happen to be in view.
  static const int kMaxRecentMarkers = 32;
  RecentMarker recent_marker_[kMaxChannels][kMaxRecentMarkers];

  // Per-channel person-robot proximity state. One instance per lens, not
  // shared, for the same reason homography_ is one object holding four
  // matrices rather than four globals: "which lens is this reading for" has
  // to be impossible to lose track of. Configure()d once in the constructor
  // from the PROXIMITY_* defaults in app_config.h; Update()/Hold() are
  // driven from ProcessWiseAiMetadata() per Human detection.
  ProximityGuard proximity_guard_[kMaxChannels];

  // Per-channel "last known bbox" per WiseAI object id, keyed by the same
  // ObjectId WiseAI's bbox stream and its IVA-area Enter/Exit events both
  // carry. Exists purely to answer "where was this object when it crossed
  // the line" -- an IVA_EVENT names an object_id but never a position (see
  // WiseAiIvaAreaEvent's comment), and the bbox that goes with that id
  // arrived, if at all, in an earlier separate metadata callback, not the
  // same one as the event. Same replace-oldest-on-full pattern as
  // recent_marker_, for the same reason: a small fixed cache the frame path
  // can touch without allocating.
  struct RecentWiseAiObject {
    char  object_id[16];  // "" = empty slot
    float left, top, right, bottom;
    long  seen_ms;
    // Where this object was relative to the channel's IVA_SYNC zone the last
    // time we judged it: 1 inside, 0 outside, -1 not yet known. The previous
    // half of the Enter/Exit edge -- see RememberWiseAiObject()'s comment.
    signed char zone_state;
  };
  static const int kMaxRecentWiseAiObjects = 16;
  RecentWiseAiObject recent_wiseai_obj_[kMaxChannels][kMaxRecentWiseAiObjects];

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
  // What last_dets_ holds is BuildHeldDets()'s output: this frame's own
  // detections unioned with anything still live in recent_marker_ (the
  // flicker hold — see that struct and BuildHeldDets' doc comment). last_w_/
  // last_h_/last_frame_ms_ always reflect the frame itself, held or not.
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
  // How often CPU_STAT goes out on the pose link (ReportCpu(), 2026-08-11) —
  // same interval cctv_app used. Independent of anyone polling /status: this
  // camera is normally driven over the pose link only, and nothing there ever
  // called SampleRates() before, so cpu_pct_ sat at -1 forever with no /status
  // client to trigger it.
  static const long kCpuReportIntervalMs = 2000;
  long          cpu_report_ms_;

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
