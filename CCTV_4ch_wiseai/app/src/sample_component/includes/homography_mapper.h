#pragma once

#include <stddef.h>

#include <opencv2/core.hpp>

#include "intrinsics_calib.h"

/**
 * One registered marker: a printed id, and where its centre actually is on the
 * floor, in the shared world frame, in millimetres.
 *
 * There is deliberately NO "is this one for fitting or for validation" flag.
 * cctv_app split the list in two and held the validation markers out of the
 * fit, which means the matrix it actually used was built from fewer points than
 * were measured — 16 markers with 4 held back is a 12-point homography in
 * service. The LOO residual (see HomographyMapper::Fit) gives an honest
 * out-of-sample error while still fitting on everything, so the split buys
 * nothing and costs accuracy.
 */
struct Anchor {
  int    id;
  double wx_mm;
  double wy_mm;
};

/**
 * Pixel -> world (mm) mapping via a 3x3 homography, for a FOUR-lens camera.
 *
 * Ported from cctv_app/src/homography_mapper.cpp. The maths and the meaning of
 * H are unchanged; what changed is the shape, for the same reason the
 * intrinsics port changed shape:
 *
 *   - cctv_app was one lens, so H lived in a file-scope `static cv::Mat g_H`.
 *     Copying that here would give four lenses ONE shared matrix — three of
 *     them silently reporting a fourth lens's world coordinates. That is the
 *     single most expensive mistake available in this port, and it is a
 *     mistake that produces plausible numbers, so it does not announce itself.
 *
 *   - H is PER LENS. Each lens sees a different patch of floor from a
 *     different angle; there is no sense in which they could share one matrix.
 *
 * Deliberately NOT here yet (this is the first slice — see docs/
 * HOMOGRAPHY_HANDOFF_PROMPT.md for the full plan): the anchor table, the
 * CALIB_START collection session, LOO residuals, the marker-plane parallax
 * correction, and the undistort pixel path. What IS here is everything needed
 * to hold, persist and restore four independent matrices, because that is the
 * part every later step depends on and the part whose failure mode is silent.
 */
class HomographyMapper {
 public:
  static const int kChannels = 4;

  HomographyMapper();

  /**
   * Load each lens's persisted H.
   *
   * Takes the directory rather than finding it, because IntrinsicsCalib::Init()
   * already worked out where this app can write (by trying candidates, not by
   * knowing — see the comment there). A second copy of that candidate list is
   * a second thing to edit, and the failure mode of the two disagreeing is
   * that K and H persist to different directories: each save reports success,
   * and only one of them comes back after a restart.
   *
   * Must therefore run AFTER IntrinsicsCalib::Init().
   *
   * The reference is kept: undistorting a pixel needs that lens's K/dist, and
   * looking them up through the object that owns them means there is never a
   * stale copy of K sitting in here after a recalibration.
   */
  void Init(const IntrinsicsCalib& calib);

  // True once this lens has a usable H (loaded, injected or computed).
  bool Available(int ch) const;

  /**
   * True when PixelToWorld() can actually produce a coordinate right now.
   *
   * Not the same as Available(): a matrix fitted in undistorted space is
   * useless until that lens also has K/dist, and injecting one before
   * calibrating K is a legitimate order to work in. Reporting the two
   * separately is what keeps "H present but nothing maps" from looking like a
   * bug in the caller.
   */
  bool Mappable(int ch) const;

  // Copy one lens's H, row-major. False if that lens has none.
  bool Get(int ch, double h[9]) const;

  /**
   * Install a matrix computed elsewhere (the RPi/PC experiment, or a later
   * on-camera CALIB_START). Kept in RAM only — persisting is a separate,
   * operator-triggered step, exactly as it is for K/dist. A calibration that
   * saves itself on success is a calibration whose save failure nobody sees.
   *
   * `fitted_undistorted` records THE SPACE THIS MATRIX WAS FITTED IN, not the
   * camera's current mode. Storing the camera's mode instead is a trap worth
   * spelling out: a dashboard that fits in raw pixels and injects while the
   * camera is in undistort mode gets no error at all — every coordinate is
   * simply wrong by the lens distortion, and the flag that would have revealed
   * it is written down wrong too, so a reboot restores the same wrong pair.
   *
   * Refuses non-finite and singular matrices so a typo cannot poison a lens.
   */
  bool Set(int ch, const double h[9], bool fitted_undistorted);

  // The space Get(ch) is expressed in. Meaningless when !Available(ch).
  bool FittedUndistorted(int ch) const;

  /**
   * True when this lens's K/dist has been replaced since its H was fitted.
   *
   * An undistorted-space H is not a property of the floor alone: the pixels it
   * was fitted from had a particular K and dist taken out of them first, so
   * replacing those numbers leaves a matrix that maps a coordinate system
   * nothing produces any more. The residuals sitting next to it are worse than
   * useless — they were measured against the old lens and still read as proof.
   *
   * NOT self-correcting, and deliberately not fatal:
   *
   *   - the matrix is not discarded. A recalibration is often a REVERT to a
   *     value very close to the old one, and destroying an afternoon's fit as
   *     a side effect of a diagnostic press is not something an operator can
   *     undo. Clearing is left to them, as it is everywhere else here.
   *   - mapping keeps working. Refusing would stop the robot feed on the
   *     server with no explanation on that end, which trades a wrong number
   *     for a silence — and unlike the marker-plane refusal, this matrix is
   *     UNVERIFIED rather than known wrong.
   *
   * So it is loud instead: reported per lens in /status and shown on the
   * operations table, where it means "refit this lens before believing it".
   * Cleared by the next Set() — a fit or an injection re-baselines against
   * whatever K is current at that moment.
   *
   * Always false for a raw-fitted H, which never involved K in the first
   * place.
   */
  bool CalibStale(int ch) const;

  // --- coordinate space ----------------------------------------------------

  /**
   * Which pixel space this lens fits and maps in. True (default) = undistorted.
   *
   * Undistorted is the default because everything downstream needs it: the
   * marker-plane parallax correction decomposes H through K^-1 and is refused
   * outright on a raw-fitted matrix, because distortion gets absorbed into R
   * and t and the recovered plane normal is then wrong in a way that still
   * looks plausible.
   */
  bool CoordModeUndistorted(int ch) const;

  /**
   * Change it. Refused, rather than obeyed, in two cases:
   *
   *   - turning undistortion ON for a lens that has no K/dist. There is
   *     nothing to undistort with, and silently staying in raw would leave the
   *     flag and the behaviour disagreeing.
   *
   *   - while this lens HAS an H. The stored matrix belongs to the space it
   *     was fitted in; flipping the space around it produces a matrix and a
   *     flag that disagree, and that pair passes the raw-H check in the
   *     marker-plane code — the one gate built to catch exactly this. So the
   *     operator is asked to HG_CLEAR first and refit. Refusing rather than
   *     silently discarding the matrix follows SetBoard(), which refuses
   *     mid-session for the same reason: destroying calibration as a side
   *     effect of a settings change is not something a person can undo.
   */
  bool SetCoordMode(int ch, bool undistorted);

  /**
   * Put a raw sensor pixel into this lens's fitting space.
   *
   * Ported from cctv_app's h_pixel(). In raw mode this is a copy; in
   * undistorted mode it is cv::undistortPoints with K passed BOTH as the
   * camera matrix and as P. Omitting the second one is the classic mistake:
   * the call succeeds and returns normalised coordinates, which are a
   * perfectly well-formed answer to a different question.
   *
   * False when undistortion is wanted but that lens has no K/dist, or when
   * the undistortion did not converge at this pixel — see the implementation,
   * which explains why that has to be checked rather than trusted.
   * `reason` (optional) receives a literal explaining which.
   */
  bool PreparePixel(int ch, float px, float py, cv::Point2f* out,
                    const char** reason = NULL) const;

  /**
   * Raw sensor pixel -> world millimetres, the whole chain: PreparePixel()
   * then H. False when this lens has no H, when undistortion is needed and
   * unavailable, or when the point maps to the horizon (w ~ 0) — a pixel on
   * the vanishing line has no finite ground position, and returning a huge
   * number there would put the robot kilometres away rather than nowhere.
   */
  bool PixelToWorld(int ch, float px, float py, double* wx, double* wy,
                    const char** reason = NULL) const;

  /**
   * World millimetres -> raw sensor pixel, the inverse of PixelToWorld().
   *
   * Inverts H, then — when this lens fits in undistorted space — pushes the
   * result back through the forward distortion model (K, D), because the
   * caller wants a pixel in the space the SENSOR IMAGE is in, not the space H
   * was fitted in. That distinction matters here specifically because the one
   * caller (IVA_SYNC) hands the result to WiseAI's own detector, which sees
   * raw frames: an undistorted-space pixel handed to it would silently place
   * the area wrong by exactly the lens distortion, worse near the edges.
   *
   * False under the same conditions as PixelToWorld: no H, or (in undistorted
   * mode) no K/dist for this lens.
   */
  bool WorldToPixel(int ch, double wx, double wy, float* px, float* py,
                    const char** reason = NULL) const;

  /**
   * Which calibration this lens's H last came from — floor (FinishFit) or
   * odometry (FinishOdom), whichever completed most recently. kNone if
   * neither ever has.
   *
   * Exists for IVA_SYNC: "the region we have good coverage of" means
   * different points depending on which method was used last — the
   * registered floor anchors for a floor fit, the driven rectangle's stop
   * points for an odometry run — and there is no way to tell those apart
   * from Available(ch) alone (both leave H_ set). This is deliberately last
   * COMPLETED, not last STARTED or "odometry preferred for pose" (that's
   * PreferMeasured(), an unrelated choice about which H_marker is used for
   * the robot, not about which points describe the calibrated area).
   */
  enum class LastHSource { kNone, kFloor, kOdom };
  LastHSource LastHomographySource(int ch) const;

  /**
   * Raw sensor pixel for the i-th odometry stop point (of OdomCount(ch)),
   * i.e. the same undistort-to-raw step WorldToPixel() applies, run on a
   * point this lens already has a pixel for (AddOdomPoint's u/v) rather
   * than one derived by inverting H. Used instead of WorldToPixel() for an
   * odometry-sourced IVA_SYNC hull: those points were never floor anchors,
   * so there is no world-mm coordinate coming back through H_ to feed it —
   * only the pixel the operator's drive already measured.
   */
  bool OdomPointRawPixel(int ch, int i, float* px, float* py,
                         const char** reason = NULL) const;

  // Tape-measured camera height for this lens, 0 = not measured. Persisted
  // with H because it belongs to the same installation.
  double CameraHeightMm(int ch) const;

  // --- marker plane (parallax correction) ----------------------------------

  /**
   * H_marker: the homography for the plane the robot's MARKER sits on, not the
   * floor.
   *
   * A homography maps one physical plane. The floor H is fitted from markers
   * lying on the floor, so projecting a marker mounted 250 mm up through it
   * puts the robot somewhere it is not — pushed away from the camera's nadir,
   * by an amount that grows with distance from it:
   *
   *     ground = F - (h / Cz) * (F - nadir)
   *
   * With a camera 1.5 m up and a marker at 250 mm that is a sixth of the
   * distance to the nadir. At the far corner of a large work area that is
   * metres, and it is SYSTEMATIC — averaging frames does not touch it.
   *
   * Derived from the floor homography alone, by decomposition:
   *
   *     H_w2p = K [r1 r2 t]        (inverse of the stored pixel->world H)
   *     B     = K^-1 H_w2p          columns b1 b2 b3
   *     lambda= 1 / |b1|            (the scale homogeneity loses)
   *     r3    = r1 x r2             the plane normal the floor H cannot see
   *     H_marker_w2p = K [r1 r2 (h*r3 + t)]
   *
   * The floor H carries no information about the third dimension; r3 recovers
   * it from the orthonormality of a rotation matrix. That is the whole trick.
   *
   * REFUSED on a raw-fitted H, and this is not a style rule. K^-1 H assumes a
   * pinhole camera; with distortion still in the pixels it is absorbed into R
   * and t, the recovered r3 is tilted, and the correction comes out wrong in a
   * way that still looks like a correction.
   *
   * Ported from cctv_app/src/marker_plane.cpp. What changed for four lenses:
   * the marker height stays SHARED (one robot, one marker, one height) while
   * the camera height is PER LENS — same housing, different tilt, different
   * effective height — and H_marker is cached per lens because the pose path
   * needs it every frame.
   */
  double MarkerHeightMm() const;              // shared across lenses
  bool   SetMarkerHeightMm(double mm);        // rejects negative / non-finite
  // 0 clears the measurement and returns to the height the decomposition
  // implies, which is how an operator undoes a bad entry.
  bool   SetCameraHeightMm(int ch, double mm);

  // True once this lens has a usable H_marker. False when there is no H, when
  // the marker height is set but the decomposition refused, or when K is
  // missing. A marker height of 0 makes this equal to Available().
  bool MarkerPlaneReady(int ch) const;
  // Copy the derived H_marker, row-major. False when !MarkerPlaneReady().
  bool GetMarkerPlane(int ch, double h[9]) const;
  // Why the last derivation failed on this lens. "" when it succeeded.
  const char* MarkerPlaneReason(int ch) const;

  /**
   * The camera pose the floor homography implies: height above the floor and
   * the nadir (the floor point directly under the camera), in world mm.
   *
   * A sanity check with real diagnostic value — an installer who knows the
   * camera is 1.5 m up can see at a glance whether the decomposition is sane,
   * and a nadir in the wrong place is the tell-tale of a bad K.
   */
  bool CameraPose(int ch, double* height_mm, double* nadir_x_mm,
                  double* nadir_y_mm, const char** reason) const;

  /**
   * The same pose, but read off the MEASURED H_marker instead of the floor H —
   * height above the MARKER plane, nadir in the measured matrix's own world
   * frame (the driven rectangle, whose origin is that session's start corner).
   *
   * This is what lets an odometry run derive its own H_floor with no floor
   * calibration present at all, and it is also the only correct source for it:
   * the floor matrix's nadir lives in a different world frame, so borrowing it
   * scales about the wrong point. See the implementation comment.
   */
  bool MeasuredCameraPose(int ch, double* height_above_marker_mm,
                          double* nadir_x_mm, double* nadir_y_mm,
                          const char** reason) const;

  /**
   * Raw sensor pixel -> world mm THROUGH THE MARKER PLANE.
   *
   * This is what the pose path uses for a marker on the robot. Deliberately
   * does NOT fall back to the floor matrix when the marker plane is
   * unavailable: a configured marker height means the operator has said the
   * marker is off the floor, so floor coordinates are wrong by a known amount,
   * and silently supplying them is the exact "plausible but wrong" failure the
   * whole derivation exists to prevent. With the height at 0 the two matrices
   * are the same object and this is the floor path.
   */
  bool PixelToWorldMarker(int ch, float px, float py, double* wx, double* wy,
                          const char** reason = NULL) const;

  // Persist the shared marker height AND this lens's camera height. Two files,
  // because the two values have different scopes: marker_plane.txt is shared,
  // and Cz travels in that lens's homography_ch<N>.txt.
  bool SaveMarkerPlane(int ch);

  // --- registered markers --------------------------------------------------

  /**
   * 24, not cctv_app's 16.
   *
   * 16 sounds generous until you try to follow the placement advice, which is
   * 10-20 markers spread out to the edges of the frame. At 16 the operator is
   * choosing between "enough points" and "points near the edge", and the edge
   * is what pins down the perspective terms — a homography fitted from a
   * cluster in the middle extrapolates badly exactly where the robot drives.
   */
  static const int kMaxAnchors = 24;

  /**
   * Replace this lens's whole marker list. n == 0 empties it.
   *
   * Whole-list rather than add-one-at-a-time because a list assembled over
   * several commands has no moment at which it is known to be complete: a
   * dropped command leaves a shorter list that looks exactly like a deliberate
   * one, and it is the OPERATOR's tape measure that would get blamed for the
   * residual. One command carries the whole truth or none of it.
   *
   * Refused, leaving the previous list untouched, on: a duplicated id, two
   * markers within 1 mm of each other in world space (which is a pasted row,
   * not a measurement — no two floor markers have coincident centres), a
   * negative id, a non-finite coordinate, or more than kMaxAnchors points.
   * Validation completes before anything is committed, so a rejected command
   * cannot leave half a list behind.
   */
  bool SetAnchors(int ch, const Anchor* list, int n);

  int  AnchorCount(int ch) const;
  bool AnchorAt(int ch, int i, Anchor* out) const;

  // Persist / forget the marker list. Separate from Save(), because the markers
  // are measurements and H is derived from them: Clear()ing a bad matrix should
  // not also throw away the afternoon spent with a tape measure.
  bool SaveAnchors(int ch);

  // --- collection session + fit --------------------------------------------

  /**
   * 20 frames in which EVERY registered marker was seen, out of at most 200
   * offered.
   *
   * Averaging over frames rather than fitting the first good one is the whole
   * reason a session exists: a single frame's corner jitter goes straight into
   * H and stays there. cctv_app used 30; 20 is deliberate, because corner noise
   * falls as 1/sqrt(N) so the accuracy cost is about 18%, while the waiting is
   * linear and this lens collects at roughly 0.7 fps (full-frame detection,
   * four channels sharing one thread). 30 frames is 45 s of an operator
   * standing still, 20 is 30 s.
   *
   * The 200-frame ceiling is also the only way a session ends unsuccessfully:
   * there is no cancel command, deliberately, because the failure it guards
   * against is a marker that is covered — and the answer to that is to uncover
   * it, which the timeout gives ~5 minutes to do. CALIB_START on a lens that is
   * already collecting restarts its session, which is the way out if the
   * operator would rather start over.
   */
  static const int kFitTargetFrames = 20;
  static const int kFitMaxFrames    = 200;

  // Four points define a homography, so four is the arithmetic floor — it is
  // NOT a recommendation. A four-point fit passes through its own points
  // exactly and has nothing left over to disagree with, so it cannot be
  // checked; see kMinLooAnchors.
  static const int kMinFitAnchors = 4;
  // Leave one out and four must remain, so LOO needs five.
  static const int kMinLooAnchors = 5;
  // Below eight, each fold is fitting 4-6 points and is close enough to
  // determined that its residual stops meaning much. The numbers are still
  // produced, flagged as advisory rather than withheld — a weak measurement
  // that is labelled is more use than no measurement.
  static const int kLooAdvisoryBelow = 8;

  /**
   * What a collection session looks like from outside, for the progress UI.
   *
   * `missing_ids` is the field that earns this struct. A bar drawn from
   * good/target sits at 0% and says nothing when one marker is covered, which
   * is BY FAR the most common way a session fails — every frame is discarded,
   * silently, for up to five minutes. Naming the marker turns that into
   * something the operator fixes on the spot.
   *
   * `unusable_ids` is the same idea for a different cause: the marker WAS
   * detected, but its centre falls where this lens's undistortion does not
   * converge (the outer ~15% of the frame on the measured lens — see
   * PreparePixel). Without this list those markers look identical to markers
   * that are not there, and the operator goes looking for an obstruction that
   * does not exist. It is also the direct readout of the decision to keep
   * anchors inside the convergent middle of the frame.
   */
  struct FitProgress {
    bool active;
    int  good;       // frames where every registered marker was usable
    int  total;      // frames offered
    int  seen_n;
    int  seen_ids[kMaxAnchors];
    int  missing_n;      // registered, not detected in the last frame
    int  missing_ids[kMaxAnchors];
    int  unusable_n;     // detected, but not mappable into the fitting space
    int  unusable_ids[kMaxAnchors];
    // Why the last session ended, or why the last CALIB_START was refused.
    // Empty while one is running and before the first attempt.
    //
    // A buffer rather than a const char*: some of these messages are built with
    // the offending count in them, and pointing four channels at one shared
    // static formatting buffer would have each lens display whichever message
    // was written most recently. Contains only text this code wrote, so it
    // needs no JSON escaping.
    // 256, raised from 128 on 2026-08-05. The two longest rejection reasons
    // measure 142 B and 193 B in UTF-8 — Korean is three bytes a character, so
    // a sentence that reads short is not. Both were being cut. CopyUtf8() keeps
    // the cut legal whatever the size, but a buffer that fits the message means
    // there is no cut to keep legal.
    char last_result[256];
  };

  /** One point's disagreement with the fitted plane, in millimetres. */
  struct FitResidual {
    int    id;
    double in_mm;   // fitted on all N points, including this one
    double loo_mm;  // fitted on the other N-1; <0 when not computed
  };

  /**
   * The result of the last fit on this lens. RAM only, never persisted.
   *
   * Persisting residuals is the trap this whole block exists to avoid. A
   * residual is not a setting, it is an observation of one moment; the day
   * somebody kicks a marker the stored numbers are wrong, and a file cannot
   * look stale. Recomputed with every fit, reported through /status, and gone
   * when the app restarts — which is honest, because the evidence for them is
   * gone too.
   */
  struct FitResult {
    bool   have;
    int    n;
    bool   loo_valid;   // n >= kMinLooAnchors
    bool   advisory;    // n < kLooAdvisoryBelow — real numbers, weak evidence
    double rmse_in_mm;
    double rmse_loo_mm;
    double max_loo_mm;
    int    max_loo_id;
    FitResidual pt[kMaxAnchors];
  };

  /**
   * Open a collection session on this lens.
   *
   * Refused when fewer than kMinFitAnchors markers are registered, and when the
   * lens wants undistorted coordinates but has no K/dist — both would otherwise
   * be discovered 200 frames later, after the operator had waited out the
   * entire timeout.
   */
  bool StartFit(int ch);

  // Whether a session is open on this lens. Separate from Fit(ch).active
  // because the frame path asks this once per frame per channel, before it has
  // decided to do anything at all.
  bool FitCollecting(int ch) const;

  /**
   * Offer one frame's detected marker centres, in RAW sensor pixels.
   *
   * Only ids that are registered on this lens are looked at. A frame counts as
   * good only when every registered marker is present AND mappable; anything
   * less is discarded whole, because a homography averaged over frames with
   * different subsets of points is not an average of anything.
   *
   * Undistortion is applied here, per frame, rather than once to the averaged
   * centre at the end. It costs ~24 point transforms on a lens running at
   * 0.7 fps, and it buys the unusable_ids list above — waiting until the fit to
   * discover a marker sits in the divergent band means discovering it after the
   * full collection, with nothing on screen having said so.
   */
  void FeedFrame(int ch, const int* ids, const float* cx, const float* cy, int n);

  const FitProgress& Fit(int ch) const;
  const FitResult&   Residuals(int ch) const;

  // --- 주행 캘리 (오도메트리) ------------------------------------------------

  /**
   * 서버가 로봇을 사각형으로 몰면서 정지점마다 보내는 (월드좌표, 마커픽셀) 쌍을
   * 모아 H_marker 를 직접 피팅한다. 체커보드 방식과 달리 시차 보정 체인을 타지
   * 않는다 — 로봇 위 마커 자체를 찍으므로 결과가 처음부터 마커 평면이다.
   *
   * 세션 하나가 한 렌즈를 점유한다. K/dist 세션이나 정적 앵커 수집과 겹치면
   * 한 렌즈의 프레임을 두 수집기가 나눠 가져 양쪽 다 굶으므로 거부한다
   * (StartFit() 이 K 세션을 거부하는 것과 같은 이유).
   *
   * `wire_reason`: NULL이 아니면, 거부 사유가 wire 규격에 있는 이름(§7)일 때
   * 그 리터럴을 채운다 — "K/dist 없음"이면 "no_intrinsics", "K 세션 충돌"이면
   * "session_conflict". 나머지 두 사유(정적 앵커 수집 중 / raw 좌표 모드)는
   * wire 표에 대응 항목이 없어 건드리지 않는다 — 호출부가 기본값
   * "session_refused"로 채운다(서버팀 8/13 회신 §4: Qt가 no_intrinsics/
   * session_conflict 전용 문구를 갖고 있어 조작자에게 다음 행동을 알려준다).
   */
  bool StartOdom(int ch, const char** wire_reason = NULL);
  /**
   * 정지점 하나. `u`,`v` 는 **준비된 픽셀 공간**(PreparePixel 통과 후)이어야 한다.
   *
   * `point_index` 는 서버가 준 번호를 그대로 쓴다 — 잔차 표에서 "3번 지점"으로
   * 읽히고, 서버 로그와 대조된다. 같은 번호가 다시 오면 덮어쓴다(재시도).
   *
   * `closing` 이면 피팅에 넣지 않고 폐합 진단용으로만 보관한다. 그 지점의
   * 라벨은 출발점과 같아서, 넣으면 "같은 월드 좌표에 다른 픽셀"이라는 모순된
   * 대응점이 된다.
   */
  bool AddOdomPoint(int ch, int point_index, double wx_mm, double wy_mm,
                    double u, double v, bool closing);
  /**
   * 수집 종료 + 피팅. 성공하면 측정 H_marker 슬롯이 채워진다.
   *
   * 정적 앵커와 같은 최소제곱·LOO 를 쓴다(FitPoints/ComputeResiduals 공유).
   * 8점이면 LOO 가 유효하고 advisory 도 붙지 않는다.
   */
  bool FinishOdom(int ch);
  void AbortOdom(int ch);
  /**
   * 이 월드 좌표가 이미 등록된 지점과 같은가 — 복귀(폐합) 지점 판별용.
   *
   * 서버가 "이건 복귀 지점이다"를 따로 표시해 주지 않으므로 라벨로 알아낸다.
   * 계획서 §3 의 idx 8 은 idx 0 과 좌표가 같고, 그게 유일한 중복이다.
   * 1mm 안이면 같은 점으로 본다(서버가 계산해 보내는 값이라 정확히 일치할
   * 것이지만, 부동소수 왕복을 신뢰하지 않는다).
   */
  bool OdomHasLabel(int ch, double wx_mm, double wy_mm) const;
  bool OdomActive(int ch) const;
  int  OdomCount(int ch) const;
  /** 주행 캘리의 잔차. 정적 앵커의 Residuals() 와 같은 구조다. */
  const FitResult& OdomResiduals(int ch) const;

  /**
   * 폐합오차 — 복귀 지점의 픽셀을 **피팅된 H 로** 월드로 보내 출발점 라벨과 비교한다.
   *
   * 복귀 지점은 피팅에서 제외됐으므로 이건 out-of-sample 검증이다. LOO 와 같은
   * 성격이면서, 동시에 "한 바퀴 도는 동안 오도메트리가 얼마나 밀렸는가"를 mm 로 준다.
   *
   * 다만 이 값도 **균일 스케일 오차는 못 잡는다** — 네 변이 모두 같은 비율로
   * 짧으면 실제 궤적도 닫힌 직사각형이라 정확히 제자리로 돌아온다.
   * 그건 CompareMarkerPlanes() 로만 드러난다.
   */
  bool OdomClosureMm(int ch, double* err_mm) const;

  // --- 측정 H_marker 슬롯 ----------------------------------------------------

  bool MeasuredMarkerPlaneReady(int ch) const;
  bool GetMeasuredMarkerPlane(int ch, double h[9]) const;
  /** 측정 당시의 K 가 교체됐는가. 버리지 않고 표시만 한다. */
  bool MeasuredKStale(int ch) const;
  /** 측정 당시의 마커 높이에서 바뀌었는가. 바뀌면 그 행렬은 다른 평면의 것이다. */
  bool MeasuredHeightStale(int ch) const;
  /** 로봇 측위에 측정값을 쓸지. 기본 false(파생=체커보드 기준). */
  bool PreferMeasured(int ch) const;
  bool SetPreferMeasured(int ch, bool on);

  /**
   * 두 방식 비교 — 체커보드에서 파생한 H_marker 와 주행으로 측정한 H_marker.
   *
   * 행렬을 직접 비교할 수 없다. 월드 원점이 다르기 때문이다(체커보드는 폼보드
   * 좌하단, 주행은 로봇 출발 위치라 세션마다 이동). 그래서 화면 격자 픽셀을
   * 양쪽으로 월드에 보낸 뒤, 두 점집합 사이의 **닮음변환**을 최소제곱으로 맞추고
   * 회전·평행이동은 버린다. 남는 두 숫자가 답이다.
   *
   *   scale  1.000 이어야 한다. 벗어나면 줄자와 엔코더의 길이 기준 불일치 —
   *          잔차도 폐합오차도 못 잡는 바로 그 오차다.
   *   rmse   정렬 후에도 남는 형상 차이(전단·원근·왜곡 미보정).
   *
   * 어느 쪽이 틀렸는지는 알려주지 않는다. 불일치의 크기만 준다.
   */
  bool CompareMarkerPlanes(int ch, double* scale, double* rmse_mm,
                           double* max_mm, const char** reason) const;


  bool Save(int ch);   // persist one lens's H now
  /**
   * Forget one lens's H, in RAM and on disk.
   *
   * The same argument as IntrinsicsCalib::Clear(): a WRONG homography is worse
   * than none. With no H the page says so and reports pixels only; with a bad
   * one every world coordinate is confidently wrong. Without a way out, the
   * only cure for a bad injection is a good one, and there is not always one
   * to hand.
   */
  bool Clear(int ch);

  bool Persistable() const { return persist_ok_; }
  // Why the last Save/Set/Clear returned false. Never NULL.
  const char* FailReason() const { return fail_reason_; }

 private:
  // Everything here runs on the scheduler thread only — the same one as
  // ProcessRawVideo() and the HTTP handlers — so none of it locks.
  bool SaveOne(int ch);
  bool LoadOne(int ch);
  bool SaveAnchorsOne(int ch);
  bool LoadAnchorsOne(int ch);
  bool SaveMarkerHeightFile();
  bool LoadMarkerHeightFile();
  // Decompose this lens's floor H into rotation columns and translation, in
  // world mm. The shared front half of the marker plane and the camera pose —
  // one copy so the two can never disagree about the same matrix.
  bool Decompose(int ch, cv::Matx33d* K, cv::Vec3d* r1, cv::Vec3d* r2,
                 cv::Vec3d* r3, cv::Vec3d* t, const char** reason) const;
  // The plane-agnostic core of the above: any pixel->world H plus a K. Floor H
  // and measured H_marker both go through here so the two can never drift
  // apart in how they are decomposed.
  bool DecomposeMatrix(const cv::Matx33d& Hp2w, const cv::Matx33d& K,
                       cv::Vec3d* r1, cv::Vec3d* r2, cv::Vec3d* r3,
                       cv::Vec3d* t, const char** reason) const;
  // This lens's K as a matrix. Shared by both decomposition entry points.
  bool IntrinsicMatrix(int ch, cv::Matx33d* K, const char** reason) const;
  // Shared back half of WorldToPixel()/OdomPointRawPixel(): a pixel already
  // in this lens's FITTING space (undistorted, if that's the mode) -> raw
  // sensor pixel. One copy so the two entry points cannot drift apart on how
  // they undo the fitting-space transform.
  bool PreparedToRawPixel(int ch, double u, double v, float* px, float* py,
                          const char** reason) const;
  // Recompute Hm_[ch] from the current H, marker height and camera height.
  // Called from every path that changes any of the three, so the cache cannot
  // outlive its inputs.
  //
  // const, over mutable state, because SyncCalib() has to be able to run it
  // from inside the const readers — see there.
  void RefreshMarkerPlane(int ch) const;
  /**
   * Rebuild anything derived from K if K has changed under us, and record that
   * the stored H no longer matches the lens it was fitted against.
   *
   * Called at the top of every reader of the derived cache rather than from
   * the K commands themselves. A notification the command path has to send is
   * a notification a later command path can forget to send, and the symptom of
   * forgetting is silent: coordinates keep coming, correct in shape and wrong
   * in value. A version compared where the cache is USED cannot be forgotten,
   * because there is no way to read the cache without passing through it.
   *
   * Costs one unsigned comparison per call in the settled case, which is what
   * makes it affordable in the frame path. The rebuild itself runs once per
   * actual change.
   */
  void SyncCalib(int ch) const;
  void PathFor(const char* leaf, char* out, size_t out_size) const;
  // Turn the collected averages into H and its residuals, and close the
  // session. Called from FeedFrame() when the target is reached.
  bool FinishFit(int ch);
  // Least-squares (DLT) fit of the given index subset. `skip` is the index to
  // leave out, or -1 for all of them — the LOO folds and the final fit go
  // through the same path so that they cannot drift apart.
  bool FitSubset(int ch, int skip, cv::Mat* out) const;
  bool FitOdomSubset(int ch, int skip, cv::Mat* out) const;
  // 두 방식이 같은 최소제곱·같은 잔차 정의를 쓰도록 점 출처만 인자로 뺀 공통부.
  bool FitPoints(const double* sum_x, const double* sum_y, double inv,
                 const Anchor* dst_pts, int n, int skip, cv::Mat* out) const;
  void ComputeResiduals(const cv::Mat& H, const double* sum_x, const double* sum_y,
                        double inv, const Anchor* dst_pts, int n,
                        bool (HomographyMapper::*subset)(int, int, cv::Mat*) const,
                        int ch, FitResult* r) const;

  cv::Mat H_[kChannels];               // 3x3 CV_64F, pixels -> world mm
  bool    available_[kChannels];
  // Set at the end of FinishFit()/FinishOdom() on success only -- see
  // LastHomographySource()'s comment for why this exists at all.
  LastHSource last_h_source_[kChannels];
  // The space the STORED matrix was fitted in.
  bool    fitted_undistorted_[kChannels];
  // The space the NEXT fit will use, and the one PreparePixel() applies.
  // Equal to fitted_undistorted_ whenever a matrix exists, because
  // SetCoordMode() refuses to change it while one does — see there.
  bool    coord_undistorted_[kChannels];
  double  camera_z_mm_[kChannels];     // 0 = not measured

  // Marker height is SHARED: one robot carries one marker at one height. The
  // camera height above is per lens because tilt differs even in one housing.
  double  marker_height_mm_;
  // H_marker, cached. Rebuilt by RefreshMarkerPlane() whenever H, the marker
  // height or the camera height changes — the pose path maps two points per
  // marker per frame through this, and deriving it there would put a 3x3
  // inverse and a decomposition in the frame loop.
  //
  // mutable, along with everything else RefreshMarkerPlane() writes: the
  // rebuild is a cache-coherence step, not a change of state — the same inputs
  // produce the same matrix whether it runs now or ran a moment ago. Making it
  // mutable is what lets SyncCalib() sit inside the const readers, which is
  // where it has to be to be unforgettable.
  mutable cv::Mat Hm_[kChannels];
  mutable bool    hm_valid_[kChannels];
  // Why Hm_ is not valid, per lens. A literal; "" when it is.
  mutable const char* hm_reason_[kChannels];
  // The K generation the three cached values below were derived against, and
  // whether the stored H outlived it. See SyncCalib() and CalibStale().
  mutable unsigned k_gen_[kChannels];
  mutable bool     k_stale_[kChannels];
  // The camera centre the floor H implies, cached alongside Hm_ and produced by
  // the same decomposition. /status reads this for four lenses on every poll,
  // and re-deriving it there would be eight 3x3 inversions a second on the
  // thread the frame path shares — for numbers whose inputs only change when an
  // operator does something. Sharing one decomposition also stops the height
  // the page reports from drifting from the height the correction is scaled by.
  mutable bool        pose_ok_[kChannels];
  mutable cv::Vec3d   pose_c_[kChannels];       // world mm; [2] is the camera height
  mutable const char* pose_reason_[kChannels];  // literal; "" when pose_ok_

  // --- 측정된 H_marker (주행 캘리 산출물) ---------------------------------
  //
  // Hm_ 는 H_ (바닥) 에서 시차 보정으로 **파생**된 값이다. 주행 캘리는 로봇 위
  // 마커를 직접 찍으므로 결과가 처음부터 마커 평면이고, 그걸 Set() 에 넣으면
  // 카메라가 그것을 바닥으로 보고 시차를 또 걸어 이중 보정이 된다.
  // 그래서 별도 슬롯에 둔다.
  //
  // 둘을 동시에 들고 있어야 하는 이유가 하나 더 있다: 두 방식을 비교해
  // 스케일 불일치를 잡는 것(CompareMarkerPlanes)이 이 방식의 유일한 독립
  // 검증 수단인데, 한 슬롯이면 나중에 잰 쪽이 앞의 것을 덮어 비교 대상이 사라진다.
  bool   hmm_have_[kChannels];
  double hmm_[kChannels][9];
  bool   hmm_undistorted_[kChannels];
  // K 나 마커 높이가 바뀌면 측정값은 더 이상 유효하지 않다. 버리지는 않고
  // 표시만 한다 — CalibStale() 과 같은 판단이다(되돌리기 어려운 폐기 대신
  // "다시 확인하라"). 마커 높이 쪽이 따로 필요한 이유: 파생 Hm_ 는 높이를
  // 바꾸면 자동으로 따라오지만, 측정값에는 잴 당시의 높이가 이미 녹아 있다.
  // mutable: SyncCalib() 이 const 인데 거기서 상함을 표시한다(k_stale_ 과 동일).
  unsigned hmm_k_gen_[kChannels];
  mutable bool hmm_k_stale_[kChannels];
  double   hmm_height_mm_[kChannels];   // 측정 당시의 마커 높이
  // 로봇 측위에 측정값을 쓸지 파생값을 쓸지. 기본은 파생(체커보드) — 주행
  // 방식은 아직 스케일 오차를 검증할 수단이 없으므로, 검증된 쪽을 기본으로
  // 두고 운영자가 의도적으로 전환하게 한다.
  bool   hmm_preferred_[kChannels];

  // Fixed arrays, not vectors: ~2.3 KB total, and it keeps the whole mapper
  // allocation-free, which matters because PixelToWorld() runs in the frame
  // path.
  Anchor  anchors_[kChannels][kMaxAnchors];
  int     anchor_n_[kChannels];

  // --- 주행 캘리(오도메트리) 수집 버퍼 (2026-08-12) ------------------------
  //
  // anchors_ 를 재사용하지 않는 이유가 둘이다.
  //   1. SetAnchors() 는 중복 id 를 거부한다. 주행 방식은 같은 마커 하나가
  //      9개 지점에서 찍히므로 id 가 전부 같다. 그 방어는 정적 앵커에서는
  //      옳으므로 풀지 않는다.
  //   2. 앵커는 줄자로 잰 측정값이다. 주행 세션이 그 위에 덮어쓰면 오후 내내
  //      잰 값이 사라진다.
  // 여기서 id 자리에는 point_index 를 넣는다 — 잔차 표가 "3번 지점이 튄다"로
  // 그대로 읽힌다.
  Anchor odom_[kChannels][kMaxAnchors];
  int    odom_n_[kChannels];
  // 지점별 **평균** 픽셀(합이 아니다). 지점마다 표본 프레임 수가 달라 공통
  // divisor 가 성립하지 않으므로 수집 단계에서 미리 나눈다 — 그래야
  // FitPoints(inv=1.0) 으로 기존 최소제곱을 그대로 쓸 수 있다.
  double odom_sx_[kChannels][kMaxAnchors];
  double odom_sy_[kChannels][kMaxAnchors];
  bool   odom_active_[kChannels];
  // 폐합 진단용 복귀 지점. 피팅에는 넣지 않는다 — 라벨이 출발점과 겹쳐
  // "같은 월드 좌표에 다른 픽셀"이 되어 모순된 대응점이 되기 때문이다.
  bool   odom_close_have_[kChannels];
  double odom_close_u_[kChannels], odom_close_v_[kChannels];
  double odom_close_wx_[kChannels], odom_close_wy_[kChannels];  // 복귀 지점의 라벨
  FitResult odom_result_[kChannels];

  FitProgress fit_[kChannels];
  FitResult   result_[kChannels];
  // Running sums of the prepared (fitting-space) marker centres over the good
  // frames, indexed the same way as anchors_. Divided by fit_[ch].good at the
  // end. Sums rather than a per-frame array because 20 frames x 24 markers of
  // history is never looked at — only the mean is.
  double  sum_x_[kChannels][kMaxAnchors];
  double  sum_y_[kChannels][kMaxAnchors];

  const IntrinsicsCalib* calib_;       // not owned; outlives this object
  bool        persist_ok_;
  char        persist_dir_[256];
  const char* fail_reason_;
};
