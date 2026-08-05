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
    char last_result[128];
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
  // Recompute Hm_[ch] from the current H, marker height and camera height.
  // Called from every path that changes any of the three, so the cache cannot
  // outlive its inputs.
  void RefreshMarkerPlane(int ch);
  void PathFor(const char* leaf, char* out, size_t out_size) const;
  // Turn the collected averages into H and its residuals, and close the
  // session. Called from FeedFrame() when the target is reached.
  bool FinishFit(int ch);
  // Least-squares (DLT) fit of the given index subset. `skip` is the index to
  // leave out, or -1 for all of them — the LOO folds and the final fit go
  // through the same path so that they cannot drift apart.
  bool FitSubset(int ch, int skip, cv::Mat* out) const;

  cv::Mat H_[kChannels];               // 3x3 CV_64F, pixels -> world mm
  bool    available_[kChannels];
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
  cv::Mat Hm_[kChannels];
  bool    hm_valid_[kChannels];
  // Why Hm_ is not valid, per lens. A literal; "" when it is.
  const char* hm_reason_[kChannels];
  // The camera centre the floor H implies, cached alongside Hm_ and produced by
  // the same decomposition. /status reads this for four lenses on every poll,
  // and re-deriving it there would be eight 3x3 inversions a second on the
  // thread the frame path shares — for numbers whose inputs only change when an
  // operator does something. Sharing one decomposition also stops the height
  // the page reports from drifting from the height the correction is scaled by.
  bool        pose_ok_[kChannels];
  cv::Vec3d   pose_c_[kChannels];       // world mm; [2] is the camera height
  const char* pose_reason_[kChannels];  // literal; "" when pose_ok_

  // Fixed arrays, not vectors: ~2.3 KB total, and it keeps the whole mapper
  // allocation-free, which matters because PixelToWorld() runs in the frame
  // path.
  Anchor  anchors_[kChannels][kMaxAnchors];
  int     anchor_n_[kChannels];

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
