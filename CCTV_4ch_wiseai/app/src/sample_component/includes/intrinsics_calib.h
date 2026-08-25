#pragma once

#include <stddef.h>

#include <map>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

/**
 * On-camera intrinsics (K, dist) calibration from a printed ChArUco board,
 * for a FOUR-lens camera.
 *
 * Ported from cctv_app/src/intrinsics_calibrator.cpp. The maths, the quality
 * gates and the file format are unchanged; what changed is the shape:
 *
 *   - cctv_app was one lens, so it was a file of free functions over globals.
 *     Here it is a class, because "which lens" now has to be a parameter of
 *     everything that touches a result.
 *
 *   - K/dist is PER LENS (four of them, four files). Each lens has its own
 *     optics; a shared K would be wrong on three of them.
 *
 *   - The board description is SHARED by all four. It describes one physical
 *     printed sheet that gets held in front of each lens in turn, so four
 *     independent copies could only ever disagree with the paper.
 *
 *   - Sessions are PER LENS and any number may be open at once (2026-08-05,
 *     revised). The original port allowed exactly one, reasoning that a session
 *     means a person standing in front of that lens holding a board. That is
 *     true and still is — but on a camera whose four lenses overlap, one board
 *     pose is visible to several of them, and the views it made for the others
 *     were being discarded because no session existed to receive them. See the
 *     session block below for what this does and does not buy.
 *
 * Deliberately NOT ported (agreed 2026-08-04): view-image upload to the RPi
 * (CALIB_K_UPLOAD), the LDC residual-distortion check, named K/dist profiles,
 * and the two hardcoded focal-length preset buttons. The first two are the
 * RPi's job or a separate tool; profiles and presets were single-lens
 * conveniences that four lenses turn into four times the bookkeeping for no
 * gain — with one K per lens there is nothing to switch between.
 *
 * Revisited 2026-08-05: the profiles argument above was right about switching
 * and wrong about what the operator needs. Nobody wants to CHOOSE between
 * calibrations, but they very much need to UNDO one — a session can succeed,
 * report a good RMS, and still produce a K that is worse than the one it
 * replaced (a board held still fits its own near-identical views beautifully).
 * So there is exactly one step of history per lens, and no names, no list, no
 * deletes: see HasPrevious()/Revert().
 */

/**
 * Runtime ChArUco board description.
 *
 * Lengths are millimetres. Camera intrinsics are scale-independent, but the
 * square/marker ratio and marker dictionary MUST exactly match the print.
 * The outer margins do not enter OpenCV's geometry; they are kept and reported
 * so the operator can verify the physical sheet.
 */
struct CharucoBoardConfig {
  int   squares_x;
  int   squares_y;
  float square_length_mm;
  float marker_length_mm;
  int   dictionary_id;
  float outer_margin_x_mm;
  float outer_margin_y_mm;
};

/** Quality measurements for the most recent capture request. */
struct CalibViewQuality {
  int         corners_found;
  int         corners_total;
  double      coverage_ratio;
  double      sharpness;
  double      mean_move_px;  // -1 = no comparable earlier view
  const char* reason;
};

enum CalibState {
  CS_IDLE,             // no session open
  CS_COLLECTING,       // session open, waiting for captures
  CS_CAPTURED,         // last capture was accepted and stored
  CS_CAPTURE_REJECTED, // last capture failed a gate; not stored
  CS_DONE_OK,          // compute succeeded (RMS within limit)
  CS_DONE_FAIL         // compute failed (too few views / bad RMS / threw)
};

class IntrinsicsCalib {
 public:
  static const int kChannels = 4;

  IntrinsicsCalib();

  // Load the persisted board config and every lens's persisted K/dist.
  // Also decides whether PERSIST_DIR is writable at all, which is why it must
  // run before anything offers the operator a Save button.
  void Init();

  // --- board (shared by all lenses) --------------------------------------
  CharucoBoardConfig Board() const { return board_; }
  // RAM only, and refused while a session is open — changing the board
  // mid-session would mix two geometries into one fit. Returns false with
  // *reason set when the numbers cannot describe a real board.
  bool SetBoard(const CharucoBoardConfig& cfg, const char** reason);
  bool SaveBoard();  // writes PERSIST_DIR, separate operator action

  // --- sessions (one per lens, any number open at once) -------------------
  //
  // Four independent sessions, not one pinned to a lens.
  //
  // The old design allowed exactly one because a person has to stand in front
  // of the lens holding a board — with one board and one operator there was
  // nothing a second session could collect. That reasoning has a hole in it on
  // a four-sensor camera whose lenses overlap: ONE board pose is visible to
  // several lenses at once, and the views it makes for the others were being
  // thrown away because no session existed to receive them.
  //
  // So: RequestCapture(-1) arms every open session, and one press of the
  // capture button banks that board pose into every lens that can see it well
  // enough. Each lens judges the view against its own quality gates and its own
  // history, so the same pose can be accepted by one lens and refused by
  // another as too far, too oblique or too close to a view it already has —
  // which is correct, and is the whole point of judging per lens.
  //
  // That is opt-IN as of 2026-08-11, not the default. The hole in the reasoning
  // above is that it assumes every open session is one somebody MEANT to open.
  // A session left open on a lens nobody is calibrating receives whatever scrap
  // of board that lens can see from every press, silently, and those views are
  // what its K gets computed from. The dashboard now sends
  // CALIB_K_CAPTURE <ch>; the all-lenses form is still there for the
  // overlapping-lens case, but somebody has to ask for it.
  //
  // What this does NOT do is remove the walk. K/dist needs the board at many
  // angles across the WHOLE of each lens's frame, including its corners and
  // close-ups; the region where all four lenses overlap is a fraction of each
  // frame (measured 21-28% of width, all central). A calibration collected only
  // there produces a K fitted from the middle of the image and a distortion
  // model that extrapolates badly at the edges — which is the likeliest
  // explanation for the k3 = -7.98 already measured on ch1. The operator still
  // walks each lens's frame; the other lenses simply bank whatever they get for
  // free while that happens, and the per-lens progress says which still needs
  // work.
  bool Start(int ch, const char** reason);
  void Stop(int ch);                            // abandon without computing
  bool Collecting(int ch) const;
  // Any lens collecting. The board probe and the frame-path branch use this.
  bool AnyCollecting() const;

  /**
   * Arm a capture on EVERY open session.
   *
   * The command arrives on an HTTP/TCP path with no frame in hand, so the
   * capture is deferred rather than faked: ProcessRawVideo calls
   * TakePendingCapture() as each lens's frames land, and keeps calling it —
   * capture_pending stays set across a rejection — until either a frame is
   * accepted or CALIB_CAPTURE_BURST_MS runs out on that (shared) deadline.
   * Arming them together with one shared deadline is what makes the captures
   * describe the same board pose: every lens's retry burst ends at the same
   * instant, and the operator is asked to hold still through it anyway.
   *
   * False only when no session is open at all. A lens that cannot see the board
   * is not an error here; it reports its own refusal when its frame arrives.
   *
   * ch >= 0 arms ONLY that lens (2026-08-11). The all-lenses form above is
   * right when the lenses overlap and the operator means to fill several at
   * once, and wrong the rest of the time: a session left open on a lens nobody
   * is calibrating banks whatever scrap of board it can see from every press,
   * and those views go into that lens's K. It happened here — CH1 ended up
   * with fx=8723 (a 17 degree lens) built from board fragments in the frame's
   * bottom-left corner while the operator was working on CH2.
   *
   * Same shape as CALIB_K_STOP: bare = every open session, with <ch> = one.
   */
  bool RequestCapture(int ch, const char** reason);
  bool CapturePending(int ch) const;
  // Called on every incoming frame while CapturePending(ch) — see
  // RequestCapture()'s doc for why that can be more than once per press.
  CalibState TakePendingCapture(int ch, const cv::Mat& gray);

  bool UndoLast(int ch);
  CalibState Compute(int ch);

  // --- session parameters -------------------------------------------------
  void SetParams(int target_views, double rms_limit);
  void SetGates(bool on) { gates_ = on; }
  bool Gates() const { return gates_; }
  int  TargetViews() const { return target_views_; }
  double RmsLimit() const { return rms_limit_; }

  // --- session readout ----------------------------------------------------
  // The SESSION's state. A rejected capture is not a session state and must
  // never appear here: Collecting() is what the frame path and every button
  // test against, so folding "that last capture was blurred" into it ended the
  // whole session on the first rejection and made the operator restart from
  // zero views. Observed on .13, 2026-08-04, before this split existed.
  CalibState  State(int ch) const;
  // How the most recent capture attempt went (CS_IDLE = none this session).
  CalibState  LastCapture(int ch) const;
  int         Views(int ch) const;
  int         PrunedViews(int ch) const;
  double      Rms(int ch) const;
  const char* FailReason() const { return fail_reason_; }
  const CalibViewQuality& LastQuality(int ch) const;
  // Put a line in front of the operator. Used by the command layer for the
  // outcomes this class returns as a bare bool — a refused board or a refused
  // START is otherwise indistinguishable from an applied one, because the form
  // just snaps back to the old numbers on the next refresh with nothing said.
  void NoteMessage(const char* text) { SetReason(text); }

  // Corners seen by the most recent board probe on the active lens, in
  // full-frame pixels, so the page can draw where the board actually is. The
  // calibration UI shows coordinates and no photo, so this IS the viewfinder.
  const std::vector<cv::Point2f>& ProbeCorners(int ch) const;
  // Board id of each probe corner. Without these a scatter of dots cannot be
  // told apart from a correct grid that happens to be viewed at an angle — the
  // id says WHICH corner of the board a dot claims to be, so a misdetection
  // shows up as an id out of sequence rather than as a dot that looks fine.
  const std::vector<int>& ProbeIds(int ch) const;
  long ProbeAgeMs(int ch, long now_ms) const;
  // Marker-layout fit for this lens's last probe: as configured, and with
  // squares_x/squares_y swapped. -1 = too few markers to judge. See
  // Session::board_fit_rms for why this exists and why corners cannot answer it.
  double BoardFitRms(int ch) const {
    return ValidCh(ch) ? sess_[ch].board_fit_rms : -1.0;
  }
  double BoardFitRmsTransposed(int ch) const {
    return ValidCh(ch) ? sess_[ch].board_fit_rms_t : -1.0;
  }
  // Run a board probe if one is due (throttled by CALIB_PROBE_MS). Per lens,
  // because each open session has its own viewfinder to keep current — a shared
  // probe would show the operator whichever lens happened to be last.
  //
  // hold_ms: how long a corner/marker not seen THIS attempt still counts, per
  // Session::RecentPoint's doc comment. Passed in rather than a constant so
  // the one runtime-adjustable value (SampleComponent::marker_hold_ms_, "HOLD_MS
  // <ms>") is the single source of truth an operator tunes — this class does
  // not keep its own copy to drift out of sync with it.
  // Returns true when a probe actually ran this call (false if invalid ch, no
  // session collecting, or still inside CALIB_PROBE_MS of the last one) — so
  // a caller that wants to push ProbeCorners()/ProbeIds() over the wire can
  // do it exactly when the held set changed, at the same CALIB_PROBE_MS
  // cadence, instead of re-sending an unchanged view every frame.
  bool ProbeIfDue(int ch, const cv::Mat& gray, long now_ms, long hold_ms);

  // --- results, per lens --------------------------------------------------
  bool Available(int ch) const;
  bool Get(int ch, double* fx, double* fy, double* cx, double* cy, double dist[5]) const;
  // The same values as Get(), in the form OpenCV wants them.
  //
  // Exposed because undistorting a point needs K and dist as cv::Mat, and the
  // only other way to get there is to rebuild both matrices from the doubles
  // on every call — which is what cctv_app did, at one lens and once per
  // calibration. Here the caller is the per-marker path in every frame. These
  // Mats already exist and already live exactly as long as the values do, so
  // handing out a reference IS the cache, with no second copy to invalidate.
  //
  // Empty when !Available(ch); callers must check that first. Out-of-range
  // channels return an empty Mat rather than reading past the array, because
  // these are reached from a command parser and `ch` comes off the network.
  const cv::Mat& KMat(int ch) const {
    static const cv::Mat kNone;
    return (ch >= 0 && ch < kChannels) ? K_[ch] : kNone;
  }
  const cv::Mat& DistMat(int ch) const {
    static const cv::Mat kNone;
    return (ch >= 0 && ch < kChannels) ? dist_[ch] : kNone;
  }
  /**
   * How many times this lens's K/dist has been replaced or dropped.
   *
   * Handing out a reference to K_ (above) makes this object the single copy of
   * the numbers, but it does NOT make anything DERIVED from them notice when
   * they change — and HomographyMapper derives two things and caches both:
   * H_marker, and the camera pose the floor H implies. Both were being rebuilt
   * only when H or one of the heights changed, so a recalibration left them
   * describing the old lens while the mapping path went on using them every
   * frame. Nothing said so; the coordinates stayed well-formed.
   *
   * A counter rather than a callback because the two objects are already
   * one-directional (the mapper holds the calib, never the reverse), and
   * because a reader that compares a number it kept cannot forget to register
   * — the check is in the path that uses the cache. Same device as
   * board_generation_, which invalidates the cached ChArUco detector.
   *
   * Wraps at 2^32 changes, which is not reachable by an operator pressing a
   * button, and equality is the only test made of it.
   */
  unsigned KGeneration(int ch) const {
    return (ch >= 0 && ch < kChannels) ? k_generation_[ch] : 0u;
  }
  // Externally-computed values, applied with no capture session. Refuses
  // obviously broken numbers so a typo cannot silently poison a lens.
  bool LoadValues(int ch, double fx, double fy, double cx, double cy, const double dist[5]);
  bool Save(int ch);            // persist one lens's K/dist now

  // --- the previous saved value, kept automatically -----------------------
  //
  // Save() rotates: whatever was on disk for this lens becomes the "previous"
  // file before the new value takes its place. One step back, not a library of
  // named profiles.
  //
  // The failure this exists for is not "I want to choose between calibrations"
  // — it is "the one I just took is worse and the good one is gone". Compute()
  // overwrites RAM the instant it succeeds, and success is not the same as
  // correct: a board held still for every view fits its own five nearly
  // identical images beautifully (RMS 0.36 px, measured on .13 2026-08-05) and
  // is still unusable. By the time that is visible, the previous value has
  // already been replaced.
  //
  // Named profiles were considered and rejected. Four lenses times a list of
  // names is a filing system, and the snapshots in tools/calib_backup.sh
  // already hold complete four-lens sets off-camera — a second, per-lens
  // history on the camera would be a second place claiming to know what the
  // right calibration is.
  bool HasPrevious(int ch) const;
  bool GetPrevious(int ch, double* fx, double* fy, double* cx, double* cy,
                   double dist[5]) const;
  // Put the previous value back, and make the current one the previous. A
  // SWAP, not a pop: reverting by mistake is then undone by reverting again,
  // and there is never a moment where one of the two values has been thrown
  // away. Applies to RAM and disk together, so it survives a restart.
  bool Revert(int ch);
  // Forget one lens's K/dist, in RAM and on disk.
  //
  // Needed because a WRONG calibration is worse than none: with no K the page
  // says so and draws raw corners only, while a bad one silently undistorts
  // into plausible nonsense. Without this the only way out of a bad load was a
  // better load — and there is not always one to hand.
  bool Clear(int ch);
  bool Persistable() const { return persist_ok_; }
  // Where files actually landed. Discovered at Init() rather than compiled in:
  // the path cctv_app used (/mnt/opensdk/storage/<app>) does not exist for
  // this app, and a Save button that reports success into a directory nothing
  // can write is worse than no button — the operator walks away believing a
  // calibration survives the next restart.
  const char* PersistDir() const { return persist_dir_; }
  // The working directory the app was started in, reported by /status purely
  // so that "where can this thing write" is answerable without an SSH session.
  const char* Cwd() const { return cwd_; }

  // Every directory Init() probed, and how it went. Reported so that "where can
  // this app write, and would a location survive the app being deleted" is
  // answerable from /status — the camera refuses SSH, so /status is the only
  // window onto its filesystem there is.
  struct PersistCandidate {
    char path[96];
    bool exists;
    bool writable;
    bool selectable;  // may be used for storage; the rest are reported only
    bool chosen;
  };
  int CandidateCount() const { return cand_n_; }
  const PersistCandidate& Candidate(int i) const { return cand_[i]; }

 private:
  // Everything below runs on the scheduler thread only — same thread as
  // ProcessRawVideo and the HTTP handlers, so none of it locks.
  bool DetectBoard(const cv::Mat& gray, std::vector<cv::Point2f>& corners,
                   std::vector<int>& ids,
                   std::vector<cv::Point2f>* marker_centers = NULL,
                   std::vector<int>* marker_ids = NULL);
  // RMS reprojection error of the detected marker centres against the layout
  // OpenCV would produce for a squares_x by squares_y ChArUco board, fitted by
  // homography so board tilt does not count against it. Both chessboard
  // parities are tried and the better one wins — which one carries the markers
  // is a property of the board generator, not something worth asserting here.
  //
  // -1 when fewer than 5 markers are visible: four points determine a
  // homography exactly, so the residual would be 0 for any layout and the
  // comparison would be meaningless rather than merely noisy.
  static double MarkerLayoutRms(const std::vector<cv::Point2f>& centers,
                                const std::vector<int>& ids, int squares_x,
                                int squares_y);
  CalibState CaptureView(int ch, const cv::Mat& gray);
  double MeanCommonCornerMove(int ch, const std::vector<cv::Point2f>& corners,
                              const std::vector<int>& ids) const;
  bool PoseDiversityOk(int ch, char* reason, size_t reason_size) const;
  void BuildPoints(int ch, const std::vector<size_t>& active,
                   std::vector<std::vector<cv::Point3f> >& obj_pts,
                   std::vector<std::vector<cv::Point2f> >& img_pts) const;
  void SetReason(const char* reason);
  // The same, for one lens's session. Also updates the shared reason so the
  // command layer's single-line reply still says something useful.
  void SetSessionReason(int ch, const char* reason);
  bool SaveOne(int ch);
  bool LoadOne(int ch);
  // File I/O for one K/dist pair, by leaf name, so the current file and the
  // previous one go through exactly the same reader and writer. Two copies of
  // this format is how the two would come to disagree.
  bool WriteValues(const char* leaf, int ch, const cv::Mat& K, const cv::Mat& dist) const;
  bool ReadValues(const char* leaf, int ch, cv::Mat* K, cv::Mat* dist) const;
  static void MainLeaf(int ch, char* out, size_t n);
  static void PrevLeaf(int ch, char* out, size_t n);
  bool ValidCh(int ch) const { return ch >= 0 && ch < kChannels; }

  CharucoBoardConfig board_;
  unsigned board_generation_;  // bumped on every change; invalidates the cached detector

  // Policy, shared: one printed board, one operator, one standard of "good
  // enough". Only the collected VIEWS and the RESULT are per lens.
  bool       gates_;
  int        target_views_;
  double     rms_limit_;

  /**
   * One lens's collection session. Four of these, all independent.
   *
   * Everything here used to be a single set of members with an active_ch_
   * beside it. Grouping it into a struct is not tidiness: it is what makes
   * "four sessions" a compile-time fact rather than a convention. With loose
   * members, adding a fifth piece of session state and forgetting to make it
   * per-lens gives two lenses one value, and the symptom is a view banked
   * against the wrong camera — a calibration that is wrong without ever failing.
   */
  struct Session {
    CalibState state;
    CalibState last_capture;
    bool       capture_pending;
    // Monotonic-clock deadline (see monotonic_ms()) for the CURRENT capture
    // request. While capture_pending is set, TakePendingCapture() retries on
    // every incoming frame until either one succeeds or this passes — see
    // CALIB_CAPTURE_BURST_MS. Meaningless once capture_pending is false.
    long long  capture_deadline_ms;

    std::vector<std::vector<cv::Point2f> > views_corners;
    std::vector<std::vector<int> >         views_ids;
    std::vector<cv::Point2f>               view_centers;
    std::vector<double>                    view_coverage;
    cv::Size    image_size;
    double      rms;
    int         pruned_views;
    CalibViewQuality last_quality;
    long long   last_capture_ms;
    // Why THIS lens accepted or refused its last capture.
    //
    // Per session, not shared, and that is the point of the whole design: one
    // board pose is judged four times, and "ch0 승인됨 / ch2 너무 멀다" is the
    // answer. A shared buffer would collapse those into whichever lens's frame
    // arrived last, which is the one thing the operator cannot act on.
    char        reason_buf[192];

    // Each lens keeps its own viewfinder. Sharing one meant the page showed
    // whichever lens ran last, which with several sessions open is nobody's.
    //
    // probe_corners/probe_ids are NOT "whatever this one probe attempt saw" —
    // see recent_corners/recent_markers below and ProbeIfDue()'s doc comment.
    std::vector<cv::Point2f> probe_corners;
    std::vector<int>         probe_ids;
    int  probe_total;
    long probe_ms;       // when the last probe ATTEMPT ran, whether or not it saw the board

    /**
     * One board point (interior corner or raw marker centre) last seen, and
     * when.
     *
     * At the print size this board's markers work out to (see
     * docs/... 2026-08-10: ~28px a side at the distance this got measured),
     * a board held rock-still still has individual markers blink in and out
     * from one probe to the next on nothing but ordinary sensor/exposure
     * noise — normal behaviour right at a detector's resolution floor, not a
     * bug. Judging the aiming view by "what did THIS ONE attempt see" makes
     * that noise fully visible as flicker; judging it by "what has been seen
     * in the last MARKER_HOLD_MS" does not, because a real, still-present
     * corner keeps reappearing across attempts far more often than it drops
     * out for that whole window.
     */
    struct RecentPoint { cv::Point2f pt; long seen_ms; };
    // Keyed by chessboard-corner id / ArUco marker id. Maps, not arrays: the
    // key space is the BOARD's (up to (100-1)*(100-1) — see valid_board()),
    // not a small fixed cap, and this only ever runs at CALIB_PROBE_MS
    // cadence, nowhere near the per-frame path, so the allocation is fine.
    std::map<int, RecentPoint> recent_corners;  // -> probe_corners/probe_ids
    std::map<int, RecentPoint> recent_markers;  // -> board_fit_rms(_t)

    // How well the MARKERS fit the configured layout, and how well they fit its
    // transpose. Negative = not measurable (too few markers seen).
    //
    // This exists because 5x7 and 7x5 are indistinguishable by every count on
    // the page — same 17 markers, same 24 interior corners — and, as of
    // 2026-08-05, also by the corner-grid check the page already draws. A wrong
    // orientation was diagnosed as right and set, twice, in one afternoon; two
    // physical boards of different layouts were in play and nothing on screen
    // could say which one the lens was looking at.
    //
    // Marker CENTRES settle it because they never pass through
    // interpolateCornersCharuco: they are where the detector found black
    // squares, so a wrong board description cannot move them. Measured on .13
    // with the 5x7 board in view: 2.07 px as configured against 350.20 px
    // transposed, on two lenses independently. The two answers are never close
    // enough to argue about.
    double board_fit_rms;      // configured layout
    double board_fit_rms_t;    // sx and sy swapped
  };
  Session sess_[kChannels];

  // Shared: whichever action last had something to say. Sessions do not each
  // need one — the operator drives one button at a time, and the reply belongs
  // to that press.
  const char* fail_reason_;
  char        reason_buf_[192];

  cv::Mat K_[kChannels];
  cv::Mat dist_[kChannels];
  bool    available_[kChannels];
  // Bumped by InstallK()/DropK() and nowhere else. See KGeneration().
  unsigned k_generation_[kChannels];
  // The ONLY two writers of K_/dist_/available_.
  //
  // There were five (Compute, Revert, LoadValues, LoadOne, Clear), which is
  // four too many for a counter that has to be bumped on every one of them:
  // the sixth write added later would compile, run, and quietly leave every
  // derived value stale. Funnelling them means the next K path cannot fail to
  // announce itself, because there is no way to set the value except through
  // the announcement.
  void InstallK(int ch, const cv::Mat& K, const cv::Mat& dist);
  void DropK(int ch);
  bool    persist_ok_;
  char    persist_dir_[256];
  static const int kMaxCandidates = 24;
  PersistCandidate cand_[kMaxCandidates];
  int              cand_n_;
  char    cwd_[256];
  // Builds a path inside persist_dir_. All file access goes through this so
  // the discovered directory is used everywhere, with no compiled-in path left
  // to disagree with it.
  void PathFor(const char* leaf, char* out, size_t out_size) const;
};
