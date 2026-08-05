#pragma once

#include <stddef.h>

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
  // So: RequestCapture() arms every open session, and one press of the capture
  // button banks that board pose into every lens that can see it well enough.
  // Each lens judges the view against its own quality gates and its own
  // history, so the same pose can be accepted by one lens and refused by
  // another as too far, too oblique or too close to a view it already has —
  // which is correct, and is the whole point of judging per lens.
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
   * TakePendingCapture() as each lens's next frame lands. Arming them together
   * is what makes the captures describe the same board pose — the frames differ
   * by at most one frame interval, and the operator is asked to hold still
   * anyway.
   *
   * False only when no session is open at all. A lens that cannot see the board
   * is not an error here; it reports its own refusal when its frame arrives.
   */
  bool RequestCapture(const char** reason);
  bool CapturePending(int ch) const;
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
  // Run a board probe if one is due (throttled by CALIB_PROBE_MS). Per lens,
  // because each open session has its own viewfinder to keep current — a shared
  // probe would show the operator whichever lens happened to be last.
  void ProbeIfDue(int ch, const cv::Mat& gray, long now_ms);

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
  // Externally-computed values, applied with no capture session. Refuses
  // obviously broken numbers so a typo cannot silently poison a lens.
  bool LoadValues(int ch, double fx, double fy, double cx, double cy, const double dist[5]);
  bool Save(int ch);            // persist one lens's K/dist now
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
                   std::vector<int>& ids);
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
    std::vector<cv::Point2f> probe_corners;
    std::vector<int>         probe_ids;
    int  probe_total;
    long probe_ms;
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
