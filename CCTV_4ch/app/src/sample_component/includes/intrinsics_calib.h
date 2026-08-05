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
 *   - Exactly ONE capture session may be open at a time, and it is pinned to
 *     the lens that opened it. Not a limitation to work around later: a
 *     session means a person standing in front of that lens holding a board,
 *     and a full ChArUco scan costs ~200 ms on a thread all four lenses share.
 *     Two concurrent sessions would be two people, or one person calibrating a
 *     lens they are not pointing the board at.
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

  // --- session (one at a time, pinned to a lens) --------------------------
  bool Start(int ch, const char** reason);
  void Stop();                                  // abandon without computing
  int  ActiveChannel() const { return active_ch_; }
  bool Collecting() const { return state_ == CS_COLLECTING; }

  // Ask for the next frame of the active lens to be captured. The command
  // arrives on an HTTP/TCP path that has no frame in hand, so the capture is
  // deferred rather than faked: ProcessRawVideo calls TakePendingCapture()
  // when that lens's next frame lands.
  bool RequestCapture(const char** reason);
  bool CapturePending() const { return capture_pending_; }
  CalibState TakePendingCapture(const cv::Mat& gray);

  bool UndoLast();
  CalibState Compute();

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
  CalibState  State() const { return state_; }
  // How the most recent capture attempt went (CS_IDLE = none this session).
  CalibState  LastCapture() const { return last_capture_; }
  int         Views() const { return (int)views_corners_.size(); }
  int         PrunedViews() const { return pruned_views_; }
  double      Rms() const { return rms_; }
  const char* FailReason() const { return fail_reason_; }
  const CalibViewQuality& LastQuality() const { return last_quality_; }
  // Put a line in front of the operator. Used by the command layer for the
  // outcomes this class returns as a bare bool — a refused board or a refused
  // START is otherwise indistinguishable from an applied one, because the form
  // just snaps back to the old numbers on the next refresh with nothing said.
  void NoteMessage(const char* text) { SetReason(text); }

  // Corners seen by the most recent board probe on the active lens, in
  // full-frame pixels, so the page can draw where the board actually is. The
  // calibration UI shows coordinates and no photo, so this IS the viewfinder.
  const std::vector<cv::Point2f>& ProbeCorners() const { return probe_corners_; }
  // Board id of each probe corner. Without these a scatter of dots cannot be
  // told apart from a correct grid that happens to be viewed at an angle — the
  // id says WHICH corner of the board a dot claims to be, so a misdetection
  // shows up as an id out of sequence rather than as a dot that looks fine.
  const std::vector<int>& ProbeIds() const { return probe_ids_; }
  long ProbeAgeMs(long now_ms) const {
    return probe_ms_ ? (now_ms - probe_ms_) : -1;
  }
  // Run a board probe if one is due (throttled by CALIB_PROBE_MS).
  void ProbeIfDue(const cv::Mat& gray, long now_ms);

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

 private:
  // Everything below runs on the scheduler thread only — same thread as
  // ProcessRawVideo and the HTTP handlers, so none of it locks.
  bool DetectBoard(const cv::Mat& gray, std::vector<cv::Point2f>& corners,
                   std::vector<int>& ids);
  CalibState CaptureView(const cv::Mat& gray);
  double MeanCommonCornerMove(const std::vector<cv::Point2f>& corners,
                              const std::vector<int>& ids) const;
  bool PoseDiversityOk(char* reason, size_t reason_size) const;
  void BuildPoints(const std::vector<size_t>& active,
                   std::vector<std::vector<cv::Point3f> >& obj_pts,
                   std::vector<std::vector<cv::Point2f> >& img_pts) const;
  void SetReason(const char* reason);
  bool SaveOne(int ch);
  bool LoadOne(int ch);

  CharucoBoardConfig board_;
  unsigned board_generation_;  // bumped on every change; invalidates the cached detector

  int        active_ch_;
  CalibState state_;
  CalibState last_capture_;
  bool       capture_pending_;
  bool       gates_;
  int        target_views_;
  double     rms_limit_;

  std::vector<std::vector<cv::Point2f> > views_corners_;
  std::vector<std::vector<int> >         views_ids_;
  std::vector<cv::Point2f>               view_centers_;
  std::vector<double>                    view_coverage_;
  cv::Size    image_size_;
  double      rms_;
  int         pruned_views_;
  const char* fail_reason_;
  char        reason_buf_[192];
  CalibViewQuality last_quality_;
  long long   last_capture_ms_;

  std::vector<cv::Point2f> probe_corners_;
  std::vector<int> probe_ids_;
  int  probe_total_;
  long probe_ms_;

  cv::Mat K_[kChannels];
  cv::Mat dist_[kChannels];
  bool    available_[kChannels];
  bool    persist_ok_;
  char    persist_dir_[256];
  char    cwd_[256];
  // Builds a path inside persist_dir_. All file access goes through this so
  // the discovered directory is used everywhere, with no compiled-in path left
  // to disagree with it.
  void PathFor(const char* leaf, char* out, size_t out_size) const;
};
