#include "intrinsics_calib.h"

#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <map>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/aruco.hpp>

#include "app_config.h"

// OpenCV moved aruco/charuco into objdetect in 4.7 and reworked the API. The
// camera builds against 4.6, so the legacy branch is the live one here — the
// new-API branch is kept only so this file survives a toolchain bump.
#define ARUCO_NEW_API (CV_VERSION_MAJOR > 4 || \
                       (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7))
#if ARUCO_NEW_API
#include <opencv2/objdetect/charuco_detector.hpp>
#else
#include <opencv2/aruco/charuco.hpp>
#endif

namespace {

// A practical calibration wants 10+ independent views for a stable K, but the
// operator is trusted to ask for fewer (quick tests), so this is a floor
// against nonsense rather than a recommendation.
const int kMinViews = 1;

long long monotonic_ms() {
  return (long long)(cv::getTickCount() * 1000.0 / cv::getTickFrequency());
}

int dictionary_capacity(int id) {
  if (id >= 0 && id <= 15) {
    static const int capacities[4] = {50, 100, 250, 1000};
    return capacities[id % 4];
  }
  if (id == 16) return 1024;  // DICT_ARUCO_ORIGINAL
  if (id == 17) return 30;    // DICT_APRILTAG_16h5
  if (id == 18) return 35;    // DICT_APRILTAG_25h9
  if (id == 19) return 2320;  // DICT_APRILTAG_36h10
  if (id == 20) return 587;   // DICT_APRILTAG_36h11
  return 0;
}

bool valid_board(const CharucoBoardConfig& c, const char** reason) {
  const char* why = NULL;
  if (c.squares_x < 3 || c.squares_y < 3 || c.squares_x > 100 || c.squares_y > 100)
    why = "칸 수는 3~100 범위여야 합니다";
  else if (!std::isfinite(c.square_length_mm) ||
           c.square_length_mm <= 0.f || c.square_length_mm > 5000.f)
    why = "한 칸 크기는 0 초과 5000mm 이하여야 합니다";
  else if (!std::isfinite(c.marker_length_mm) || c.marker_length_mm <= 0.f ||
           c.marker_length_mm >= c.square_length_mm)
    why = "ArUco 크기는 0 초과이면서 한 칸 크기보다 작아야 합니다";
  else if (c.dictionary_id < 0 || c.dictionary_id > 20)
    why = "지원하지 않는 사전 id";
  else if ((c.squares_x * c.squares_y) / 2 > dictionary_capacity(c.dictionary_id))
    why = "이 보드가 필요한 마커 수를 사전이 감당하지 못합니다";
  else if (!std::isfinite(c.outer_margin_x_mm) || !std::isfinite(c.outer_margin_y_mm) ||
           c.outer_margin_x_mm < 0.f || c.outer_margin_y_mm < 0.f)
    why = "바깥 여백은 음수일 수 없습니다";
  if (reason) *reason = why;
  return why == NULL;
}

double view_sharpness(const cv::Mat& gray, const std::vector<cv::Point2f>& corners) {
  cv::Rect roi = cv::boundingRect(corners);
  roi.x = std::max(0, roi.x);
  roi.y = std::max(0, roi.y);
  roi.width = std::min(gray.cols - roi.x, roi.width);
  roi.height = std::min(gray.rows - roi.y, roi.height);
  if (roi.width < 8 || roi.height < 8) return 0.0;

  cv::Mat lap;
  cv::Laplacian(gray(roi), lap, CV_64F);
  cv::Scalar mean, stddev;
  cv::meanStdDev(lap, mean, stddev);
  return stddev[0] * stddev[0];
}

double view_coverage(const cv::Size& image_size, const std::vector<cv::Point2f>& corners) {
  std::vector<cv::Point2f> hull;
  cv::convexHull(corners, hull);
  if (hull.size() < 3 || image_size.area() <= 0) return 0.0;
  return std::fabs(cv::contourArea(hull)) / (double)image_size.area();
}

double per_view_rms(const std::vector<cv::Point3f>& object_points,
                    const std::vector<cv::Point2f>& image_points,
                    const cv::Mat& rvec, const cv::Mat& tvec,
                    const cv::Mat& K, const cv::Mat& dist) {
  std::vector<cv::Point2f> projected;
  cv::projectPoints(object_points, rvec, tvec, K, dist, projected);
  if (projected.empty()) return 1e9;
  double sum_sq = 0.0;
  for (size_t i = 0; i < projected.size(); ++i) {
    cv::Point2f d = projected[i] - image_points[i];
    sum_sq += d.dot(d);
  }
  return std::sqrt(sum_sq / projected.size());
}

}  // namespace

IntrinsicsCalib::IntrinsicsCalib()
    : board_generation_(1),
      gates_(CALIB_QUALITY_GATES_DEFAULT != 0),
      target_views_(K_CALIB_VIEWS),
      rms_limit_(K_CALIB_RMS_LIMIT),
      fail_reason_(""),
      persist_ok_(false) {
  const CharucoBoardConfig def = {
      CHARUCO_SQUARES_X, CHARUCO_SQUARES_Y, CHARUCO_SQUARE_LEN,
      CHARUCO_MARKER_LEN, CHARUCO_DICTIONARY,
      CHARUCO_MARGIN_X_MM, CHARUCO_MARGIN_Y_MM};
  board_ = def;
  reason_buf_[0] = '\0';
  for (int c = 0; c < kChannels; ++c) {
    available_[c] = false;
    k_generation_[c] = 0;
    Session& S = sess_[c];
    S.state = CS_IDLE;
    S.last_capture = CS_IDLE;
    S.capture_pending = false;
    S.rms = 0.0;
    S.pruned_views = 0;
    S.last_capture_ms = 0;
    S.probe_total = 0;
    S.probe_ms = 0;
    S.board_fit_rms = -1.0;    // -1 = 아직 잴 만큼 마커를 못 봄
    S.board_fit_rms_t = -1.0;
    memset(&S.last_quality, 0, sizeof(S.last_quality));
    S.last_quality.mean_move_px = -1.0;
    S.reason_buf[0] = '\0';
    S.last_quality.reason = S.reason_buf;
  }
}

void IntrinsicsCalib::PathFor(const char* leaf, char* out, size_t out_size) const {
  snprintf(out, out_size, "%s/%s", persist_dir_, leaf);
}

void IntrinsicsCalib::SetReason(const char* reason) {
  snprintf(reason_buf_, sizeof(reason_buf_), "%s", reason ? reason : "");
  fail_reason_ = reason_buf_;
}

void IntrinsicsCalib::SetSessionReason(int ch, const char* reason) {
  SetReason(reason);
  if (!ValidCh(ch)) return;
  Session& S = sess_[ch];
  snprintf(S.reason_buf, sizeof(S.reason_buf), "%s", reason ? reason : "");
  S.last_quality.reason = S.reason_buf;
}

void IntrinsicsCalib::Init() {
  if (getcwd(cwd_, sizeof(cwd_)) == NULL) snprintf(cwd_, sizeof(cwd_), "?");

  // Find somewhere to write, by trying rather than by knowing. The path
  // cctv_app used is compiled in as the first candidate, but it does not exist
  // for this app (confirmed on .13, 2026-08-04: persist ok=false), and the SDK
  // does not document an absolute one — the manifest only ever refers to
  // storage RELATIVE to where the app is unpacked. So: the documented relative
  // locations, then the app's own directory, then the legacy absolute path.
  //
  // Order matters. "../storage" is the directory the packaging layout calls
  // app/storage, the one the installer offers to preserve across an app
  // upgrade; anything else here survives a restart but not necessarily an
  // upgrade, and the app's own directory is very likely to be replaced by one.
  // Every candidate is probed and RECORDED, not just probed until one works.
  //
  // Stopping at the first success answered "can we save at all", which was the
  // question in 2026-08-04. The question now is different: everything here
  // lives under the app directory, so deleting the app with KeepOldSettings
  // unchecked takes the calibration with it. Whether anywhere OUTSIDE that tree
  // is writable is not knowable from a probe that stops early — and the camera
  // refuses SSH (port 22 closed), so there is no other way to look.
  //
  // The paths below the fixed list are the ones that would survive an app
  // delete if the SDK sandbox permits them. They are probed last and only
  // reported, never selected: moving the storage location silently would orphan
  // the files already written and lose a calibration to a "fix".
  static const char* kCandidates[] = {
      // Selectable, in priority order. "../storage" is what the packaging
      // layout calls app/storage, the directory the installer offers to
      // preserve across an upgrade.
      "../storage/ArucoPosePNM", "../storage", "./storage",
      PERSIST_DIR, "/mnt/opensdk/storage", ".",
      // Reported only — outside the app tree, so a delete would not take them.
      "/mnt/opensdk", "/mnt/opensdk/data", "/mnt/opensdk/apps",
      "/mnt/userdata", "/mnt/user", "/mnt/data", "/mnt/nand", "/mnt/sd",
      "/mnt/sdcard", "/mnt", "/opt/opensdk", "/data", "/tmp",
  };
  // Only the first kSelectable may be chosen as the storage location.
  static const size_t kSelectable = 6;

  persist_dir_[0] = '\0';
  cand_n_ = 0;
  for (size_t i = 0; i < sizeof(kCandidates) / sizeof(kCandidates[0]); ++i) {
    const bool selectable = (i < kSelectable);
    // mkdir only where we are allowed to settle. Creating directories all over
    // the camera's filesystem to find out whether we could is not a probe, it
    // is litter — and on a device whose layout we cannot inspect, litter we
    // could not find again to remove.
    if (selectable) mkdir(kCandidates[i], 0755);

    struct stat st;
    const bool exists = (stat(kCandidates[i], &st) == 0) && S_ISDIR(st.st_mode);

    bool writable = false;
    if (exists || selectable) {
      char probe[512];
      snprintf(probe, sizeof(probe), "%s/.write_test", kCandidates[i]);
      FILE* f = fopen(probe, "w");
      if (f) {
        fclose(f);
        remove(probe);
        writable = true;
      }
    }

    if (cand_n_ < kMaxCandidates) {
      PersistCandidate& c = cand_[cand_n_++];
      snprintf(c.path, sizeof(c.path), "%s", kCandidates[i]);
      c.exists = exists;
      c.writable = writable;
      c.selectable = selectable;
      c.chosen = false;
    }

    if (selectable && writable && !persist_ok_) {
      snprintf(persist_dir_, sizeof(persist_dir_), "%s", kCandidates[i]);
      persist_ok_ = true;
      if (cand_n_ > 0) cand_[cand_n_ - 1].chosen = true;
    }
  }
  if (!persist_ok_)
    snprintf(persist_dir_, sizeof(persist_dir_), "%s", PERSIST_DIR);  // for the message

  // Board first: LoadOne() does not need it, but a K loaded without knowing
  // which board produced it is only half the record.
  char board_path[512];
  PathFor("charuco_board.txt", board_path, sizeof(board_path));
  FILE* bf = fopen(board_path, "r");
  if (bf) {
    CharucoBoardConfig c;
    const int n = fscanf(bf, "%d %d %f %f %d %f %f", &c.squares_x, &c.squares_y,
                         &c.square_length_mm, &c.marker_length_mm, &c.dictionary_id,
                         &c.outer_margin_x_mm, &c.outer_margin_y_mm);
    fclose(bf);
    if (n == 7 && valid_board(c, NULL)) {
      board_ = c;
      ++board_generation_;
    }
  }

  // LoadOne() sets available_ itself now, through InstallK() — the load is a
  // K change like any other, and routing it anywhere else would leave the one
  // path that runs at start-up outside the counter.
  for (int c = 0; c < kChannels; ++c) LoadOne(c);

  printf("[ArucoPosePNM] calib: persist=%s dir=%s cwd=%s  loaded K:",
         persist_ok_ ? "ok" : "READ-ONLY", persist_dir_, cwd_);
  for (int c = 0; c < kChannels; ++c) printf(" ch%d=%d", c, (int)available_[c]);
  printf("\n");
  fflush(stdout);
}

// --- persistence -----------------------------------------------------------
// One file per lens, same three-line format cctv_app used minus the profile
// name (profiles were not ported). Text, because this file is something an
// operator may well end up reading over SSH while working out why a lens looks
// wrong, and a binary blob would need a tool to answer that.
namespace {
const int kFileVersion = 2;  // v1 was cctv_app's single-lens file (had a profile line)
}

void IntrinsicsCalib::MainLeaf(int ch, char* out, size_t n) {
  snprintf(out, n, "camera_intrinsics_ch%d.txt", ch);
}

void IntrinsicsCalib::PrevLeaf(int ch, char* out, size_t n) {
  snprintf(out, n, "camera_intrinsics_ch%d.prev.txt", ch);
}

bool IntrinsicsCalib::WriteValues(const char* leaf, int ch, const cv::Mat& K,
                                  const cv::Mat& dist) const {
  if (K.empty() || dist.empty()) return false;
  char path[512];
  PathFor(leaf, path, sizeof(path));
  FILE* f = fopen(path, "w");
  if (!f) return false;
  fprintf(f, "%d %d\n", kFileVersion, ch);
  fprintf(f, "%.10e %.10e %.10e %.10e\n", K.at<double>(0, 0), K.at<double>(1, 1),
          K.at<double>(0, 2), K.at<double>(1, 2));
  for (int i = 0; i < 5; ++i) fprintf(f, "%.10e ", dist.at<double>(0, i));
  fprintf(f, "\n");
  fclose(f);
  return true;
}

bool IntrinsicsCalib::ReadValues(const char* leaf, int ch, cv::Mat* K,
                                 cv::Mat* dist) const {
  char path[512];
  PathFor(leaf, path, sizeof(path));
  FILE* f = fopen(path, "r");
  if (!f) return false;
  int version = 0, file_ch = -1;
  if (fscanf(f, "%d %d", &version, &file_ch) != 2 || version != kFileVersion) {
    fclose(f);
    return false;  // unknown/old format — treat as "not available"
  }
  double fx, fy, cx, cy, d[5];
  const int n = fscanf(f, "%lf %lf %lf %lf %lf %lf %lf %lf %lf", &fx, &fy, &cx, &cy,
                       &d[0], &d[1], &d[2], &d[3], &d[4]);
  fclose(f);
  if (n != 9) return false;
  // The channel is in the file as well as the name. A file copied to the wrong
  // name is a plausible mistake (four near-identical files, one SSH session)
  // and a lens silently running another lens's K is not something the numbers
  // would ever look wrong enough to reveal.
  if (file_ch != ch) {
    printf("[ArucoPosePNM] calib: %s says ch%d — ignored\n", path, file_ch);
    fflush(stdout);
    return false;
  }
  if (K) *K = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
  if (dist) *dist = (cv::Mat_<double>(1, 5) << d[0], d[1], d[2], d[3], d[4]);
  return true;
}

/**
 * Write this lens's K/dist, keeping the value it replaces.
 *
 * The rotation happens here rather than in Compute() because the FILE is the
 * durable thing: Compute() only touches RAM, so rotating there would push a
 * "previous" onto disk for a value that may never be saved at all — and two
 * computes in a row would then lose the last SAVED value, which is exactly the
 * one worth keeping.
 *
 * A failed rotation does not stop the save. Losing the ability to step back is
 * bad; refusing to store the calibration the operator just spent ten minutes
 * taking is worse.
 */
bool IntrinsicsCalib::SaveOne(int ch) {
  char main_leaf[64], prev_leaf[64];
  MainLeaf(ch, main_leaf, sizeof(main_leaf));
  PrevLeaf(ch, prev_leaf, sizeof(prev_leaf));

  cv::Mat old_K, old_dist;
  if (ReadValues(main_leaf, ch, &old_K, &old_dist)) {
    if (!WriteValues(prev_leaf, ch, old_K, old_dist)) {
      printf("[ArucoPosePNM] calib ch%d: 이전 값 보관 실패 — 저장은 계속합니다\n", ch);
      fflush(stdout);
    }
  }
  return WriteValues(main_leaf, ch, K_[ch], dist_[ch]);
}

bool IntrinsicsCalib::HasPrevious(int ch) const {
  if (!ValidCh(ch)) return false;
  char leaf[64];
  PrevLeaf(ch, leaf, sizeof(leaf));
  return ReadValues(leaf, ch, NULL, NULL);
}

bool IntrinsicsCalib::GetPrevious(int ch, double* fx, double* fy, double* cx, double* cy,
                                  double dist[5]) const {
  if (!ValidCh(ch)) return false;
  char leaf[64];
  PrevLeaf(ch, leaf, sizeof(leaf));
  cv::Mat K, D;
  if (!ReadValues(leaf, ch, &K, &D)) return false;
  if (fx) *fx = K.at<double>(0, 0);
  if (fy) *fy = K.at<double>(1, 1);
  if (cx) *cx = K.at<double>(0, 2);
  if (cy) *cy = K.at<double>(1, 2);
  if (dist)
    for (int i = 0; i < 5; ++i) dist[i] = D.at<double>(0, i);
  return true;
}

/**
 * Swap the current and previous values, on disk and in RAM.
 *
 * Both files are read in full before either is written, so a write that fails
 * halfway cannot leave the lens holding a value that exists nowhere else.
 *
 * A swap rather than a pop: reverting by mistake is undone by reverting again,
 * and there is never a moment when one of the two has been thrown away.
 *
 * If the current file is missing (computed but never saved) RAM stands in for
 * it. The operator's model is "go back to the previous one"; silently dropping
 * the unsaved value would make the second Revert — the one meant to undo this
 * — return nothing.
 */
bool IntrinsicsCalib::Revert(int ch) {
  if (!ValidCh(ch)) return false;
  char main_leaf[64], prev_leaf[64];
  MainLeaf(ch, main_leaf, sizeof(main_leaf));
  PrevLeaf(ch, prev_leaf, sizeof(prev_leaf));

  cv::Mat prev_K, prev_dist;
  if (!ReadValues(prev_leaf, ch, &prev_K, &prev_dist)) {
    SetReason("이 렌즈에는 되돌릴 이전 값이 없습니다");
    return false;
  }

  cv::Mat cur_K, cur_dist;
  bool have_cur = ReadValues(main_leaf, ch, &cur_K, &cur_dist);
  if (!have_cur && available_[ch]) {
    cur_K = K_[ch].clone();
    cur_dist = dist_[ch].clone();
    have_cur = true;
  }

  if (!WriteValues(main_leaf, ch, prev_K, prev_dist)) {
    SetReason("되돌리기 실패 — 저장 위치에 쓸 수 없습니다");
    return false;
  }
  if (have_cur) {
    WriteValues(prev_leaf, ch, cur_K, cur_dist);
  } else {
    char path[512];
    PathFor(prev_leaf, path, sizeof(path));
    remove(path);  // nothing left to step back to
  }

  InstallK(ch, prev_K, prev_dist);
  return true;
}

void IntrinsicsCalib::InstallK(int ch, const cv::Mat& K, const cv::Mat& dist) {
  if (!ValidCh(ch)) return;
  K_[ch] = K;
  dist_[ch] = dist;
  available_[ch] = true;
  ++k_generation_[ch];
}

void IntrinsicsCalib::DropK(int ch) {
  if (!ValidCh(ch)) return;
  K_[ch].release();
  dist_[ch].release();
  available_[ch] = false;
  // Bumped on the way out too. A lens whose K was cleared is exactly as
  // different from the one an H was fitted against as a lens that was
  // recalibrated, and the mapping path has to hear about both.
  ++k_generation_[ch];
}

bool IntrinsicsCalib::LoadOne(int ch) {
  char leaf[64];
  MainLeaf(ch, leaf, sizeof(leaf));
  cv::Mat K, dist;
  if (!ReadValues(leaf, ch, &K, &dist)) return false;
  InstallK(ch, K, dist);
  return true;
}

bool IntrinsicsCalib::Save(int ch) {
  if (ch < 0 || ch >= kChannels || !available_[ch]) {
    SetReason("저장할 K/dist가 없습니다 — 먼저 계산하거나 직접 입력하세요");
    return false;
  }
  if (!SaveOne(ch)) {
    SetReason("저장 위치에 쓰기 실패");
    return false;
  }
  return true;
}

bool IntrinsicsCalib::Clear(int ch) {
  if (ch < 0 || ch >= kChannels) return false;
  DropK(ch);
  char leaf[64], path[512];
  MainLeaf(ch, leaf, sizeof(leaf));
  PathFor(leaf, path, sizeof(path));
  remove(path);  // absent already is success as far as the caller cares
  // The previous value is deliberately LEFT in place. Clearing a lens is what
  // an operator does when its calibration is wrong, and the value before that
  // one is the most likely thing they want next — deleting it here would make
  // "clear" quietly destroy two calibrations while naming one.
  return true;
}

bool IntrinsicsCalib::SaveBoard() {
  char path[512];
  PathFor("charuco_board.txt", path, sizeof(path));
  FILE* f = fopen(path, "w");
  if (!f) {
    SetReason("저장 위치에 쓰기 실패");
    return false;
  }
  fprintf(f, "%d %d %.6f %.6f %d %.6f %.6f\n", board_.squares_x, board_.squares_y,
          board_.square_length_mm, board_.marker_length_mm, board_.dictionary_id,
          board_.outer_margin_x_mm, board_.outer_margin_y_mm);
  fclose(f);
  return true;
}

// --- board -----------------------------------------------------------------

bool IntrinsicsCalib::SetBoard(const CharucoBoardConfig& cfg, const char** reason) {
  // An open session blocks this — but only once it has BANKED something.
  //
  // The rule exists so a change cannot mix two geometries into one fit, and a
  // session holding zero views has no geometry to mix. Refusing those too made
  // the wrong-orientation warning unusable: the board probe only runs while a
  // session is open, so the moment the operator can SEE that the layout is
  // wrong is exactly the moment they were forbidden from fixing it. They had to
  // stop the session, change the board, and start again — three steps to undo a
  // setting that had not yet touched anything.
  for (int c = 0; c < kChannels; ++c) {
    if (sess_[c].state == CS_COLLECTING && !sess_[c].views_corners.empty()) {
      if (reason)
        *reason = "이미 촬영한 뷰가 있는 세션이 있습니다 — 보드 설정을 바꾸려면 먼저 그 세션을 멈추세요";
      return false;
    }
  }
  if (!valid_board(cfg, reason)) return false;
  // RAM only. Persisting is a separate operator action so that applying a
  // config can never fail because of storage.
  board_ = cfg;
  ++board_generation_;
  if (reason) *reason = NULL;
  return true;
}

/**
 * How well do these marker centres match an sx-by-sy ChArUco board?
 *
 * OpenCV lays a ChArUco board out by walking the chessboard row by row and
 * dropping marker 0, 1, 2 ... into every square of one colour. So marker i has
 * a known square, and the square's centre is a known board coordinate. Fit
 * board coordinates to the observed pixels with a homography — which absorbs
 * however the board happens to be tilted — and the leftover is the answer.
 *
 * Both parities are tried because which colour carries the markers is the
 * generator's business, and guessing it wrong would make a correct layout look
 * like a wrong one.
 */
double IntrinsicsCalib::MarkerLayoutRms(const std::vector<cv::Point2f>& centers,
                                        const std::vector<int>& ids, int squares_x,
                                        int squares_y) {
  if (centers.size() != ids.size() || ids.size() < 5) return -1.0;
  if (squares_x < 2 || squares_y < 2) return -1.0;

  double best = -1.0;
  for (int parity = 0; parity < 2; ++parity) {
    // Board coordinate of each marker index, in squares.
    std::vector<cv::Point2f> slot;
    for (int r = 0; r < squares_y; ++r)
      for (int c = 0; c < squares_x; ++c)
        if (((r + c) & 1) == parity)
          slot.push_back(cv::Point2f((float)c + 0.5f, (float)r + 0.5f));

    std::vector<cv::Point2f> src, dst;
    for (size_t i = 0; i < ids.size(); ++i) {
      if (ids[i] < 0 || ids[i] >= (int)slot.size()) continue;  // not this board
      src.push_back(slot[ids[i]]);
      dst.push_back(centers[i]);
    }
    if (src.size() < 5) continue;

    cv::Mat H;
    try {
      H = cv::findHomography(src, dst, 0);  // plain least squares, no RANSAC:
                                            // an outlier here IS the signal
    } catch (const cv::Exception&) {
      continue;
    }
    if (H.empty()) continue;

    double sum = 0.0;
    for (size_t i = 0; i < src.size(); ++i) {
      const double w = H.at<double>(2, 0) * src[i].x + H.at<double>(2, 1) * src[i].y +
                       H.at<double>(2, 2);
      if (std::fabs(w) < 1e-12) { sum = -1.0; break; }
      const double px = (H.at<double>(0, 0) * src[i].x + H.at<double>(0, 1) * src[i].y +
                         H.at<double>(0, 2)) / w;
      const double py = (H.at<double>(1, 0) * src[i].x + H.at<double>(1, 1) * src[i].y +
                         H.at<double>(1, 2)) / w;
      const double dx = px - dst[i].x, dy = py - dst[i].y;
      sum += dx * dx + dy * dy;
    }
    if (sum < 0.0) continue;
    const double rms = std::sqrt(sum / (double)src.size());
    if (best < 0.0 || rms < best) best = rms;
  }
  return best;
}

bool IntrinsicsCalib::DetectBoard(const cv::Mat& gray, std::vector<cv::Point2f>& corners,
                                  std::vector<int>& ids,
                                  std::vector<cv::Point2f>* marker_centers,
                                  std::vector<int>* out_marker_ids) {
  corners.clear();
  ids.clear();
  try {
    std::vector<int> marker_ids;
    std::vector<std::vector<cv::Point2f> > marker_corners;
    // The detector and board are rebuilt only when the board description
    // changes — constructing them is not free and the description is stable
    // for whole sessions at a time.
#if ARUCO_NEW_API
    static unsigned cached_generation = 0;
    static cv::Ptr<cv::aruco::CharucoDetector> detector;
    if (cached_generation != board_generation_ || detector.empty()) {
      cv::aruco::DetectorParameters params;
      params.cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
      cv::aruco::CharucoBoard board(
          cv::Size(board_.squares_x, board_.squares_y), board_.square_length_mm,
          board_.marker_length_mm, cv::aruco::getPredefinedDictionary(board_.dictionary_id));
      detector = cv::makePtr<cv::aruco::CharucoDetector>(
          board, cv::aruco::CharucoParameters(), params);
      cached_generation = board_generation_;
    }
    detector->detectBoard(gray, corners, ids, marker_corners, marker_ids);
#else
    static unsigned cached_generation = 0;
    static cv::Ptr<cv::aruco::Dictionary> dict;
    static cv::Ptr<cv::aruco::CharucoBoard> board;
    static cv::Ptr<cv::aruco::DetectorParameters> params;
    if (cached_generation != board_generation_ || board.empty()) {
      dict = cv::aruco::getPredefinedDictionary(board_.dictionary_id);
      board = cv::aruco::CharucoBoard::create(board_.squares_x, board_.squares_y,
                                              board_.square_length_mm,
                                              board_.marker_length_mm, dict);
      params = cv::aruco::DetectorParameters::create();
      params->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
      cached_generation = board_generation_;
    }
    cv::aruco::detectMarkers(gray, dict, marker_corners, marker_ids, params);
    if (!marker_ids.empty())
      cv::aruco::interpolateCornersCharuco(marker_corners, marker_ids, gray, board,
                                           corners, ids);
#endif
    // The raw marker detections, handed back for the layout check. Taken here
    // rather than re-detecting: running detectMarkers a second time would cost
    // as much as the whole probe and could disagree with the corners above.
    if (marker_centers && out_marker_ids) {
      marker_centers->clear();
      out_marker_ids->clear();
      for (size_t i = 0; i < marker_corners.size() && i < marker_ids.size(); ++i) {
        if (marker_corners[i].size() < 4) continue;
        cv::Point2f c(0.f, 0.f);
        for (int k = 0; k < 4; ++k) c += marker_corners[i][k];
        marker_centers->push_back(c * 0.25f);
        out_marker_ids->push_back(marker_ids[i]);
      }
    }
  } catch (const cv::Exception&) {
    return false;
  }
  return true;
}

void IntrinsicsCalib::ProbeIfDue(int ch, const cv::Mat& gray, long now_ms) {
  if (!ValidCh(ch)) return;
  Session& S = sess_[ch];
  if (S.state != CS_COLLECTING) return;
  if (S.probe_ms != 0 && now_ms - S.probe_ms < CALIB_PROBE_MS) return;
  std::vector<cv::Point2f> mcen;
  std::vector<int> mids;
  DetectBoard(gray, S.probe_corners, S.probe_ids, &mcen, &mids);
  S.probe_total = (board_.squares_x - 1) * (board_.squares_y - 1);
  // Judge the layout on the same frame the viewfinder is showing, so the two
  // can never disagree about what was in front of the lens.
  S.board_fit_rms   = MarkerLayoutRms(mcen, mids, board_.squares_x, board_.squares_y);
  S.board_fit_rms_t = MarkerLayoutRms(mcen, mids, board_.squares_y, board_.squares_x);
  S.probe_ms = now_ms;
}

// --- sessions --------------------------------------------------------------

bool IntrinsicsCalib::Collecting(int ch) const {
  return ValidCh(ch) && sess_[ch].state == CS_COLLECTING;
}

bool IntrinsicsCalib::AnyCollecting() const {
  for (int c = 0; c < kChannels; ++c)
    if (sess_[c].state == CS_COLLECTING) return true;
  return false;
}

CalibState IntrinsicsCalib::State(int ch) const {
  return ValidCh(ch) ? sess_[ch].state : CS_IDLE;
}
CalibState IntrinsicsCalib::LastCapture(int ch) const {
  return ValidCh(ch) ? sess_[ch].last_capture : CS_IDLE;
}
int IntrinsicsCalib::Views(int ch) const {
  return ValidCh(ch) ? (int)sess_[ch].views_corners.size() : 0;
}
int IntrinsicsCalib::PrunedViews(int ch) const {
  return ValidCh(ch) ? sess_[ch].pruned_views : 0;
}
double IntrinsicsCalib::Rms(int ch) const {
  return ValidCh(ch) ? sess_[ch].rms : 0.0;
}
const CalibViewQuality& IntrinsicsCalib::LastQuality(int ch) const {
  static const CalibViewQuality kNone = {0, 0, 0.0, 0.0, -1.0, ""};
  return ValidCh(ch) ? sess_[ch].last_quality : kNone;
}
bool IntrinsicsCalib::CapturePending(int ch) const {
  return ValidCh(ch) && sess_[ch].capture_pending;
}
const std::vector<cv::Point2f>& IntrinsicsCalib::ProbeCorners(int ch) const {
  static const std::vector<cv::Point2f> kNone;
  return ValidCh(ch) ? sess_[ch].probe_corners : kNone;
}
const std::vector<int>& IntrinsicsCalib::ProbeIds(int ch) const {
  static const std::vector<int> kNone;
  return ValidCh(ch) ? sess_[ch].probe_ids : kNone;
}
long IntrinsicsCalib::ProbeAgeMs(int ch, long now_ms) const {
  if (!ValidCh(ch) || sess_[ch].probe_ms == 0) return -1;
  return now_ms - sess_[ch].probe_ms;
}

bool IntrinsicsCalib::Start(int ch, const char** reason) {
  if (!ValidCh(ch)) {
    if (reason) *reason = "채널 번호 범위 초과";
    return false;
  }
  // No exclusivity check any more. Another lens collecting is the normal case
  // now, not a conflict: one board pose is meant to feed several sessions. The
  // old refusal existed to stop views from two optics landing in one fit, and
  // that is now prevented by construction — every view goes into the session of
  // the lens whose frame it came from.
  Session& S = sess_[ch];
  S.views_corners.clear();
  S.views_ids.clear();
  S.view_centers.clear();
  S.view_coverage.clear();
  S.probe_corners.clear();
  S.probe_ids.clear();
  S.probe_ms = 0;
  S.board_fit_rms = -1.0;
  S.board_fit_rms_t = -1.0;
  S.pruned_views = 0;
  S.last_capture_ms = 0;
  S.image_size = cv::Size();
  S.capture_pending = false;
  memset(&S.last_quality, 0, sizeof(S.last_quality));
  S.last_quality.mean_move_px = -1.0;
  S.reason_buf[0] = '\0';
  S.last_quality.reason = S.reason_buf;
  S.state = CS_COLLECTING;
  S.last_capture = CS_IDLE;
  fail_reason_ = "";
  reason_buf_[0] = '\0';
  if (reason) *reason = NULL;
  return true;
}

void IntrinsicsCalib::Stop(int ch) {
  if (!ValidCh(ch)) return;
  Session& S = sess_[ch];
  S.state = CS_IDLE;
  S.capture_pending = false;
  S.probe_corners.clear();
  S.probe_ids.clear();
  S.probe_ms = 0;
  S.board_fit_rms = -1.0;
  S.board_fit_rms_t = -1.0;
}

bool IntrinsicsCalib::RequestCapture(const char** reason) {
  // Arms EVERY open session, so one press banks one board pose into every lens
  // that can see it. A lens that cannot is not an error here — it refuses on
  // its own frame, with its own reason, which is what the per-lens readout is
  // for.
  int armed = 0;
  for (int c = 0; c < kChannels; ++c) {
    if (sess_[c].state != CS_COLLECTING) continue;
    sess_[c].capture_pending = true;
    ++armed;
  }
  if (armed == 0) {
    if (reason) *reason = "열린 세션이 없습니다 — 먼저 렌즈를 골라 세션을 시작하세요";
    return false;
  }
  if (reason) *reason = NULL;
  return true;
}

CalibState IntrinsicsCalib::TakePendingCapture(int ch, const cv::Mat& gray) {
  if (!ValidCh(ch)) return CS_IDLE;
  Session& S = sess_[ch];
  S.capture_pending = false;
  // Recorded as the capture outcome, NOT as the session state — a rejected
  // pose leaves the session exactly where it was, with its views intact.
  S.last_capture = CaptureView(ch, gray);
  // A capture attempt is the freshest board reading there is; keeping it as
  // the probe means the page shows exactly the frame that was judged, not one
  // from up to a second earlier.
  S.probe_ms = 0;
  return S.last_capture;
}

double IntrinsicsCalib::MeanCommonCornerMove(int ch, const std::vector<cv::Point2f>& corners,
                                             const std::vector<int>& ids) const {
  const Session& S = sess_[ch];
  double best = -1.0;
  for (size_t v = 0; v < S.views_corners.size(); ++v) {
    std::map<int, cv::Point2f> old_by_id;
    for (size_t i = 0; i < S.views_ids[v].size(); ++i)
      old_by_id[S.views_ids[v][i]] = S.views_corners[v][i];

    double sum = 0.0;
    int common = 0;
    for (size_t i = 0; i < ids.size(); ++i) {
      std::map<int, cv::Point2f>::const_iterator it = old_by_id.find(ids[i]);
      if (it == old_by_id.end()) continue;
      sum += cv::norm(corners[i] - it->second);
      ++common;
    }
    if (common >= 4) {
      const double mean = sum / common;
      if (best < 0.0 || mean < best) best = mean;
    }
  }
  return best;
}

CalibState IntrinsicsCalib::CaptureView(int ch, const cv::Mat& gray) {
  Session& S = sess_[ch];
  memset(&S.last_quality, 0, sizeof(S.last_quality));
  S.last_quality.mean_move_px = -1.0;
  CalibViewQuality& q = S.last_quality;

  if (gray.empty()) {
    SetSessionReason(ch, "빈 프레임");
    return CS_CAPTURE_REJECTED;
  }
  if (S.image_size.area() > 0 && gray.size() != S.image_size) {
    SetSessionReason(ch, "세션 도중 해상도가 바뀌었습니다");
    return CS_CAPTURE_REJECTED;
  }

  std::vector<cv::Point2f> corners;
  std::vector<int> ids;
  if (!DetectBoard(gray, corners, ids)) {
    SetSessionReason(ch, "OpenCV 보드 검출 오류");
    return CS_CAPTURE_REJECTED;
  }
  S.probe_corners = corners;  // whatever was seen, so the page can show it
  S.probe_ids = ids;

  const int found = (int)ids.size();
  const int total = (board_.squares_x - 1) * (board_.squares_y - 1);
  S.probe_total = total;
  // Gates off: require only the mathematical minimum, so a small/partial view
  // is never rejected.
  const int min_corners =
      gates_ ? std::max(8, (int)std::ceil(total * K_CALIB_MIN_CORNER_RATIO)) : 4;
  q.corners_found = found;
  q.corners_total = total;
  if (found < min_corners) {
    char reason[128];
    snprintf(reason, sizeof(reason), "코너가 %d/%d개 — %d개 이상 필요", found, total,
             min_corners);
    SetSessionReason(ch, reason);
    return CS_CAPTURE_REJECTED;
  }

  for (size_t i = 0; i < corners.size(); ++i) {
    const cv::Point2f& p = corners[i];
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || p.x < 0.f || p.y < 0.f ||
        p.x >= gray.cols || p.y >= gray.rows) {
      SetSessionReason(ch, "프레임 밖이거나 잘못된 코너");
      return CS_CAPTURE_REJECTED;
    }
  }

  q.coverage_ratio = view_coverage(gray.size(), corners);
  if (gates_ && q.coverage_ratio < K_CALIB_MIN_COVERAGE) {
    SetSessionReason(ch, "보드가 너무 작게/멀리 잡혔습니다");
    return CS_CAPTURE_REJECTED;
  }

  q.sharpness = view_sharpness(gray, corners);
  if (gates_ && q.sharpness < K_CALIB_MIN_SHARPNESS) {
    SetSessionReason(ch, "흔들렸습니다 — 보드를 멈추고 다시 누르세요");
    return CS_CAPTURE_REJECTED;
  }

  const long long now = monotonic_ms();
  if (S.last_capture_ms > 0 && now - S.last_capture_ms < K_CALIB_MIN_GAP_MS) {
    SetSessionReason(ch, "직전 캡처와 너무 가깝습니다");
    return CS_CAPTURE_REJECTED;
  }

  q.mean_move_px = MeanCommonCornerMove(ch, corners, ids);
  if (gates_ && q.mean_move_px >= 0.0 && q.mean_move_px < K_CALIB_MIN_MOVE_PX) {
    SetSessionReason(ch, "이미 받은 자세와 거의 같습니다 — 각도나 거리를 바꾸세요");
    return CS_CAPTURE_REJECTED;
  }

  S.views_corners.push_back(corners);
  S.views_ids.push_back(ids);
  S.image_size = gray.size();
  cv::Point2f center(0.f, 0.f);
  for (size_t i = 0; i < corners.size(); ++i) center += corners[i];
  center *= 1.f / (float)corners.size();
  S.view_centers.push_back(center);
  S.view_coverage.push_back(q.coverage_ratio);
  S.last_capture_ms = now;
  SetSessionReason(ch, "승인됨");
  return CS_CAPTURED;
}

bool IntrinsicsCalib::UndoLast(int ch) {
  if (!ValidCh(ch)) return false;
  Session& S = sess_[ch];
  if (S.views_corners.empty()) return false;
  S.views_corners.pop_back();
  S.views_ids.pop_back();
  S.view_centers.pop_back();
  S.view_coverage.pop_back();
  S.last_capture_ms = 0;
  S.last_capture = CS_IDLE;
  return true;
}

bool IntrinsicsCalib::PoseDiversityOk(int ch, char* reason, size_t reason_size) const {
  const Session& S = sess_[ch];
  if (S.view_centers.empty() || S.view_coverage.empty()) return false;

  float min_x = S.view_centers[0].x, max_x = min_x;
  float min_y = S.view_centers[0].y, max_y = min_y;
  double min_cov = S.view_coverage[0], max_cov = min_cov;
  for (size_t i = 1; i < S.view_centers.size(); ++i) {
    min_x = std::min(min_x, S.view_centers[i].x);
    max_x = std::max(max_x, S.view_centers[i].x);
    min_y = std::min(min_y, S.view_centers[i].y);
    max_y = std::max(max_y, S.view_centers[i].y);
    min_cov = std::min(min_cov, S.view_coverage[i]);
    max_cov = std::max(max_cov, S.view_coverage[i]);
  }
  if (max_x - min_x < S.image_size.width * 0.25f) {
    snprintf(reason, reason_size, "좌우로 더 옮겨가며 찍어야 합니다");
    return false;
  }
  if (max_y - min_y < S.image_size.height * 0.20f) {
    snprintf(reason, reason_size, "위아래로 더 옮겨가며 찍어야 합니다");
    return false;
  }
  if (min_cov <= 0.0 || max_cov / min_cov < 1.35) {
    snprintf(reason, reason_size, "거리(크기) 변화가 부족합니다 — 가까이·멀리 섞으세요");
    return false;
  }
  return true;
}

void IntrinsicsCalib::BuildPoints(int ch, const std::vector<size_t>& active,
                                  std::vector<std::vector<cv::Point3f> >& obj_pts,
                                  std::vector<std::vector<cv::Point2f> >& img_pts) const {
  const Session& S = sess_[ch];
  obj_pts.clear();
  img_pts.clear();
#if ARUCO_NEW_API
  cv::aruco::CharucoBoard board(
      cv::Size(board_.squares_x, board_.squares_y), board_.square_length_mm,
      board_.marker_length_mm, cv::aruco::getPredefinedDictionary(board_.dictionary_id));
  for (size_t a = 0; a < active.size(); ++a) {
    const size_t v = active[a];
    std::vector<cv::Point3f> o;
    std::vector<cv::Point2f> i;
    board.matchImagePoints(S.views_corners[v], S.views_ids[v], o, i);
    if (o.size() >= 4) {
      obj_pts.push_back(o);
      img_pts.push_back(i);
    }
  }
#else
  cv::Ptr<cv::aruco::Dictionary> dict = cv::aruco::getPredefinedDictionary(board_.dictionary_id);
  cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(
      board_.squares_x, board_.squares_y, board_.square_length_mm,
      board_.marker_length_mm, dict);
  for (size_t a = 0; a < active.size(); ++a) {
    const size_t v = active[a];
    std::vector<cv::Point3f> o;
    std::vector<cv::Point2f> i;
    for (size_t p = 0; p < S.views_ids[v].size(); ++p) {
      const int id = S.views_ids[v][p];
      if (id >= 0 && id < (int)board->chessboardCorners.size()) {
        o.push_back(board->chessboardCorners[id]);
        i.push_back(S.views_corners[v][p]);
      }
    }
    if (o.size() >= 4) {
      obj_pts.push_back(o);
      img_pts.push_back(i);
    }
  }
#endif
}

CalibState IntrinsicsCalib::Compute(int ch) {
  if (!ValidCh(ch)) return CS_DONE_FAIL;
  Session& S = sess_[ch];
  if (S.state != CS_COLLECTING) {
    SetSessionReason(ch, "세션을 먼저 시작하세요");
    return CS_DONE_FAIL;
  }

  if ((int)S.views_corners.size() < kMinViews) {
    char reason[128];
    snprintf(reason, sizeof(reason), "승인된 뷰가 %d개 필요합니다 (현재 %d개)", kMinViews,
             (int)S.views_corners.size());
    SetSessionReason(ch, reason);
    return CS_DONE_FAIL;
  }
  {
    char reason[192];
    if (gates_ && !PoseDiversityOk(ch, reason, sizeof(reason))) {
      SetSessionReason(ch, reason);
      return CS_DONE_FAIL;
    }
  }

  std::vector<size_t> active;
  for (size_t i = 0; i < S.views_corners.size(); ++i) active.push_back(i);

  cv::Mat best_K, best_dist;
  double best_rms = 1e9;
  S.pruned_views = 0;

  // Refit after dropping the worst-reprojecting view, repeatedly. Bounded by
  // the view count and only ever runs on an explicit Compute — this is the one
  // place in the app where seconds of blocking work is the right answer.
  try {
    while ((int)active.size() >= kMinViews) {
      std::vector<std::vector<cv::Point3f> > obj_pts;
      std::vector<std::vector<cv::Point2f> > img_pts;
      BuildPoints(ch, active, obj_pts, img_pts);
      if ((int)obj_pts.size() < kMinViews) {
        SetSessionReason(ch, "점 대응 후 쓸 수 있는 뷰가 부족합니다");
        return CS_DONE_FAIL;
      }

      cv::Mat K, dist;
      std::vector<cv::Mat> rvecs, tvecs;
      const double rms = cv::calibrateCamera(obj_pts, img_pts, S.image_size, K, dist,
                                             rvecs, tvecs);

      double worst = -1.0;
      size_t worst_index = 0;
      for (size_t v = 0; v < obj_pts.size(); ++v) {
        const double view_rms =
            per_view_rms(obj_pts[v], img_pts[v], rvecs[v], tvecs[v], K, dist);
        if (view_rms > worst) {
          worst = view_rms;
          worst_index = v;
        }
      }

      best_K = K;
      best_dist = dist;
      best_rms = rms;
      if (!gates_) break;  // gates off: take the fit, no pruning and no RMS fail
      if (worst <= K_CALIB_VIEW_RMS_LIMIT) break;
      if ((int)active.size() == kMinViews) {
        char reason[192];
        snprintf(reason, sizeof(reason),
                 "최악 뷰 RMS %.2fpx가 한계 %.2fpx를 넘습니다 — 다시 촬영하세요", worst,
                 (double)K_CALIB_VIEW_RMS_LIMIT);
        SetSessionReason(ch, reason);
        return CS_DONE_FAIL;
      }
      active.erase(active.begin() + worst_index);
      ++S.pruned_views;
    }
  } catch (const cv::Exception& e) {
    char reason[192];
    snprintf(reason, sizeof(reason), "OpenCV 계산 실패: %.90s", e.what());
    SetSessionReason(ch, reason);
    return CS_DONE_FAIL;
  }

  // A non-finite RMS means the fit is garbage and always fails; the over-limit
  // check is a quality gate and is skipped when gates are off.
  if (!std::isfinite(best_rms) || (gates_ && best_rms > rms_limit_)) {
    char reason[160];
    snprintf(reason, sizeof(reason), "RMS %.2fpx가 한계 %.2fpx를 넘습니다", best_rms,
             rms_limit_);
    SetSessionReason(ch, reason);
    return CS_DONE_FAIL;
  }
  if (best_K.empty() || best_dist.total() < 5) {
    SetSessionReason(ch, "OpenCV가 불완전한 행렬을 반환했습니다");
    return CS_DONE_FAIL;
  }

  // Keep only the views that survived pruning, so the counter the page shows
  // afterwards is the number that actually produced this K.
  std::vector<std::vector<cv::Point2f> > kept_corners;
  std::vector<std::vector<int> > kept_ids;
  std::vector<cv::Point2f> kept_centers;
  std::vector<double> kept_coverage;
  for (size_t i = 0; i < active.size(); ++i) {
    kept_corners.push_back(S.views_corners[active[i]]);
    kept_ids.push_back(S.views_ids[active[i]]);
    kept_centers.push_back(S.view_centers[active[i]]);
    kept_coverage.push_back(S.view_coverage[active[i]]);
  }
  S.views_corners.swap(kept_corners);
  S.views_ids.swap(kept_ids);
  S.view_centers.swap(kept_centers);
  S.view_coverage.swap(kept_coverage);

  InstallK(ch, best_K, best_dist.reshape(1, 1).colRange(0, 5).clone());
  S.rms = best_rms;
  SetSessionReason(ch, "");
  // Not saved automatically. The value is live in RAM and usable at once;
  // writing it is a separate button so a storage failure is something the
  // operator sees and retries, not something that silently loses a session.
  // The session stays on this lens rather than being unpinned — there is no
  // single "active lens" any more, and CS_DONE_OK is what the page reads to
  // show the result next to the lens it belongs to.
  S.state = CS_DONE_OK;
  return CS_DONE_OK;
}

// --- parameters and results ------------------------------------------------

void IntrinsicsCalib::SetParams(int target_views, double rms_limit) {
  // Non-positive values are ignored rather than applied: a half-filled command
  // must not zero the target or disable the RMS gate.
  if (target_views >= kMinViews) target_views_ = target_views;
  if (rms_limit > 0.0) rms_limit_ = rms_limit;
}

bool IntrinsicsCalib::Available(int ch) const {
  return ch >= 0 && ch < kChannels && available_[ch];
}

bool IntrinsicsCalib::Get(int ch, double* fx, double* fy, double* cx, double* cy,
                          double dist[5]) const {
  if (!Available(ch)) return false;
  if (fx) *fx = K_[ch].at<double>(0, 0);
  if (fy) *fy = K_[ch].at<double>(1, 1);
  if (cx) *cx = K_[ch].at<double>(0, 2);
  if (cy) *cy = K_[ch].at<double>(1, 2);
  if (dist)
    for (int i = 0; i < 5; ++i) dist[i] = dist_[ch].at<double>(0, i);
  return true;
}

bool IntrinsicsCalib::LoadValues(int ch, double fx, double fy, double cx, double cy,
                                 const double dist[5]) {
  if (ch < 0 || ch >= kChannels) return false;
  if (!std::isfinite(fx) || !std::isfinite(fy) || fx <= 0.0 || fy <= 0.0) return false;
  if (!std::isfinite(cx) || !std::isfinite(cy) || cx < 0.0 || cy < 0.0) return false;
  for (int i = 0; i < 5; ++i)
    if (!std::isfinite(dist[i])) return false;

  const cv::Mat K = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
  const cv::Mat D =
      (cv::Mat_<double>(1, 5) << dist[0], dist[1], dist[2], dist[3], dist[4]);
  InstallK(ch, K, D);
  return true;
}
