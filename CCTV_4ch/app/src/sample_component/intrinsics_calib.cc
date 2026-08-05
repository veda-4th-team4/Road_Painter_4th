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
    Session& S = sess_[c];
    S.state = CS_IDLE;
    S.last_capture = CS_IDLE;
    S.capture_pending = false;
    S.rms = 0.0;
    S.pruned_views = 0;
    S.last_capture_ms = 0;
    S.probe_total = 0;
    S.probe_ms = 0;
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

  for (int c = 0; c < kChannels; ++c) available_[c] = LoadOne(c);

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

bool IntrinsicsCalib::SaveOne(int ch) {
  char leaf[64], path[512];
  snprintf(leaf, sizeof(leaf), "camera_intrinsics_ch%d.txt", ch);
  PathFor(leaf, path, sizeof(path));
  FILE* f = fopen(path, "w");
  if (!f) return false;
  fprintf(f, "%d %d\n", kFileVersion, ch);
  fprintf(f, "%.10e %.10e %.10e %.10e\n", K_[ch].at<double>(0, 0), K_[ch].at<double>(1, 1),
          K_[ch].at<double>(0, 2), K_[ch].at<double>(1, 2));
  for (int i = 0; i < 5; ++i) fprintf(f, "%.10e ", dist_[ch].at<double>(0, i));
  fprintf(f, "\n");
  fclose(f);
  return true;
}

bool IntrinsicsCalib::LoadOne(int ch) {
  char leaf[64], path[512];
  snprintf(leaf, sizeof(leaf), "camera_intrinsics_ch%d.txt", ch);
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
  K_[ch] = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
  dist_[ch] = (cv::Mat_<double>(1, 5) << d[0], d[1], d[2], d[3], d[4]);
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
  available_[ch] = false;
  K_[ch].release();
  dist_[ch].release();
  char leaf[64], path[512];
  snprintf(leaf, sizeof(leaf), "camera_intrinsics_ch%d.txt", ch);
  PathFor(leaf, path, sizeof(path));
  remove(path);  // absent already is success as far as the caller cares
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
  // ANY open session blocks this, not just one. The board is shared, so a
  // change mid-session would mix two geometries into whichever fits are still
  // collecting — and with four sessions the odds of one being open are higher,
  // not lower.
  if (AnyCollecting()) {
    if (reason) *reason = "열려 있는 세션이 있습니다 — 보드 설정은 세션 중에 바꿀 수 없습니다";
    return false;
  }
  if (!valid_board(cfg, reason)) return false;
  // RAM only. Persisting is a separate operator action so that applying a
  // config can never fail because of storage.
  board_ = cfg;
  ++board_generation_;
  if (reason) *reason = NULL;
  return true;
}

bool IntrinsicsCalib::DetectBoard(const cv::Mat& gray, std::vector<cv::Point2f>& corners,
                                  std::vector<int>& ids) {
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
  DetectBoard(gray, S.probe_corners, S.probe_ids);
  S.probe_total = (board_.squares_x - 1) * (board_.squares_y - 1);
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

  K_[ch] = best_K;
  dist_[ch] = best_dist.reshape(1, 1).colRange(0, 5).clone();
  available_[ch] = true;
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

  K_[ch] = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
  dist_[ch] = (cv::Mat_<double>(1, 5) << dist[0], dist[1], dist[2], dist[3], dist[4]);
  available_[ch] = true;
  return true;
}
