#include "homography_mapper.h"

#include <stdio.h>
#include <string.h>

#include <cmath>

#include <opencv2/calib3d.hpp>

#include "app_config.h"  // CopyUtf8 — last_result goes straight into /status

namespace {

// Bump when the on-disk layout changes. A file from an older version is
// ignored rather than guessed at — a half-understood matrix is worse than no
// matrix, because it still produces coordinates.
const int kFileVersion = 1;

// A homography this close to singular cannot map pixels to a stable plane.
// Same threshold cctv_app used (homography_set).
const double kMinDeterminant = 1e-12;

// |w| below this means the pixel is on (or beyond) the horizon of the fitted
// plane. There is no finite ground point there, and the division would return
// a coordinate kilometres away rather than an error.
const double kMinHomogeneousW = 1e-9;

// How far an undistorted point may land from where it started after being put
// back through the forward distortion model. Inside the convergent area of the
// measured lens this is 0.00 px; the marginal band sits at 0.2..1.4 px and the
// divergent band is unbounded. Half a pixel therefore separates "converged"
// from "did not", and is also below what a homography fit would notice.
const double kMaxRoundTripPx = 0.5;

// Two registered markers closer together than this in world space are not two
// markers — they are one row entered twice. Real floor markers are hundreds of
// millimetres apart, so there is no legitimate measurement this rejects.
const double kMinAnchorSeparationMm = 1.0;

// A marker height outside this range is a typo, not a measurement. 100 m of
// ceiling is not a camera installation, and the !(x >= 0) form rejects NaN as
// well as negatives — a plain `x < 0` lets NaN through, and NaN would poison
// every derived matrix without ever failing a comparison.
const double kMaxHeightMm = 100000.0;
bool ValidHeightMm(double mm) { return (mm >= 0.0) && mm < kMaxHeightMm; }

// Map one point through a 3x3, in world millimetres. False on the vanishing
// line, where there is no finite ground point and the division would put the
// robot kilometres away rather than nowhere.
//
// One copy, called by both public mappers and by the residual computation.
// Three copies is what this file had while the marker plane was being added,
// and they had already drifted: one spelled the horizon epsilon 1e-9 inline
// while the others used kMinHomogeneousW, so tuning the named constant would
// have fixed two of the three paths.
bool MapThrough(const cv::Mat& H, double u, double v, double* wx, double* wy) {
  const double w = H.at<double>(2, 0) * u + H.at<double>(2, 1) * v + H.at<double>(2, 2);
  if (std::fabs(w) < kMinHomogeneousW) return false;
  *wx = (H.at<double>(0, 0) * u + H.at<double>(0, 1) * v + H.at<double>(0, 2)) / w;
  *wy = (H.at<double>(1, 0) * u + H.at<double>(1, 1) * v + H.at<double>(1, 2)) / w;
  return true;
}

}  // namespace

HomographyMapper::HomographyMapper()
    : calib_(NULL), persist_ok_(false), fail_reason_("") {
  persist_dir_[0] = '\0';
  for (int c = 0; c < kChannels; ++c) {
    available_[c] = false;
    last_h_source_[c] = LastHSource::kNone;
    fitted_undistorted_[c] = false;
    // Undistorted by default: it is the space every later step needs, and a
    // lens that has never been configured should not start out in the one
    // mode that makes the marker-plane maths refuse to run.
    coord_undistorted_[c] = true;
    camera_z_mm_[c] = 0.0;
    anchor_n_[c] = 0;
    hm_valid_[c] = false;
    hm_reason_[c] = "이 렌즈에 H가 없습니다";
    pose_ok_[c] = false;
    pose_reason_[c] = "이 렌즈에 H가 없습니다";
    pose_c_[c] = cv::Vec3d(0.0, 0.0, 0.0);
    k_gen_[c] = 0;
    k_stale_[c] = false;
    memset(&fit_[c], 0, sizeof(fit_[c]));
    memset(&result_[c], 0, sizeof(result_[c]));
    // 주행 캘리
    odom_n_[c] = 0;
    odom_active_[c] = false;
    odom_close_have_[c] = false;
    odom_close_u_[c] = odom_close_v_[c] = 0.0;
    odom_close_wx_[c] = odom_close_wy_[c] = 0.0;
    memset(&odom_result_[c], 0, sizeof(odom_result_[c]));
    // 측정 H_marker
    hmm_have_[c] = false;
    hmm_undistorted_[c] = false;
    hmm_k_gen_[c] = 0;
    hmm_k_stale_[c] = false;
    hmm_height_mm_[c] = 0.0;
    hmm_preferred_[c] = false;   // 기본은 파생(체커보드) 쪽
    memset(hmm_[c], 0, sizeof(hmm_[c]));
  }
  marker_height_mm_ = 0.0;
}

void HomographyMapper::Init(const IntrinsicsCalib& calib) {
  calib_ = &calib;
  snprintf(persist_dir_, sizeof(persist_dir_), "%s", calib.PersistDir());
  persist_ok_ = calib.Persistable();

  // Baseline every lens against the K that is loaded RIGHT NOW, before any H
  // comes off disk. A persisted H was fitted against the persisted K, and both
  // files came back together — a start-up is the one moment the two are known
  // to agree, so it must not be reported as a mismatch.
  for (int c = 0; c < kChannels; ++c) {
    k_gen_[c] = calib.KGeneration(c);
    k_stale_[c] = false;
  }

  // Shared marker height first: LoadOne() -> Set() -> RefreshMarkerPlane()
  // derives H_marker, and deriving it against a height of 0 and then never
  // revisiting would leave every lens uncorrected until someone re-entered the
  // number they had already saved.
  LoadMarkerHeightFile();

  for (int c = 0; c < kChannels; ++c) {
    available_[c] = LoadOne(c);
    LoadAnchorsOne(c);  // independent of H: markers may outlive a cleared matrix
  }

  printf("[ArucoPosePNM] homography: persist=%s dir=%s  loaded H:",
         persist_ok_ ? "ok" : "READ-ONLY", persist_dir_);
  for (int c = 0; c < kChannels; ++c) printf(" ch%d=%d", c, (int)available_[c]);
  printf("  anchors:");
  for (int c = 0; c < kChannels; ++c) printf(" ch%d=%d", c, anchor_n_[c]);
  printf("  marker_h=%.1fmm  H_marker:", marker_height_mm_);
  for (int c = 0; c < kChannels; ++c) printf(" ch%d=%d", c, (int)hm_valid_[c]);
  printf("\n");
  fflush(stdout);
}

bool HomographyMapper::Available(int ch) const {
  return ch >= 0 && ch < kChannels && available_[ch];
}

bool HomographyMapper::Get(int ch, double h[9]) const {
  if (!Available(ch) || h == NULL) return false;
  for (int i = 0; i < 9; ++i) h[i] = H_[ch].at<double>(i / 3, i % 3);
  return true;
}

bool HomographyMapper::FittedUndistorted(int ch) const {
  return ch >= 0 && ch < kChannels && fitted_undistorted_[ch];
}

bool HomographyMapper::CoordModeUndistorted(int ch) const {
  return ch >= 0 && ch < kChannels && coord_undistorted_[ch];
}

bool HomographyMapper::SetCoordMode(int ch, bool undistorted) {
  if (ch < 0 || ch >= kChannels) {
    fail_reason_ = "채널 번호가 0..3 범위를 벗어났습니다";
    return false;
  }
  if (coord_undistorted_[ch] == undistorted) {
    fail_reason_ = "";
    return true;  // nothing to do, and nothing to protect
  }
  if (available_[ch]) {
    fail_reason_ = "이 렌즈에 이미 H가 있습니다 — 먼저 HG_CLEAR 후 다시 계산하세요 "
                   "(H는 피팅된 좌표공간에 묶여 있습니다)";
    return false;
  }
  if (undistorted && (calib_ == NULL || !calib_->Available(ch))) {
    fail_reason_ = "이 렌즈에 K/dist가 없습니다 — 먼저 내부 파라미터를 캘리브레이션하세요";
    return false;
  }
  coord_undistorted_[ch] = undistorted;
  fail_reason_ = "";
  return true;
}

/**
 * Undistortion, with a round-trip check that is not optional.
 *
 * cv::undistortPoints inverts the distortion by fixed-point iteration, and
 * that iteration is NOT contractive for every lens. When it diverges, OpenCV
 * does not report failure: on the first iteration whose radial polynomial goes
 * negative it silently resets to the INPUT point and breaks. The caller gets a
 * plausible pixel back, with no error, and that pixel is the distorted one.
 *
 * Measured on this camera's ch1 K/dist (2026-08-04): the iteration diverges on
 * roughly the outer 15% of the frame — the left and right edge columns. That
 * is exactly where the calibration plan requires anchors to be placed
 * ("화면 가장자리까지 퍼뜨릴 것"). Without this check, a fit would silently mix
 * undistorted centre points with raw edge points, and the resulting per-point
 * residual would blame a correctly measured anchor.
 *
 * So: undistort, push the result back through the forward model, and require
 * it to land where it started. The forward model has no iteration and cannot
 * diverge, which is what makes it usable as the judge.
 */
bool HomographyMapper::PreparePixel(int ch, float px, float py, cv::Point2f* out,
                                    const char** reason) const {
  if (reason) *reason = "";
  if (out == NULL || ch < 0 || ch >= kChannels) {
    if (reason) *reason = "채널 번호가 0..3 범위를 벗어났습니다";
    return false;
  }
  if (!coord_undistorted_[ch]) {
    *out = cv::Point2f(px, py);
    return true;
  }
  if (calib_ == NULL || !calib_->Available(ch)) {
    if (reason) *reason = "이 렌즈에 K/dist가 없습니다";
    return false;
  }

  const cv::Mat& K = calib_->KMat(ch);
  const cv::Mat& D = calib_->DistMat(ch);

  // Normalised output (no P), because the round-trip below needs normalised
  // coordinates anyway and applying K by hand afterwards is two multiplies.
  //
  // 20 iterations rather than OpenCV's default 5: on this lens that moves the
  // convergent area from 80% of the frame to 85%, and it saturates there —
  // past 20 nothing more converges, so the rest is genuinely not invertible
  // rather than merely slow.
  // cv::Mat HEADERS over stack points, not std::vector.
  //
  // This function became a frame-path function when the pose packet started
  // carrying world coordinates: two calls per detected marker per frame, on
  // four lenses. The vector form allocated four times per call and OpenCV
  // allocated more inside, which is roughly 1500 mallocs a second on the one
  // thread that also runs detection — against a file whose stated rule is that
  // the frame path does not allocate. A Mat header over an existing point is
  // the same call with the same arguments and no heap at all: OutputArray sees
  // a destination that is already the right size and type, so it writes in
  // place instead of creating.
  cv::Point2f in_pt(px, py), norm_pt;
  const cv::Mat in_m(1, 1, CV_32FC2, &in_pt);
  cv::Mat norm_m(1, 1, CV_32FC2, &norm_pt);
  cv::undistortPoints(in_m, norm_m, K, D, cv::noArray(), cv::noArray(),
                      cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS,
                                       20, 1e-8));

  static const cv::Mat kZero3 = cv::Mat::zeros(3, 1, CV_64F);
  cv::Point3f obj_pt(norm_pt.x, norm_pt.y, 1.0f);
  cv::Point2f back_pt;
  const cv::Mat obj_m(1, 1, CV_32FC3, &obj_pt);
  cv::Mat back_m(1, 1, CV_32FC2, &back_pt);
  cv::projectPoints(obj_m, kZero3, kZero3, K, D, back_m);
  const double dx = back_pt.x - px, dy = back_pt.y - py;
  if (dx * dx + dy * dy > kMaxRoundTripPx * kMaxRoundTripPx) {
    if (reason)
      *reason = "왜곡보정이 수렴하지 않는 영역입니다 (화면 좌우 가장자리) — "
                "이 점은 쓸 수 없습니다";
    return false;
  }

  out->x = (float)(K.at<double>(0, 0) * norm_pt.x + K.at<double>(0, 2));
  out->y = (float)(K.at<double>(1, 1) * norm_pt.y + K.at<double>(1, 2));
  return true;
}

bool HomographyMapper::PixelToWorld(int ch, float px, float py, double* wx, double* wy,
                                    const char** reason) const {
  if (reason) *reason = "";
  if (wx == NULL || wy == NULL) return false;
  if (!Available(ch)) {
    if (reason) *reason = "이 렌즈에 H가 없습니다";
    return false;
  }
  cv::Point2f p;
  if (!PreparePixel(ch, px, py, &p, reason)) return false;
  if (!MapThrough(H_[ch], p.x, p.y, wx, wy)) {
    if (reason) *reason = "지평선 (매핑 불가)";
    return false;
  }
  return true;
}

bool HomographyMapper::WorldToPixel(int ch, double wx, double wy, float* px, float* py,
                                    const char** reason) const {
  if (reason) *reason = "";
  if (px == NULL || py == NULL) return false;
  if (!Available(ch)) {
    if (reason) *reason = "이 렌즈에 H가 없습니다";
    return false;
  }
  // Matx33d rather than cv::Mat's own .inv()/determinant(): DecomposeMatrix
  // already goes through Matx33d for the same reason (a fixed 3x3 has a
  // closed-form inverse, no LU/SVD dispatch), and this file has exactly one
  // way to invert a 3x3 rather than two that could silently drift apart.
  const cv::Matx33d Hm(H_[ch].at<double>(0, 0), H_[ch].at<double>(0, 1), H_[ch].at<double>(0, 2),
                       H_[ch].at<double>(1, 0), H_[ch].at<double>(1, 1), H_[ch].at<double>(1, 2),
                       H_[ch].at<double>(2, 0), H_[ch].at<double>(2, 1), H_[ch].at<double>(2, 2));
  if (std::fabs(cv::determinant(Hm)) < 1e-18) {
    if (reason) *reason = "H가 특이(singular)합니다";
    return false;
  }
  const cv::Mat Hinv(Hm.inv());
  double u, v;
  if (!MapThrough(Hinv, wx, wy, &u, &v)) {
    if (reason) *reason = "역변환이 지평선에 걸립니다";
    return false;
  }
  return PreparedToRawPixel(ch, u, v, px, py, reason);
}

bool HomographyMapper::PreparedToRawPixel(int ch, double u, double v, float* px, float* py,
                                          const char** reason) const {
  if (!coord_undistorted_[ch]) {
    *px = (float)u;
    *py = (float)v;
    return true;
  }
  if (calib_ == NULL || !calib_->Available(ch)) {
    if (reason) *reason = "이 렌즈에 K/dist가 없습니다";
    return false;
  }
  // Undistorted fitting-space pixel -> raw sensor pixel: normalise by K^-1,
  // then re-apply distortion with cv::projectPoints (the same forward model
  // PreparePixel()'s round-trip check uses, run here as the actual answer
  // rather than a check).
  const cv::Mat& K = calib_->KMat(ch);
  const cv::Mat& D = calib_->DistMat(ch);
  cv::Point3f obj_pt((float)((u - K.at<double>(0, 2)) / K.at<double>(0, 0)),
                     (float)((v - K.at<double>(1, 2)) / K.at<double>(1, 1)),
                     1.0f);
  cv::Point2f raw_pt;
  static const cv::Mat kZero3 = cv::Mat::zeros(3, 1, CV_64F);
  const cv::Mat obj_m(1, 1, CV_32FC3, &obj_pt);
  cv::Mat raw_m(1, 1, CV_32FC2, &raw_pt);
  cv::projectPoints(obj_m, kZero3, kZero3, K, D, raw_m);
  *px = raw_pt.x;
  *py = raw_pt.y;
  return true;
}

HomographyMapper::LastHSource HomographyMapper::LastHomographySource(int ch) const {
  if (ch < 0 || ch >= kChannels) return LastHSource::kNone;
  return last_h_source_[ch];
}

bool HomographyMapper::OdomPointRawPixel(int ch, int i, float* px, float* py,
                                         const char** reason) const {
  if (reason) *reason = "";
  if (px == NULL || py == NULL) return false;
  if (ch < 0 || ch >= kChannels || i < 0 || i >= odom_n_[ch]) {
    if (reason) *reason = "지점 인덱스가 범위 밖입니다";
    return false;
  }
  return PreparedToRawPixel(ch, odom_sx_[ch][i], odom_sy_[ch][i], px, py, reason);
}

double HomographyMapper::CameraHeightMm(int ch) const {
  return (ch >= 0 && ch < kChannels) ? camera_z_mm_[ch] : 0.0;
}

// --- marker plane ------------------------------------------------------------

double HomographyMapper::MarkerHeightMm() const { return marker_height_mm_; }

bool HomographyMapper::SetMarkerHeightMm(double mm) {
  if (!ValidHeightMm(mm)) {
    fail_reason_ = "마커 높이가 0..100000 mm 범위를 벗어났습니다";
    return false;
  }
  marker_height_mm_ = mm;
  // Shared value, so every lens's cached matrix is now stale.
  for (int c = 0; c < kChannels; ++c) RefreshMarkerPlane(c);
  fail_reason_ = "";
  return true;
}

bool HomographyMapper::SetCameraHeightMm(int ch, double mm) {
  if (ch < 0 || ch >= kChannels) {
    fail_reason_ = "채널 번호가 0..3 범위를 벗어났습니다";
    return false;
  }
  if (!ValidHeightMm(mm)) {
    fail_reason_ = "카메라 높이가 0..100000 mm 범위를 벗어났습니다 (0 = 미측정, 분해값 사용)";
    return false;
  }
  camera_z_mm_[ch] = mm;
  RefreshMarkerPlane(ch);
  fail_reason_ = "";
  return true;
}

bool HomographyMapper::MarkerPlaneReady(int ch) const {
  if (ch < 0 || ch >= kChannels) return false;
  SyncCalib(ch);  // PixelToWorldMarker() and GetMarkerPlane() enter through here
  return hm_valid_[ch];
}

const char* HomographyMapper::MarkerPlaneReason(int ch) const {
  if (ch < 0 || ch >= kChannels) return "채널 번호가 0..3 범위를 벗어났습니다";
  SyncCalib(ch);
  return hm_reason_[ch];
}

bool HomographyMapper::GetMarkerPlane(int ch, double h[9]) const {
  if (!MarkerPlaneReady(ch) || h == NULL) return false;
  for (int i = 0; i < 9; ++i) h[i] = Hm_[ch].at<double>(i / 3, i % 3);
  return true;
}

/**
 * The shared front half of the marker plane and the camera pose.
 *
 * Ported from cctv_app's decompose(). The two refusals below are the reason it
 * is a separate function rather than inline in either caller: both callers must
 * refuse identically, and a check that exists in one copy and not the other is
 * how a raw-fitted matrix eventually gets through.
 */
/**
 * 픽셀->월드 H 하나와 K 로 외부 파라미터를 푼다.
 *
 * 어느 평면의 H 인지는 상관없다 — 바닥이면 결과가 바닥 기준이고, 마커평면이면
 * 마커평면 기준이다. 그래서 이 함수는 채널 상태를 안 본다. 게이트(H 존재,
 * 피팅 공간, K 유무)는 호출부가 각자 건다.
 */
bool HomographyMapper::DecomposeMatrix(const cv::Matx33d& Hp2w, const cv::Matx33d& K,
                                       cv::Vec3d* r1, cv::Vec3d* r2, cv::Vec3d* r3,
                                       cv::Vec3d* t, const char** reason) const {
  if (reason) *reason = "";
  if (std::fabs(cv::determinant(Hp2w)) < 1e-18) {
    if (reason) *reason = "H가 특이(singular)합니다";
    return false;
  }
  const cv::Matx33d Hw2p = Hp2w.inv();       // world mm -> pixel
  const cv::Matx33d B    = K.inv() * Hw2p;   // == [r1 r2 t] up to scale

  const cv::Vec3d b1(B(0, 0), B(1, 0), B(2, 0));
  const cv::Vec3d b2(B(0, 1), B(1, 1), B(2, 1));
  const cv::Vec3d b3(B(0, 2), B(1, 2), B(2, 2));

  const double n1 = cv::norm(b1), n2 = cv::norm(b2);
  if (n1 < 1e-12 || n2 < 1e-12) {
    if (reason) *reason = "분해가 퇴화했습니다 (degenerate)";
    return false;
  }
  // Homogeneity loses one scale factor. Averaging the two column norms is
  // steadier than trusting either alone — neither is exactly unit once fitting
  // noise is in the matrix.
  double lambda = 2.0 / (n1 + n2);
  // The sign is lost too. The camera must be on the +Z side of the floor, so
  // t_z > 0; otherwise the whole solution is mirrored and the correction would
  // push the marker the wrong way.
  if (b3[2] < 0.0) lambda = -lambda;

  *r1 = b1 * lambda;
  *r2 = b2 * lambda;
  *t  = b3 * lambda;
  *r3 = (*r1).cross(*r2);

  const double n3 = cv::norm(*r3);
  if (n3 < 1e-9) {
    if (reason) *reason = "회전 열벡터가 평행합니다";
    return false;
  }
  *r3 /= n3;  // re-normalise
  return true;
}

/**
 * 이 렌즈의 K 를 Matx33d 로. 없으면 false.
 */
bool HomographyMapper::IntrinsicMatrix(int ch, cv::Matx33d* K, const char** reason) const {
  if (calib_ == NULL || !calib_->Available(ch)) {
    if (reason) *reason = "이 렌즈에 K/dist가 없습니다";
    return false;
  }
  double fx, fy, cx, cy, d[5];
  if (!calib_->Get(ch, &fx, &fy, &cx, &cy, d)) {
    if (reason) *reason = "K를 읽지 못했습니다";
    return false;
  }
  *K = cv::Matx33d(fx, 0.0, cx,
                   0.0, fy, cy,
                   0.0, 0.0, 1.0);
  return true;
}

bool HomographyMapper::Decompose(int ch, cv::Matx33d* K, cv::Vec3d* r1, cv::Vec3d* r2,
                                 cv::Vec3d* r3, cv::Vec3d* t, const char** reason) const {
  if (reason) *reason = "";
  if (!Available(ch)) {
    if (reason) *reason = "이 렌즈에 H가 없습니다";
    return false;
  }
  // Distortion would be absorbed into R and t, tilting r3 — and r3 IS the
  // correction. Refuse rather than emit a plausible-looking wrong answer.
  if (!fitted_undistorted_[ch]) {
    if (reason)
      *reason = "H가 raw 픽셀로 피팅돼 있습니다 — 시차 보정은 undistort 공간에서만 성립합니다 "
                "(HG_CLEAR 후 HG_COORD_MODE 1 로 다시 계산하세요)";
    return false;
  }
  if (!IntrinsicMatrix(ch, K, reason)) return false;

  const cv::Mat& H = H_[ch];
  const cv::Matx33d Hp2w(H.at<double>(0, 0), H.at<double>(0, 1), H.at<double>(0, 2),
                         H.at<double>(1, 0), H.at<double>(1, 1), H.at<double>(1, 2),
                         H.at<double>(2, 0), H.at<double>(2, 1), H.at<double>(2, 2));
  return DecomposeMatrix(Hp2w, *K, r1, r2, r3, t, reason);
}

/**
 * 측정된 H_marker 가 함의하는 카메라 위치 — **마커평면 기준**.
 *
 * CameraPose() 와 같은 계산이지만 분해 대상이 바닥 H 가 아니라 주행으로 잰
 * H_marker 다. 이게 따로 필요한 이유가 둘 있다:
 *
 *  1. **독립성** — CameraPose() 는 RefreshMarkerPlane() 의 `Available(ch)` 게이트
 *     뒤에 있어서 바닥 H(floor 탭)가 없으면 아예 안 나온다. 주행 캘리만 돌린
 *     렌즈에서는 H_floor 역산이 통째로 막힌다.
 *  2. **좌표계** — 더 중요한 쪽이다. 바닥 H 의 월드 프레임과 주행 캘리의 월드
 *     프레임은 **원점이 다르다**(주행 쪽 원점은 그 세션 로봇 출발점이다, wire
 *     스펙 §6). 바닥 H 에서 얻은 나디르 (x,y) 를 주행 H 에 그대로 쓰면 엉뚱한
 *     점을 중심으로 확대하게 된다 — 결과는 그럴듯하고 조용히 틀린 H_floor 다.
 *
 * 반환하는 높이는 **마커평면 위 높이**다. 바닥 위 높이는 여기에 마커 높이를
 * 더한 값이고, 그 덧셈은 이 값을 쓰는 쪽에서 한다(마커 높이가 0 인 설치도
 * 있어서 여기서 미리 더해두면 "무엇 기준인가"가 흐려진다).
 */
bool HomographyMapper::MeasuredCameraPose(int ch, double* height_above_marker_mm,
                                          double* nadir_x_mm, double* nadir_y_mm,
                                          const char** reason) const {
  if (reason) *reason = "";
  if (ch < 0 || ch >= kChannels) {
    if (reason) *reason = "채널 번호가 0..3 범위를 벗어났습니다";
    return false;
  }
  SyncCalib(ch);
  if (!hmm_have_[ch]) {
    if (reason) *reason = "측정된 H_marker 가 없습니다";
    return false;
  }
  // Decompose() 와 같은 이유로 raw 는 거부한다 — 왜곡이 R/t 에 흡수되면 r3 가
  // 기울고, r3 가 바로 우리가 구하려는 보정이다.
  if (!hmm_undistorted_[ch]) {
    if (reason) *reason = "측정 H_marker 가 raw 픽셀로 피팅돼 있습니다";
    return false;
  }
  cv::Matx33d K;
  if (!IntrinsicMatrix(ch, &K, reason)) return false;

  const cv::Matx33d Hp2w(hmm_[ch][0], hmm_[ch][1], hmm_[ch][2],
                         hmm_[ch][3], hmm_[ch][4], hmm_[ch][5],
                         hmm_[ch][6], hmm_[ch][7], hmm_[ch][8]);
  cv::Vec3d r1, r2, r3, t;
  if (!DecomposeMatrix(Hp2w, K, &r1, &r2, &r3, &t, reason)) return false;

  const cv::Matx33d R(r1[0], r2[0], r3[0],
                      r1[1], r2[1], r3[1],
                      r1[2], r2[2], r3[2]);
  const cv::Vec3d c = -(R.t() * t);
  if (!(c[2] > 0.0)) {
    if (reason) *reason = "카메라 높이가 0 이하로 나왔습니다 (분해 실패)";
    return false;
  }
  if (height_above_marker_mm) *height_above_marker_mm = c[2];
  if (nadir_x_mm) *nadir_x_mm = c[0];
  if (nadir_y_mm) *nadir_y_mm = c[1];
  return true;
}

/**
 * Reads the cache. Does NOT decompose.
 *
 * /status calls this for all four lenses on every poll, and a poll is once a
 * second with the dashboard open. Decomposing there meant eight 3x3 inversions
 * per second re-deriving numbers RefreshMarkerPlane() had already computed —
 * on the thread the frame path shares. The inputs (H, K, the two heights)
 * change only on an operator action, which is exactly when the cache is
 * rebuilt.
 */
bool HomographyMapper::CameraPose(int ch, double* height_mm, double* nadir_x_mm,
                                  double* nadir_y_mm, const char** reason) const {
  if (reason) *reason = "";
  if (ch < 0 || ch >= kChannels) {
    if (reason) *reason = "채널 번호가 0..3 범위를 벗어났습니다";
    return false;
  }
  // The pose is the diagnostic an installer uses to judge a FRESH K, so it is
  // the last number in the app that may be allowed to describe the old one.
  SyncCalib(ch);
  if (!pose_ok_[ch]) {
    if (reason) *reason = pose_reason_[ch];
    return false;
  }
  if (height_mm)  *height_mm  = pose_c_[ch][2];
  if (nadir_x_mm) *nadir_x_mm = pose_c_[ch][0];
  if (nadir_y_mm) *nadir_y_mm = pose_c_[ch][1];
  return true;
}

/**
 * Rebuild this lens's cached H_marker. Never fails loudly — it records why.
 *
 * Called from everything that changes H, the marker height or the camera
 * height. Recomputing rather than invalidating-and-deriving-on-demand keeps the
 * frame path free of the 3x3 inverse, and there is no path that reads Hm_
 * without one of those three having been set first.
 */
void HomographyMapper::RefreshMarkerPlane(int ch) const {
  if (ch < 0 || ch >= kChannels) return;
  hm_valid_[ch] = false;
  hm_reason_[ch] = "";
  pose_ok_[ch] = false;
  pose_reason_[ch] = "";
  Hm_[ch].release();

  if (!Available(ch)) {
    hm_reason_[ch] = "이 렌즈에 H가 없습니다";
    pose_reason_[ch] = hm_reason_[ch];
    return;
  }

  // Decompose FIRST, and unconditionally.
  //
  // The camera pose is a diagnostic the page shows whether or not a marker
  // height has been entered — it is how an installer checks that the
  // decomposition is sane before trusting anything derived from it. Deriving it
  // only on the marker-height path would leave the pose blank in exactly the
  // state where somebody is deciding whether to enter a height.
  cv::Matx33d K;
  cv::Vec3d r1, r2, r3, t;
  const char* why = "";
  const bool decomposed = Decompose(ch, &K, &r1, &r2, &r3, &t, &why);
  if (decomposed) {
    // Camera centre in world coordinates: C = -R^T t, with R = [r1 r2 r3].
    const cv::Matx33d R(r1[0], r2[0], r3[0],
                        r1[1], r2[1], r3[1],
                        r1[2], r2[2], r3[2]);
    pose_c_[ch] = -(R.t() * t);
    pose_ok_[ch] = true;
  } else {
    pose_reason_[ch] = why;
  }

  // A marker genuinely on the floor needs no correction, and deriving one would
  // only inject decomposition noise into a matrix that is already right. Note
  // this runs even when the decomposition failed: a raw-fitted H still maps the
  // floor correctly, it just cannot be lifted to another plane.
  if (marker_height_mm_ == 0.0) {
    Hm_[ch] = H_[ch].clone();
    hm_valid_[ch] = true;
    return;
  }
  if (!decomposed) {
    hm_reason_[ch] = why;
    return;
  }

  // Apply a tape-measured camera height, if one was supplied.
  //
  // The parallax is governed by the RATIO h / Cz, and Cz here is whatever the
  // decomposition produced — its scale is exactly the part residual distortion
  // corrupts. The obvious fix, rescaling t so the camera lands at the measured
  // height, is WRONG: t is what reproduces the fitted floor H, and moving it
  // makes the derived matrices disagree with H_floor itself.
  //
  // Scaling the marker height instead leaves t (and therefore H_floor)
  // untouched while producing exactly the intended ratio:
  //
  //     h_eff / Cz_derived  ==  h_measured / Cz_measured
  double effective_h = marker_height_mm_;
  if (camera_z_mm_[ch] > 0.0) {
    // The same number /status shows as derived_z_mm. Recomputing it here would
    // let the height the page reports drift from the height the correction is
    // actually scaled against.
    const double cz_derived = pose_c_[ch][2];
    if (std::fabs(cz_derived) < 1e-6) {
      hm_reason_[ch] = "분해된 카메라 높이가 0에 가까워 보정 배율을 낼 수 없습니다";
      return;
    }
    effective_h = marker_height_mm_ * (cz_derived / camera_z_mm_[ch]);
  }

  // Same camera, plane shifted along its own normal by the marker height.
  const cv::Vec3d tm = t + r3 * effective_h;
  const cv::Matx33d Hm_w2p = K * cv::Matx33d(r1[0], r2[0], tm[0],
                                             r1[1], r2[1], tm[1],
                                             r1[2], r2[2], tm[2]);
  if (std::fabs(cv::determinant(Hm_w2p)) < 1e-18) {
    hm_reason_[ch] = "유도된 마커 평면 행렬이 특이합니다";
    return;
  }
  const cv::Matx33d Hm = Hm_w2p.inv();  // back to pixel -> world mm

  // Normalise so h22 == 1. The server stores and compares these, and an
  // arbitrary scale makes two mathematically identical matrices look different
  // in a log.
  const double s = (std::fabs(Hm(2, 2)) > 1e-18) ? 1.0 / Hm(2, 2) : 1.0;
  Hm_[ch] = cv::Mat(3, 3, CV_64F);
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c) Hm_[ch].at<double>(r, c) = Hm(r, c) * s;
  hm_valid_[ch] = true;
}

bool HomographyMapper::PixelToWorldMarker(int ch, float px, float py, double* wx,
                                          double* wy, const char** reason) const {
  if (reason) *reason = "";
  if (wx == NULL || wy == NULL) return false;

  // 운영자가 명시적으로 고른 경우에만 측정값(주행 캘리)을 쓴다. 기본은 파생값
  // (체커보드에서 시차 보정으로 얻은 Hm_)이다 — 주행 방식은 아직 균일 스케일
  // 오차를 검증할 수단이 없으므로, 검증된 쪽을 기본으로 두고 전환을 의도적인
  // 행동으로 만든다. 전환 여부는 /status 에 드러난다.
  const bool use_measured = (ch >= 0 && ch < kChannels) && hmm_preferred_[ch] && hmm_have_[ch];
  if (!use_measured && !MarkerPlaneReady(ch)) {
    if (reason) *reason = hm_reason_[ch >= 0 && ch < kChannels ? ch : 0];
    return false;
  }
  cv::Point2f p;
  if (!PreparePixel(ch, px, py, &p, reason)) return false;
  cv::Mat Hm = use_measured
             ? cv::Mat(3, 3, CV_64F, const_cast<double*>(hmm_[ch]))
             : Hm_[ch];
  if (!MapThrough(Hm, p.x, p.y, wx, wy)) {
    if (reason) *reason = "지평선 (매핑 불가)";
    return false;
  }
  return true;
}

bool HomographyMapper::SaveMarkerPlane(int ch) {
  if (ch < 0 || ch >= kChannels) {
    fail_reason_ = "채널 번호가 0..3 범위를 벗어났습니다";
    return false;
  }
  if (!persist_ok_) {
    fail_reason_ = "저장 위치에 쓸 수 없습니다 (/status 의 calib.persist 확인)";
    return false;
  }
  // Two files on purpose: the marker height is shared by every lens, the camera
  // height belongs to this one and travels in its homography file. Writing the
  // shared value into four per-lens files would create four copies that can
  // disagree, and the one that wins would depend on load order.
  if (!SaveMarkerHeightFile()) {
    fail_reason_ = "marker_plane.txt 쓰기 실패";
    return false;
  }
  // Only if there is an H to write it alongside. Cz with no H is not a state
  // worth a file — the next fit will ask for it again anyway.
  if (Available(ch) && !SaveOne(ch)) {
    fail_reason_ = "homography_ch<N>.txt 쓰기 실패 (카메라 높이가 저장되지 않았습니다)";
    return false;
  }
  fail_reason_ = "";
  return true;
}

/**
 * Validate the whole list, then commit it. Never the other way round.
 *
 * The checks below all catch the same shape of mistake: a list that is
 * well-formed, accepted, and wrong. A duplicated id makes one marker's pixel
 * observation fight another's world coordinate and the fit splits the
 * difference; a pasted row puts two world points on top of each other and
 * quietly removes a degree of freedom from the fit. Neither produces an error
 * later — they produce a homography with a slightly worse residual spread
 * across every point, which is the hardest kind of fault to trace back.
 */
bool HomographyMapper::SetAnchors(int ch, const Anchor* list, int n) {
  if (ch < 0 || ch >= kChannels) {
    fail_reason_ = "채널 번호가 0..3 범위를 벗어났습니다";
    return false;
  }
  if (n < 0 || n > kMaxAnchors) {
    fail_reason_ = "마커 개수가 0..24 범위를 벗어났습니다";
    return false;
  }
  if (n > 0 && list == NULL) {
    fail_reason_ = "마커 목록이 비어 있습니다";
    return false;
  }

  static char why[160];
  for (int i = 0; i < n; ++i) {
    if (list[i].id < 0) {
      snprintf(why, sizeof(why), "%d번째 마커의 id가 음수입니다", i + 1);
      fail_reason_ = why;
      return false;
    }
    if (!std::isfinite(list[i].wx_mm) || !std::isfinite(list[i].wy_mm)) {
      snprintf(why, sizeof(why), "id %d 의 월드 좌표가 숫자가 아닙니다", list[i].id);
      fail_reason_ = why;
      return false;
    }
    for (int k = 0; k < i; ++k) {
      if (list[k].id == list[i].id) {
        snprintf(why, sizeof(why),
                 "id %d 가 %d번째와 %d번째에 중복으로 들어 있습니다",
                 list[i].id, k + 1, i + 1);
        fail_reason_ = why;
        return false;
      }
      const double dx = list[k].wx_mm - list[i].wx_mm;
      const double dy = list[k].wy_mm - list[i].wy_mm;
      if (dx * dx + dy * dy < kMinAnchorSeparationMm * kMinAnchorSeparationMm) {
        snprintf(why, sizeof(why),
                 "id %d 와 id %d 의 월드 좌표가 같습니다 (%.1f, %.1f) — 행 복사 실수로 보입니다",
                 list[k].id, list[i].id, list[i].wx_mm, list[i].wy_mm);
        fail_reason_ = why;
        return false;
      }
    }
  }

  for (int i = 0; i < n; ++i) anchors_[ch][i] = list[i];
  anchor_n_[ch] = n;

  // Anything derived from the old list is now about markers that are not
  // registered any more. An open session is abandoned rather than adapted: its
  // accumulated sums are indexed by the positions that just changed, so
  // carrying it on would average one marker's pixels against another's world
  // coordinate — plausible input, silently wrong H.
  if (fit_[ch].active) {
    memset(&fit_[ch], 0, sizeof(fit_[ch]));
    CopyUtf8(fit_[ch].last_result, sizeof(fit_[ch].last_result),
             "마커 목록이 바뀌어 수집 세션을 중단했습니다");
  }
  memset(&result_[ch], 0, sizeof(result_[ch]));
  fail_reason_ = "";
  return true;
}

int HomographyMapper::AnchorCount(int ch) const {
  return (ch >= 0 && ch < kChannels) ? anchor_n_[ch] : 0;
}

bool HomographyMapper::AnchorAt(int ch, int i, Anchor* out) const {
  if (out == NULL || ch < 0 || ch >= kChannels) return false;
  if (i < 0 || i >= anchor_n_[ch]) return false;
  *out = anchors_[ch][i];
  return true;
}

// --- collection session + fit ----------------------------------------------

bool HomographyMapper::StartFit(int ch) {
  if (ch < 0 || ch >= kChannels) {
    fail_reason_ = "채널 번호가 0..3 범위를 벗어났습니다";
    return false;
  }
  // A refusal is recorded in last_result, not only returned.
  //
  // /status is the only answer POST /cmd gives, so without this a refused
  // CALIB_START is indistinguishable from an accepted one that has not
  // collected a frame yet: collecting stays false, the counters stay at zero,
  // and the reason is on a stdout nobody driving this by curl can read.
  if (anchor_n_[ch] < kMinFitAnchors) {
    static char why[128];
    snprintf(why, sizeof(why),
             "등록된 마커가 %d개뿐입니다 — 최소 4개(권장 10~20개)가 필요합니다",
             anchor_n_[ch]);
    fail_reason_ = why;
    CopyUtf8(fit_[ch].last_result, sizeof(fit_[ch].last_result), why);
    return false;
  }
  // A K/dist session on this same lens takes the frame and breaks before the
  // ArUco pass (see ProcessRawVideo), so FeedFrame() is never reached.
  //
  // And FeedFrame() is what drives BOTH counters — `good` and `total` — so a
  // collection opened underneath a board session does not fail slowly, it
  // stops: 0 of 20, 0 of 200, `collecting:true`, forever. The 200-frame
  // ceiling cannot save it either, because that ceiling is counted in frames
  // this session never receives. There is no cancel command by design, so the
  // only way out would be to notice the board session and stop it — which is
  // exactly what the operator cannot see, the two living on different tabs.
  if (calib_ != NULL && calib_->Collecting(ch)) {
    fail_reason_ = "이 렌즈는 K/dist 캘리브레이션 세션 중입니다 "
                   "— 캘리브레이션 탭에서 먼저 끝내거나 취소(CALIB_K_STOP)하세요";
    CopyUtf8(fit_[ch].last_result, sizeof(fit_[ch].last_result), fail_reason_);
    return false;
  }
  // Checked now, not at the fit. Undistortion failing after 200 frames of
  // collection tells the operator the same thing five minutes later.
  if (coord_undistorted_[ch] && (calib_ == NULL || !calib_->Available(ch))) {
    fail_reason_ = "이 렌즈는 undistort 좌표로 피팅하도록 설정돼 있는데 K/dist가 없습니다 "
                   "— 내부 파라미터를 먼저 캘리브레이션하거나 HG_COORD_MODE 로 raw 로 바꾸세요";
    CopyUtf8(fit_[ch].last_result, sizeof(fit_[ch].last_result), fail_reason_);
    return false;
  }

  memset(&fit_[ch], 0, sizeof(fit_[ch]));
  fit_[ch].active = true;
  for (int i = 0; i < kMaxAnchors; ++i) sum_x_[ch][i] = sum_y_[ch][i] = 0.0;
  fail_reason_ = "";
  return true;
}

bool HomographyMapper::FitCollecting(int ch) const {
  return ch >= 0 && ch < kChannels && fit_[ch].active;
}

const HomographyMapper::FitProgress& HomographyMapper::Fit(int ch) const {
  static const FitProgress kNone = FitProgress();
  if (ch < 0 || ch >= kChannels) return kNone;
  return fit_[ch];
}

const HomographyMapper::FitResult& HomographyMapper::Residuals(int ch) const {
  static const FitResult kNone = FitResult();
  if (ch < 0 || ch >= kChannels) return kNone;
  return result_[ch];
}

void HomographyMapper::FeedFrame(int ch, const int* ids, const float* cx, const float* cy,
                                 int n) {
  if (ch < 0 || ch >= kChannels || !fit_[ch].active) return;

  FitProgress& f = fit_[ch];
  ++f.total;
  f.seen_n = f.missing_n = f.unusable_n = 0;

  // One pass per registered marker rather than per detection: the answer being
  // built is "which of MY markers showed up", and a frame full of markers from
  // some other part of the site should not be able to affect it.
  double px[kMaxAnchors], py[kMaxAnchors];
  for (int a = 0; a < anchor_n_[ch]; ++a) {
    int hit = -1;
    for (int d = 0; d < n; ++d) {
      if (ids[d] == anchors_[ch][a].id) {
        hit = d;
        break;
      }
    }
    if (hit < 0) {
      f.missing_ids[f.missing_n++] = anchors_[ch][a].id;
      continue;
    }
    cv::Point2f p;
    if (!PreparePixel(ch, cx[hit], cy[hit], &p)) {
      // Detected but not mappable — a different problem from "not there", and
      // the operator needs to be told which one it is.
      f.unusable_ids[f.unusable_n++] = anchors_[ch][a].id;
      continue;
    }
    f.seen_ids[f.seen_n++] = anchors_[ch][a].id;
    px[a] = p.x;
    py[a] = p.y;
  }

  // All or nothing. A frame contributing only the markers it happened to see
  // would weight each point by how often it was visible, which quietly favours
  // the middle of the frame — the opposite of what the placement advice is
  // trying to achieve.
  // seen_n == anchor_n_ is exactly "every anchor filled px/py above", so no
  // per-anchor validity flag is needed here — one would be a second
  // representation of the same fact, free to drift from it.
  if (f.seen_n == anchor_n_[ch]) {
    for (int a = 0; a < anchor_n_[ch]; ++a) {
      sum_x_[ch][a] += px[a];
      sum_y_[ch][a] += py[a];
    }
    ++f.good;
  }

  if (f.good >= kFitTargetFrames) {
    const bool ok = FinishFit(ch);
    f.active = false;
    CopyUtf8(f.last_result, sizeof(f.last_result), ok ? "완료" : fail_reason_);
    return;
  }
  if (f.total >= kFitMaxFrames) {
    f.active = false;
    CopyUtf8(f.last_result, sizeof(f.last_result),
             "시간 초과 — 모든 마커가 함께 보인 프레임이 부족했습니다 (missing_ids 확인)");
  }
}

/**
 * Least squares, never RANSAC.
 *
 * cctv_app fitted with RANSAC, and copying that here would break the residuals
 * outright: RANSAC picks its inlier set probabilistically, so each LOO fold
 * would draw a different one, and the same data would produce different
 * residuals on every run. There would be nothing to compare, and no way to tell
 * an improvement from a reroll.
 *
 * The thing RANSAC was there to do — throw out a badly measured point — is done
 * instead by naming that point in the residual table and letting a person go
 * and re-measure it. Discarding it silently leaves the tape measure wrong
 * forever, and the next calibration inherits the same mistake.
 */
/**
 * 준비된 픽셀 합 / 대응 월드점 한 벌에서 H 하나.
 *
 * 정적 앵커(FitSubset)와 주행 캘리(FitOdomSubset)가 같은 최소제곱을 쓰도록
 * 점 출처만 인자로 뺐다(2026-08-12). 두 벌로 복사하면 "RANSAC 을 쓰지 않는다"
 * 같은 판단이 한쪽에만 남고, LOO 잔차의 의미도 조용히 갈라진다.
 *
 * `inv` 는 sum 을 평균으로 되돌리는 계수다. 정적 앵커는 1/good(모든 앵커가 같은
 * 프레임 수만큼 누적된다), 주행 캘리는 지점마다 표본 수가 달라 수집 단계에서
 * 이미 평균을 넣어 두므로 1.0 이다.
 */
bool HomographyMapper::FitPoints(const double* sum_x, const double* sum_y, double inv,
                                 const Anchor* dst_pts, int n, int skip,
                                 cv::Mat* out) const {
  std::vector<cv::Point2f> src;
  std::vector<cv::Point2f> dst;
  for (int a = 0; a < n; ++a) {
    if (a == skip) continue;
    src.push_back(cv::Point2f((float)(sum_x[a] * inv), (float)(sum_y[a] * inv)));
    dst.push_back(cv::Point2f((float)dst_pts[a].wx_mm, (float)dst_pts[a].wy_mm));
  }
  if ((int)src.size() < kMinFitAnchors) return false;
  cv::Mat h = cv::findHomography(src, dst, 0);
  if (h.empty() || h.rows != 3 || h.cols != 3) return false;
  h.convertTo(*out, CV_64F);
  return true;
}

bool HomographyMapper::FitSubset(int ch, int skip, cv::Mat* out) const {
  const double inv = (fit_[ch].good > 0) ? 1.0 / (double)fit_[ch].good : 0.0;
  return FitPoints(sum_x_[ch], sum_y_[ch], inv, anchors_[ch], anchor_n_[ch], skip, out);
}

bool HomographyMapper::FitOdomSubset(int ch, int skip, cv::Mat* out) const {
  // inv = 1: 지점별 평균은 AddOdomPoint() 가 넣을 때 이미 계산돼 있다.
  return FitPoints(odom_sx_[ch], odom_sy_[ch], 1.0, odom_[ch], odom_n_[ch], skip, out);
}

/**
 * Fit on everything, then measure honestly.
 *
 * The final matrix uses all N points — holding some back as a validation set
 * would mean shipping a homography fitted from fewer points than were measured,
 * which is a real accuracy cost paid for a number that leave-one-out provides
 * for free.
 *
 * Two residuals per point, not one, because either alone is misleading:
 *
 *   in-sample large, LOO large  -> the point is measured wrong. Re-measure it.
 *   in-sample small, LOO large  -> the point is on the outside of the cluster.
 *                                  Removing it leaves nothing to bracket its
 *                                  position, so the fold extrapolates. Normal.
 *   both small                  -> good.
 *   in-sample large, LOO small  -> essentially does not happen. When it does,
 *                                  suspect a duplicated id or swapped x/y.
 *
 * With only the in-sample column, the second row looks like the first and a
 * correctly measured edge marker gets "corrected" until it agrees with a fit
 * that was wrong. That is the specific mistake this function exists to prevent.
 */
bool HomographyMapper::FinishFit(int ch) {
  const int n = anchor_n_[ch];
  cv::Mat H;
  if (!FitSubset(ch, -1, &H)) {
    fail_reason_ = "호모그래피 계산 실패 — 마커들이 한 직선 위에 있거나 너무 몰려 있습니다";
    return false;
  }

  double h[9];
  for (int i = 0; i < 9; ++i) h[i] = H.at<double>(i / 3, i % 3);
  // Through Set() so the singularity and finiteness gates apply to a computed
  // matrix exactly as they do to an injected one. A fit CAN diverge, and this
  // is where that gets caught rather than at the first pose packet.
  if (!Set(ch, h, coord_undistorted_[ch])) return false;
  last_h_source_[ch] = LastHSource::kFloor;

  ComputeResiduals(H, sum_x_[ch], sum_y_[ch],
                   (fit_[ch].good > 0) ? 1.0 / (double)fit_[ch].good : 0.0,
                   anchors_[ch], n, &HomographyMapper::FitSubset, ch, &result_[ch]);
  fail_reason_ = "";
  return true;
}

/**
 * in-sample / LOO 잔차 표. 정적 앵커와 주행 캘리가 공유한다.
 *
 * `subset` 은 "이 점 하나를 빼고 다시 피팅" 하는 함수다 — 점 출처가 달라도
 * 잔차의 정의는 하나여야 하므로 호출만 갈아끼운다.
 */
void HomographyMapper::ComputeResiduals(
    const cv::Mat& H, const double* sum_x, const double* sum_y, double inv,
    const Anchor* dst_pts, int n,
    bool (HomographyMapper::*subset)(int, int, cv::Mat*) const,
    int ch, FitResult* out) const {
  FitResult& r = *out;
  memset(&r, 0, sizeof(r));
  r.have = true;
  r.n = n;
  r.loo_valid = (n >= kMinLooAnchors);
  r.advisory = (n < kLooAdvisoryBelow);
  r.max_loo_id = -1;

  double sum_in = 0.0, sum_loo = 0.0;
  int loo_n = 0;

  for (int a = 0; a < n; ++a) {
    const double u = sum_x[a] * inv, v = sum_y[a] * inv;
    r.pt[a].id = dst_pts[a].id;
    r.pt[a].loo_mm = -1.0;

    double wx, wy;
    if (MapThrough(H, u, v, &wx, &wy)) {
      const double dx = wx - dst_pts[a].wx_mm, dy = wy - dst_pts[a].wy_mm;
      r.pt[a].in_mm = std::sqrt(dx * dx + dy * dy);
    } else {
      r.pt[a].in_mm = -1.0;
    }
    sum_in += (r.pt[a].in_mm > 0) ? r.pt[a].in_mm * r.pt[a].in_mm : 0.0;

    if (!r.loo_valid) continue;
    cv::Mat Hf;
    if (!(this->*subset)(ch, a, &Hf)) continue;
    if (!MapThrough(Hf, u, v, &wx, &wy)) continue;
    const double dx = wx - dst_pts[a].wx_mm, dy = wy - dst_pts[a].wy_mm;
    r.pt[a].loo_mm = std::sqrt(dx * dx + dy * dy);
    sum_loo += r.pt[a].loo_mm * r.pt[a].loo_mm;
    ++loo_n;
    if (r.pt[a].loo_mm > r.max_loo_mm) {
      r.max_loo_mm = r.pt[a].loo_mm;
      r.max_loo_id = r.pt[a].id;
    }
  }

  r.rmse_in_mm = (n > 0) ? std::sqrt(sum_in / (double)n) : 0.0;
  r.rmse_loo_mm = (loo_n > 0) ? std::sqrt(sum_loo / (double)loo_n) : -1.0;
}

bool HomographyMapper::SaveAnchors(int ch) {
  if (ch < 0 || ch >= kChannels) {
    fail_reason_ = "채널 번호가 0..3 범위를 벗어났습니다";
    return false;
  }
  if (!persist_ok_) {
    fail_reason_ = "저장 위치에 쓸 수 없습니다 (/status 의 calib.persist 확인)";
    return false;
  }
  if (!SaveAnchorsOne(ch)) {
    fail_reason_ = "저장 위치에 쓰기 실패";
    return false;
  }
  fail_reason_ = "";
  return true;
}

// ===== 주행 캘리 (오도메트리) =================================================

bool HomographyMapper::StartOdom(int ch, const char** wire_reason) {
  if (wire_reason) *wire_reason = NULL;
  if (ch < 0 || ch >= kChannels) {
    fail_reason_ = "채널 번호가 0..3 범위를 벗어났습니다";
    return false;
  }
  // 한 렌즈의 프레임을 두 수집기가 나눠 가지면 양쪽 다 조용히 굶는다.
  // StartFit() 이 K 세션을 거부하는 것과 같은 판단이다.
  if (calib_ != NULL && calib_->Collecting(ch)) {
    fail_reason_ = "이 렌즈는 K/dist 캘리브레이션 세션 중입니다";
    if (wire_reason) *wire_reason = "session_conflict";
    return false;
  }
  if (fit_[ch].active) {
    fail_reason_ = "이 렌즈는 정적 앵커 수집(CALIB_START) 중입니다";
    return false;
  }
  // 주행 캘리는 왜곡 보정된 픽셀 위에서만 성립한다. raw 픽셀은 평면과
  // 호모그래피 관계 자체가 아니다(이 렌즈는 k3 = -7.98).
  if (!coord_undistorted_[ch]) {
    fail_reason_ = "이 렌즈가 raw 좌표로 설정돼 있습니다 — HG_COORD_MODE 로 undistort 로 바꾸세요";
    return false;
  }
  if (calib_ == NULL || !calib_->Available(ch)) {
    fail_reason_ = "이 렌즈에 K/dist 가 없습니다 — 내부 파라미터를 먼저 캘리브레이션하세요";
    if (wire_reason) *wire_reason = "no_intrinsics";
    return false;
  }

  odom_n_[ch] = 0;
  odom_active_[ch] = true;
  odom_close_have_[ch] = false;
  memset(&odom_result_[ch], 0, sizeof(odom_result_[ch]));
  fail_reason_ = "";
  return true;
}

bool HomographyMapper::AddOdomPoint(int ch, int point_index,
                                    double wx_mm, double wy_mm,
                                    double u, double v, bool closing) {
  if (ch < 0 || ch >= kChannels) { fail_reason_ = "채널 범위 밖"; return false; }
  if (!odom_active_[ch]) { fail_reason_ = "열린 주행 세션이 없습니다"; return false; }

  if (closing) {
    odom_close_have_[ch] = true;
    odom_close_u_[ch] = u;   odom_close_v_[ch] = v;
    odom_close_wx_[ch] = wx_mm; odom_close_wy_[ch] = wy_mm;
    fail_reason_ = "";
    return true;
  }

  // 같은 지점이 다시 오면 덮어쓴다(서버 재시도). 순번으로 찾는 이유는 도착
  // 순서를 신뢰하지 않기 위해서다 — 재시도가 섞이면 순서가 뒤집힐 수 있다.
  int slot = -1;
  for (int i = 0; i < odom_n_[ch]; ++i)
    if (odom_[ch][i].id == point_index) { slot = i; break; }
  if (slot < 0) {
    if (odom_n_[ch] >= kMaxAnchors) {
      fail_reason_ = "지점이 너무 많습니다";
      return false;
    }
    slot = odom_n_[ch]++;
  }
  odom_[ch][slot].id = point_index;
  odom_[ch][slot].wx_mm = wx_mm;
  odom_[ch][slot].wy_mm = wy_mm;
  odom_sx_[ch][slot] = u;      // 이미 평균낸 값 (FitPoints 는 inv=1.0 로 읽는다)
  odom_sy_[ch][slot] = v;
  fail_reason_ = "";
  return true;
}

bool HomographyMapper::OdomHasLabel(int ch, double wx_mm, double wy_mm) const {
  if (ch < 0 || ch >= kChannels) return false;
  for (int i = 0; i < odom_n_[ch]; ++i) {
    if (std::fabs(odom_[ch][i].wx_mm - wx_mm) <= 1.0 &&
        std::fabs(odom_[ch][i].wy_mm - wy_mm) <= 1.0)
      return true;
  }
  return false;
}

void HomographyMapper::AbortOdom(int ch) {
  if (ch < 0 || ch >= kChannels) return;
  odom_active_[ch] = false;
  odom_n_[ch] = 0;
  odom_close_have_[ch] = false;
}

bool HomographyMapper::OdomActive(int ch) const {
  return ch >= 0 && ch < kChannels && odom_active_[ch];
}

int HomographyMapper::OdomCount(int ch) const {
  return (ch >= 0 && ch < kChannels) ? odom_n_[ch] : 0;
}

const HomographyMapper::FitResult& HomographyMapper::OdomResiduals(int ch) const {
  static const FitResult kNone = FitResult();
  if (ch < 0 || ch >= kChannels) return kNone;
  return odom_result_[ch];
}

bool HomographyMapper::FinishOdom(int ch) {
  if (ch < 0 || ch >= kChannels) { fail_reason_ = "채널 범위 밖"; return false; }
  odom_active_[ch] = false;

  const int n = odom_n_[ch];
  if (n < kMinFitAnchors) {
    static char why[96];
    snprintf(why, sizeof(why), "유효 지점이 %d개뿐입니다 — 최소 %d개 필요", n, kMinFitAnchors);
    fail_reason_ = why;
    return false;
  }

  cv::Mat H;
  if (!FitOdomSubset(ch, -1, &H)) {
    fail_reason_ = "호모그래피 계산 실패 — 지점들이 한 직선 위에 있거나 너무 몰려 있습니다";
    return false;
  }

  double h[9];
  for (int i = 0; i < 9; ++i) h[i] = H.at<double>(i / 3, i % 3);
  // Set() 의 게이트와 같은 검사. 주행 피팅도 발산할 수 있고, 그걸 여기서 잡지
  // 않으면 첫 pose 패킷에서야 드러난다.
  for (int i = 0; i < 9; ++i) {
    if (!std::isfinite(h[i])) { fail_reason_ = "행렬에 유한하지 않은 값이 있습니다"; return false; }
  }
  {
    cv::Mat m(3, 3, CV_64F, h);
    if (std::fabs(cv::determinant(m)) < 1e-12) {
      fail_reason_ = "행렬이 특이합니다 (역변환 불가)";
      return false;
    }
  }

  memcpy(hmm_[ch], h, sizeof(hmm_[ch]));
  hmm_have_[ch] = true;
  hmm_undistorted_[ch] = coord_undistorted_[ch];
  hmm_k_gen_[ch] = (calib_ != NULL) ? calib_->KGeneration(ch) : 0u;
  hmm_k_stale_[ch] = false;
  hmm_height_mm_[ch] = marker_height_mm_;
  last_h_source_[ch] = LastHSource::kOdom;

  ComputeResiduals(H, odom_sx_[ch], odom_sy_[ch], 1.0, odom_[ch], n,
                   &HomographyMapper::FitOdomSubset, ch, &odom_result_[ch]);
  fail_reason_ = "";
  return true;
}

bool HomographyMapper::OdomClosureMm(int ch, double* err_mm) const {
  if (ch < 0 || ch >= kChannels || !odom_close_have_[ch] || !hmm_have_[ch]) return false;
  cv::Mat H(3, 3, CV_64F, const_cast<double*>(hmm_[ch]));
  double wx, wy;
  if (!MapThrough(H, odom_close_u_[ch], odom_close_v_[ch], &wx, &wy)) return false;
  const double dx = wx - odom_close_wx_[ch], dy = wy - odom_close_wy_[ch];
  if (err_mm) *err_mm = std::sqrt(dx * dx + dy * dy);
  return true;
}

// ===== 측정 H_marker 슬롯 =====================================================

bool HomographyMapper::MeasuredMarkerPlaneReady(int ch) const {
  return ch >= 0 && ch < kChannels && hmm_have_[ch];
}

bool HomographyMapper::GetMeasuredMarkerPlane(int ch, double h[9]) const {
  if (!MeasuredMarkerPlaneReady(ch) || h == NULL) return false;
  memcpy(h, hmm_[ch], sizeof(hmm_[ch]));
  return true;
}

bool HomographyMapper::MeasuredKStale(int ch) const {
  return ch >= 0 && ch < kChannels && hmm_have_[ch] && hmm_k_stale_[ch];
}

bool HomographyMapper::MeasuredHeightStale(int ch) const {
  if (ch < 0 || ch >= kChannels || !hmm_have_[ch]) return false;
  // 1mm 는 측정 오차 안이라 그 정도 차이로 경고를 띄우지 않는다.
  return std::fabs(hmm_height_mm_[ch] - marker_height_mm_) > 1.0;
}

bool HomographyMapper::PreferMeasured(int ch) const {
  return ch >= 0 && ch < kChannels && hmm_preferred_[ch];
}

bool HomographyMapper::SetPreferMeasured(int ch, bool on) {
  if (ch < 0 || ch >= kChannels) { fail_reason_ = "채널 범위 밖"; return false; }
  if (on && !hmm_have_[ch]) {
    fail_reason_ = "이 렌즈에 측정된 H_marker 가 없습니다 (주행 캘리를 먼저 하세요)";
    return false;
  }
  hmm_preferred_[ch] = on;
  fail_reason_ = "";
  return true;
}

/**
 * 두 H_marker 비교. 자세한 근거는 헤더 참고.
 *
 * 닮음변환(scale·rotation·translation)만 맞춘다. 완전한 Procrustes 가 아니라
 * 2x2 최소제곱으로 회전+스케일을 함께 푸는 방식인데, 두 평면이 같은 물리
 * 평면이면 그 해가 곧 닮음변환이고, 아니라면 rmse 가 그 사실을 드러낸다.
 */
bool HomographyMapper::CompareMarkerPlanes(int ch, double* scale, double* rmse_mm,
                                           double* max_mm, const char** reason) const {
  if (ch < 0 || ch >= kChannels) { if (reason) *reason = "채널 범위 밖"; return false; }
  if (!hmm_have_[ch]) { if (reason) *reason = "측정된 H_marker 가 없습니다"; return false; }
  double hd[9];
  if (!GetMarkerPlane(ch, hd)) {
    if (reason) *reason = "파생 H_marker 가 없습니다 (체커보드 캘리와 마커 높이가 필요)";
    return false;
  }
  // 두 행렬이 다른 픽셀 공간에서 왔으면 비교 자체가 무의미하다.
  if (hmm_undistorted_[ch] != fitted_undistorted_[ch]) {
    if (reason) *reason = "두 행렬의 픽셀 공간이 다릅니다 (한쪽이 raw)";
    return false;
  }

  cv::Mat A(3, 3, CV_64F, hd);
  cv::Mat B(3, 3, CV_64F, const_cast<double*>(hmm_[ch]));

  // 화면 중앙 80% 격자. 가장자리는 왜곡 보정이 발산하는 영역이라 뺀다.
  // 해상도를 모르므로 이 렌즈가 실제로 매핑하는 픽셀 범위 대신 넉넉한
  // 고정 격자를 쓴다 — 비교는 두 매핑의 "차이"만 보므로 절대 위치는 무관하다.
  const int kGrid = 5;
  double ax[kGrid * kGrid], ay[kGrid * kGrid], bx[kGrid * kGrid], by[kGrid * kGrid];
  int m = 0;
  for (int i = 0; i < kGrid; ++i) {
    for (int j = 0; j < kGrid; ++j) {
      const double u = 259.2 + 2073.6 * i / (kGrid - 1);   // 2592 의 10%~90%
      const double v = 152.0 + 1216.0 * j / (kGrid - 1);   // 1520 의 10%~90%
      double wax, way, wbx, wby;
      if (!MapThrough(A, u, v, &wax, &way)) continue;
      if (!MapThrough(B, u, v, &wbx, &wby)) continue;
      ax[m] = wax; ay[m] = way; bx[m] = wbx; by[m] = wby; ++m;
    }
  }
  if (m < 3) { if (reason) *reason = "비교할 점이 부족합니다"; return false; }

  // 중심을 빼고 닮음변환 계수를 푼다.
  //   b = s*R*a + t  ->  중심화하면 b' = (s*cos, -s*sin; s*sin, s*cos) a'
  // 최소제곱 해가 닫힌 형태로 나온다.
  double amx = 0, amy = 0, bmx = 0, bmy = 0;
  for (int i = 0; i < m; ++i) { amx += ax[i]; amy += ay[i]; bmx += bx[i]; bmy += by[i]; }
  amx /= m; amy /= m; bmx /= m; bmy /= m;

  double sxx = 0, sxy = 0, saa = 0;
  for (int i = 0; i < m; ++i) {
    const double a1 = ax[i] - amx, a2 = ay[i] - amy;
    const double b1 = bx[i] - bmx, b2 = by[i] - bmy;
    sxx += a1 * b1 + a2 * b2;      // cos 성분
    sxy += a1 * b2 - a2 * b1;      // sin 성분
    saa += a1 * a1 + a2 * a2;
  }
  if (saa < 1e-9) { if (reason) *reason = "비교 점들이 한 곳에 몰려 있습니다"; return false; }
  const double c = sxx / saa, s = sxy / saa;      // c = s*cos, s = s*sin
  const double sc = std::sqrt(c * c + s * s);
  if (!(sc > 0.0)) { if (reason) *reason = "닮음변환 해가 없습니다"; return false; }

  double sum2 = 0.0, worst = 0.0;
  for (int i = 0; i < m; ++i) {
    const double a1 = ax[i] - amx, a2 = ay[i] - amy;
    const double px = c * a1 - s * a2 + bmx;
    const double py = s * a1 + c * a2 + bmy;
    const double dx = bx[i] - px, dy = by[i] - py;
    const double d = std::sqrt(dx * dx + dy * dy);
    sum2 += d * d;
    if (d > worst) worst = d;
  }
  if (scale)   *scale = sc;
  if (rmse_mm) *rmse_mm = std::sqrt(sum2 / (double)m);
  if (max_mm)  *max_mm = worst;
  if (reason)  *reason = "";
  return true;
}

bool HomographyMapper::Set(int ch, const double h[9], bool fitted_undistorted) {
  if (ch < 0 || ch >= kChannels) {
    fail_reason_ = "채널 번호가 0..3 범위를 벗어났습니다";
    return false;
  }
  if (h == NULL) {
    fail_reason_ = "3x3 행렬이 없습니다";
    return false;
  }
  cv::Mat m(3, 3, CV_64F);
  for (int i = 0; i < 9; ++i) {
    // Rejecting NaN/Inf here rather than at the parser: Set() is also the door
    // a computed H will come through later, and a fit that diverged produces
    // exactly these values.
    if (!std::isfinite(h[i])) {
      fail_reason_ = "행렬에 유한하지 않은 값이 있습니다";
      return false;
    }
    m.at<double>(i / 3, i % 3) = h[i];
  }
  if (std::fabs(cv::determinant(m)) < kMinDeterminant) {
    fail_reason_ = "행렬이 특이(singular)합니다 — 픽셀을 평면에 대응시킬 수 없습니다";
    return false;
  }

  H_[ch] = m;
  available_[ch] = true;
  fitted_undistorted_[ch] = fitted_undistorted;
  // The residuals described the PREVIOUS matrix. Leaving them attached to a new
  // one is the trap the whole no-persistence rule is about, only faster: an
  // injected H would arrive wearing the last fit's error figures, and a good
  // LOO number next to a matrix that was never checked is worse than none.
  // FinishFit() calls this and then refills the table, so a computed matrix
  // still gets its own.
  memset(&result_[ch], 0, sizeof(result_[ch]));
  // The matrix carries its own space, so installing one sets the mode rather
  // than being checked against it. This is the one path that may change the
  // mode while a matrix exists, and it is safe precisely because both are set
  // together — it is the SPLIT between them that SetCoordMode() refuses.
  coord_undistorted_[ch] = fitted_undistorted;

  // This matrix belongs to the K that is current at this instant, whether it
  // arrived from a fit, an injection or the disk. Re-baselining here is what
  // makes CalibStale() mean "changed SINCE the fit" rather than "changed at
  // some point".
  k_gen_[ch] = (calib_ != NULL) ? calib_->KGeneration(ch) : 0u;
  k_stale_[ch] = false;

  // Every derived value was derived from the matrix that just got replaced.
  //
  // This was missing, and it is the same failure as the K one a level down:
  // Set() is the ONE door a new matrix comes through — FinishFit(), HG_SET and
  // LoadOne() all end here — so leaving the rebuild to the callers meant
  // leaving it to three of them. Two got it by accident (LoadOne() calls
  // SetCameraHeightMm() afterwards, which refreshes) and the one that mattered
  // did not: after a successful CALIB_START the pose path went on consulting
  // an H_marker built from the PREVIOUS matrix, or, on a lens fitting for the
  // first time, kept refusing to map at all because the cache still said "이
  // 렌즈에 H가 없습니다" — an H that is present and mappable, producing no
  // world coordinates, with nothing anywhere to say why.
  RefreshMarkerPlane(ch);
  fail_reason_ = "";
  return true;
}

void HomographyMapper::SyncCalib(int ch) const {
  if (ch < 0 || ch >= kChannels || calib_ == NULL) return;
  const unsigned gen = calib_->KGeneration(ch);
  if (gen == k_gen_[ch]) return;  // the settled case, and the only cost of this
  k_gen_[ch] = gen;

  // A matrix fitted in RAW pixels never saw K, so a new K leaves it exactly as
  // valid as it was — only its marker plane, which decomposes through K, has
  // to be rebuilt (and that path refuses a raw-fitted H anyway). It is the
  // undistorted-space fit that is orphaned, because the pixels it was fitted
  // from were produced by the K that just went away.
  if (available_[ch] && fitted_undistorted_[ch]) k_stale_[ch] = true;
  // 측정 H_marker 도 같은 이유로 상한다 — 잴 당시의 K 로 보정한 픽셀 위에서
  // 피팅됐으므로, K 가 바뀌면 더 이상 유효하지 않은 보정 결과 위에 얹힌 행렬이
  // 된다. 파생 Hm_ 와 달리 자동으로 따라오지 않는다(서버팀 회신 §3-3).
  if (hmm_have_[ch] && hmm_undistorted_[ch] && hmm_k_gen_[ch] != gen)
    hmm_k_stale_[ch] = true;

  // Both cached derivations — H_marker and the camera pose — read K.
  RefreshMarkerPlane(ch);
}

bool HomographyMapper::CalibStale(int ch) const {
  if (ch < 0 || ch >= kChannels) return false;
  SyncCalib(ch);
  return k_stale_[ch];
}

bool HomographyMapper::Mappable(int ch) const {
  if (!Available(ch)) return false;
  if (!coord_undistorted_[ch]) return true;
  return calib_ != NULL && calib_->Available(ch);
}

bool HomographyMapper::Save(int ch) {
  if (!Available(ch)) {
    fail_reason_ = "저장할 호모그래피가 없습니다 — 먼저 계산하거나 주입하세요";
    return false;
  }
  if (!persist_ok_) {
    fail_reason_ = "저장 위치에 쓸 수 없습니다 (/status 의 calib.persist 확인)";
    return false;
  }
  if (!SaveOne(ch)) {
    fail_reason_ = "저장 위치에 쓰기 실패";
    return false;
  }
  fail_reason_ = "";
  return true;
}

bool HomographyMapper::Clear(int ch) {
  if (ch < 0 || ch >= kChannels) {
    fail_reason_ = "채널 번호가 0..3 범위를 벗어났습니다";
    return false;
  }
  available_[ch] = false;
  fitted_undistorted_[ch] = false;
  camera_z_mm_[ch] = 0.0;
  // Nothing left to be stale. The flag describes a matrix, and there is no
  // longer one — leaving it set would put a warning on a lens whose next fit
  // has not happened yet.
  k_stale_[ch] = false;
  k_gen_[ch] = (calib_ != NULL) ? calib_->KGeneration(ch) : 0u;
  H_[ch].release();
  memset(&result_[ch], 0, sizeof(result_[ch]));  // measured the matrix just dropped
  RefreshMarkerPlane(ch);                        // derived FROM the matrix just dropped
  // The marker list deliberately survives: it is the tape-measured input, not
  // a product of the matrix, and re-registering 24 points because a bad
  // injection had to be undone would be an unreasonable thing to ask.

  char leaf[64], path[512];
  snprintf(leaf, sizeof(leaf), "homography_ch%d.txt", ch);
  PathFor(leaf, path, sizeof(path));
  remove(path);  // already absent is success as far as the caller cares
  fail_reason_ = "";
  return true;
}

void HomographyMapper::PathFor(const char* leaf, char* out, size_t out_size) const {
  snprintf(out, out_size, "%s/%s", persist_dir_, leaf);
}

/**
 * File layout (text, one lens per file, overwritten on every save):
 *
 *   <version> <channel>
 *   h00 h01 h02
 *   h10 h11 h12
 *   h20 h21 h22
 *   <fitted_undistorted> <camera_z_mm>
 *
 * Text and not binary for the same reason K/dist is text: some day the
 * question will be "why is this one lens wrong", and with no shell on the
 * camera the answer starts by looking at the file.
 *
 * %.17g rather than K/dist's %.10e: H is only defined up to scale, so its
 * entries routinely span many orders of magnitude within one matrix, and a
 * round-trip that loses low bits of h20/h21 moves the far end of the frame.
 */
bool HomographyMapper::SaveOne(int ch) {
  char leaf[64], path[512];
  snprintf(leaf, sizeof(leaf), "homography_ch%d.txt", ch);
  PathFor(leaf, path, sizeof(path));
  FILE* f = fopen(path, "w");
  if (!f) return false;
  fprintf(f, "%d %d\n", kFileVersion, ch);
  for (int r = 0; r < 3; ++r)
    fprintf(f, "%.17g %.17g %.17g\n", H_[ch].at<double>(r, 0), H_[ch].at<double>(r, 1),
            H_[ch].at<double>(r, 2));
  fprintf(f, "%d %.17g\n", fitted_undistorted_[ch] ? 1 : 0, camera_z_mm_[ch]);
  // Which calibration produced this H, so IVA_SYNC still knows after a
  // restart. Appended rather than folded into a version bump: LoadOne() reads
  // it with a separate fscanf that is allowed to come up empty, so a file
  // written before this line still loads (as kNone, which is what it was).
  fprintf(f, "%d\n", (int)last_h_source_[ch]);
  const bool ok = (fflush(f) == 0);
  fclose(f);
  return ok;
}

/**
 * Marker list file (text, one lens per file, overwritten on every save):
 *
 *   <version> <channel> <count>
 *   <id> <wx_mm> <wy_mm>
 *   ... <count> lines ...
 *
 * A count of 0 is written rather than the file being deleted: "this lens was
 * deliberately left with no markers" and "nobody has been here yet" are
 * different states, and only the first one has a file.
 */
bool HomographyMapper::SaveAnchorsOne(int ch) {
  char leaf[64], path[512];
  snprintf(leaf, sizeof(leaf), "anchors_ch%d.txt", ch);
  PathFor(leaf, path, sizeof(path));
  FILE* f = fopen(path, "w");
  if (!f) return false;
  fprintf(f, "%d %d %d\n", kFileVersion, ch, anchor_n_[ch]);
  for (int i = 0; i < anchor_n_[ch]; ++i)
    fprintf(f, "%d %.4f %.4f\n", anchors_[ch][i].id, anchors_[ch][i].wx_mm,
            anchors_[ch][i].wy_mm);
  const bool ok = (fflush(f) == 0);
  fclose(f);
  return ok;
}

bool HomographyMapper::LoadAnchorsOne(int ch) {
  char leaf[64], path[512];
  snprintf(leaf, sizeof(leaf), "anchors_ch%d.txt", ch);
  PathFor(leaf, path, sizeof(path));
  FILE* f = fopen(path, "r");
  if (!f) return false;

  int version = 0, file_ch = -1, count = -1;
  if (fscanf(f, "%d %d %d", &version, &file_ch, &count) != 3 || version != kFileVersion) {
    fclose(f);
    return false;
  }
  if (file_ch != ch) {
    fclose(f);
    printf("[ArucoPosePNM] anchors: %s says ch%d — ignored\n", path, file_ch);
    fflush(stdout);
    return false;
  }
  if (count < 0 || count > kMaxAnchors) {
    fclose(f);
    printf("[ArucoPosePNM] anchors: %s claims %d markers — ignored\n", path, count);
    fflush(stdout);
    return false;
  }

  Anchor tmp[kMaxAnchors];
  int got = 0;
  for (int i = 0; i < count; ++i) {
    if (fscanf(f, "%d %lf %lf", &tmp[i].id, &tmp[i].wx_mm, &tmp[i].wy_mm) != 3) break;
    ++got;
  }
  fclose(f);

  // A file that promises N markers and delivers fewer is a truncated write, not
  // a shorter list. Loading what did arrive would put a lens into calibration
  // with a marker missing and nothing on screen to say so.
  if (got != count) {
    printf("[ArucoPosePNM] anchors: %s ended after %d of %d markers — ignored\n",
           path, got, count);
    fflush(stdout);
    return false;
  }
  // Same gates as the command path: a hand-edited file can hold a duplicate id
  // just as easily as a typed command can.
  if (!SetAnchors(ch, tmp, count)) {
    printf("[ArucoPosePNM] anchors: %s rejected (%s)\n", path, fail_reason_);
    fflush(stdout);
    return false;
  }
  return true;
}

/**
 * Shared marker height, one file for all four lenses.
 *
 *   <version> <marker_height_mm>
 *
 * The camera height is NOT here — it is per lens and lives in that lens's
 * homography file, next to the matrix it rescales.
 */
bool HomographyMapper::SaveMarkerHeightFile() {
  char path[512];
  PathFor("marker_plane.txt", path, sizeof(path));
  FILE* f = fopen(path, "w");
  if (!f) return false;
  fprintf(f, "%d %.4f\n", kFileVersion, marker_height_mm_);
  const bool ok = (fflush(f) == 0);
  fclose(f);
  return ok;
}

bool HomographyMapper::LoadMarkerHeightFile() {
  char path[512];
  PathFor("marker_plane.txt", path, sizeof(path));
  FILE* f = fopen(path, "r");
  if (!f) return false;
  int version = 0;
  double mm = 0.0;
  const int got = fscanf(f, "%d %lf", &version, &mm);
  fclose(f);
  if (got != 2 || version != kFileVersion) return false;
  if (!ValidHeightMm(mm)) {
    printf("[ArucoPosePNM] marker_plane.txt: 높이 %.1f mm — 무시\n", mm);
    fflush(stdout);
    return false;
  }
  marker_height_mm_ = mm;
  return true;
}

bool HomographyMapper::LoadOne(int ch) {
  char leaf[64], path[512];
  snprintf(leaf, sizeof(leaf), "homography_ch%d.txt", ch);
  PathFor(leaf, path, sizeof(path));
  FILE* f = fopen(path, "r");
  if (!f) return false;

  int version = 0, file_ch = -1;
  if (fscanf(f, "%d %d", &version, &file_ch) != 2 || version != kFileVersion) {
    fclose(f);
    return false;  // unknown/old format — treat as "not available"
  }
  double h[9];
  int n = 0;
  for (int i = 0; i < 9; ++i) n += fscanf(f, "%lf", &h[i]);
  int undist = 0;
  double cz = 0.0;
  const int tail = fscanf(f, "%d %lf", &undist, &cz);
  // Optional trailing field (see SaveOne): which calibration produced this H.
  // Absent in files written before it existed, which read as kNone -- the
  // same answer those files gave before, not a regression.
  int src = (int)LastHSource::kNone;
  const bool have_src = (fscanf(f, "%d", &src) == 1);
  fclose(f);
  if (n != 9 || tail != 2) return false;

  // The channel is in the file as well as in its name, and a mismatch is
  // refused. Four near-identical files and one SSH session is all it takes to
  // copy the wrong one, and a lens running another lens's H reports
  // coordinates that are wrong by metres while looking entirely well-formed.
  if (file_ch != ch) {
    printf("[ArucoPosePNM] homography: %s says ch%d — ignored\n", path, file_ch);
    fflush(stdout);
    return false;
  }
  // Same gates as Set(): a file edited by hand is exactly as able to contain
  // a singular matrix as a command line is.
  if (!Set(ch, h, undist != 0)) {
    printf("[ArucoPosePNM] homography: %s rejected (%s)\n", path, fail_reason_);
    fflush(stdout);
    return false;
  }
  // Through the setter, exactly as the matrix went through Set() and the marker
  // list goes through SetAnchors(). Cz was the one input that reached a member
  // by direct assignment, which meant the range check lived only on the command
  // path — a hand-edited file could install a negative or NaN height that
  // CAMERA_HEIGHT would have refused — and the cache refresh was a separate
  // line someone had to remember. Routing it here leaves one mutation site.
  if (!SetCameraHeightMm(ch, cz)) {
    printf("[ArucoPosePNM] %s: 카메라 높이 %.1f 거부 (%s) — 0으로 둡니다\n", path, cz,
           fail_reason_);
    fflush(stdout);
  }
  // Down here, past the gates, so a file we ended up refusing cannot leave a
  // source behind claiming its matrix was installed. Only the two real values
  // are honoured -- anything else in that slot is a corrupt or hand-edited
  // file, and kNone (IVA_SYNC refuses, and says why) is the safe reading.
  if (have_src && (src == (int)LastHSource::kFloor || src == (int)LastHSource::kOdom)) {
    last_h_source_[ch] = (LastHSource)src;
  }
  return true;
}
