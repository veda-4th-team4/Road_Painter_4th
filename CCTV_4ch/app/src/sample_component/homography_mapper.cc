#include "homography_mapper.h"

#include <stdio.h>
#include <string.h>

#include <cmath>

#include <opencv2/calib3d.hpp>

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

}  // namespace

HomographyMapper::HomographyMapper()
    : calib_(NULL), persist_ok_(false), fail_reason_("") {
  persist_dir_[0] = '\0';
  for (int c = 0; c < kChannels; ++c) {
    available_[c] = false;
    fitted_undistorted_[c] = false;
    // Undistorted by default: it is the space every later step needs, and a
    // lens that has never been configured should not start out in the one
    // mode that makes the marker-plane maths refuse to run.
    coord_undistorted_[c] = true;
    camera_z_mm_[c] = 0.0;
    anchor_n_[c] = 0;
    memset(&fit_[c], 0, sizeof(fit_[c]));
    memset(&result_[c], 0, sizeof(result_[c]));
  }
}

void HomographyMapper::Init(const IntrinsicsCalib& calib) {
  calib_ = &calib;
  snprintf(persist_dir_, sizeof(persist_dir_), "%s", calib.PersistDir());
  persist_ok_ = calib.Persistable();

  for (int c = 0; c < kChannels; ++c) {
    available_[c] = LoadOne(c);
    LoadAnchorsOne(c);  // independent of H: markers may outlive a cleared matrix
  }

  printf("[ArucoPosePNM] homography: persist=%s dir=%s  loaded H:",
         persist_ok_ ? "ok" : "READ-ONLY", persist_dir_);
  for (int c = 0; c < kChannels; ++c) printf(" ch%d=%d", c, (int)available_[c]);
  printf("  anchors:");
  for (int c = 0; c < kChannels; ++c) printf(" ch%d=%d", c, anchor_n_[c]);
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
  std::vector<cv::Point2f> in(1, cv::Point2f(px, py)), norm;
  cv::undistortPoints(in, norm, K, D, cv::noArray(), cv::noArray(),
                      cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS,
                                       20, 1e-8));
  if (norm.empty()) {
    if (reason) *reason = "undistortPoints 실패";
    return false;
  }

  static const cv::Mat kZero3 = cv::Mat::zeros(3, 1, CV_64F);
  std::vector<cv::Point3f> obj(1, cv::Point3f(norm[0].x, norm[0].y, 1.0f));
  std::vector<cv::Point2f> back;
  cv::projectPoints(obj, kZero3, kZero3, K, D, back);
  if (back.empty()) {
    if (reason) *reason = "projectPoints 실패";
    return false;
  }
  const double dx = back[0].x - px, dy = back[0].y - py;
  if (dx * dx + dy * dy > kMaxRoundTripPx * kMaxRoundTripPx) {
    if (reason)
      *reason = "왜곡보정이 수렴하지 않는 영역입니다 (화면 좌우 가장자리) — "
                "이 점은 쓸 수 없습니다";
    return false;
  }

  out->x = (float)(K.at<double>(0, 0) * norm[0].x + K.at<double>(0, 2));
  out->y = (float)(K.at<double>(1, 1) * norm[0].y + K.at<double>(1, 2));
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

  const cv::Mat& H = H_[ch];
  const double u = p.x, v = p.y;
  const double w = H.at<double>(2, 0) * u + H.at<double>(2, 1) * v + H.at<double>(2, 2);
  if (std::fabs(w) < kMinHomogeneousW) return false;
  *wx = (H.at<double>(0, 0) * u + H.at<double>(0, 1) * v + H.at<double>(0, 2)) / w;
  *wy = (H.at<double>(1, 0) * u + H.at<double>(1, 1) * v + H.at<double>(1, 2)) / w;
  return true;
}

double HomographyMapper::CameraHeightMm(int ch) const {
  return (ch >= 0 && ch < kChannels) ? camera_z_mm_[ch] : 0.0;
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
    snprintf(fit_[ch].last_result, sizeof(fit_[ch].last_result),
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
    snprintf(fit_[ch].last_result, sizeof(fit_[ch].last_result), "%s", why);
    return false;
  }
  // Checked now, not at the fit. Undistortion failing after 200 frames of
  // collection tells the operator the same thing five minutes later.
  if (coord_undistorted_[ch] && (calib_ == NULL || !calib_->Available(ch))) {
    fail_reason_ = "이 렌즈는 undistort 좌표로 피팅하도록 설정돼 있는데 K/dist가 없습니다 "
                   "— 내부 파라미터를 먼저 캘리브레이션하거나 HG_COORD_MODE 로 raw 로 바꾸세요";
    snprintf(fit_[ch].last_result, sizeof(fit_[ch].last_result), "%s", fail_reason_);
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
  bool   have[kMaxAnchors];
  for (int a = 0; a < anchor_n_[ch]; ++a) {
    have[a] = false;
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
    have[a] = true;
  }

  // All or nothing. A frame contributing only the markers it happened to see
  // would weight each point by how often it was visible, which quietly favours
  // the middle of the frame — the opposite of what the placement advice is
  // trying to achieve.
  if (f.seen_n == anchor_n_[ch]) {
    for (int a = 0; a < anchor_n_[ch]; ++a) {
      if (!have[a]) continue;  // unreachable given the test above; cheap to keep
      sum_x_[ch][a] += px[a];
      sum_y_[ch][a] += py[a];
    }
    ++f.good;
  }

  if (f.good >= kFitTargetFrames) {
    const bool ok = FinishFit(ch);
    f.active = false;
    snprintf(f.last_result, sizeof(f.last_result), "%s", ok ? "완료" : fail_reason_);
    return;
  }
  if (f.total >= kFitMaxFrames) {
    f.active = false;
    snprintf(f.last_result, sizeof(f.last_result),
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
bool HomographyMapper::FitSubset(int ch, int skip, cv::Mat* out) const {
  std::vector<cv::Point2f> src;
  std::vector<cv::Point2f> dst;
  const int n = anchor_n_[ch];
  const double inv = (fit_[ch].good > 0) ? 1.0 / (double)fit_[ch].good : 0.0;
  for (int a = 0; a < n; ++a) {
    if (a == skip) continue;
    src.push_back(cv::Point2f((float)(sum_x_[ch][a] * inv), (float)(sum_y_[ch][a] * inv)));
    dst.push_back(cv::Point2f((float)anchors_[ch][a].wx_mm, (float)anchors_[ch][a].wy_mm));
  }
  if ((int)src.size() < kMinFitAnchors) return false;
  cv::Mat h = cv::findHomography(src, dst, 0);
  if (h.empty() || h.rows != 3 || h.cols != 3) return false;
  h.convertTo(*out, CV_64F);
  return true;
}

namespace {
// Map one point through a 3x3, in world millimetres. Returns false on the
// vanishing line, where there is no finite answer.
bool MapThrough(const cv::Mat& H, double u, double v, double* wx, double* wy) {
  const double w = H.at<double>(2, 0) * u + H.at<double>(2, 1) * v + H.at<double>(2, 2);
  if (std::fabs(w) < 1e-9) return false;
  *wx = (H.at<double>(0, 0) * u + H.at<double>(0, 1) * v + H.at<double>(0, 2)) / w;
  *wy = (H.at<double>(1, 0) * u + H.at<double>(1, 1) * v + H.at<double>(1, 2)) / w;
  return true;
}
}  // namespace

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

  FitResult& r = result_[ch];
  memset(&r, 0, sizeof(r));
  r.have = true;
  r.n = n;
  r.loo_valid = (n >= kMinLooAnchors);
  r.advisory = (n < kLooAdvisoryBelow);
  r.max_loo_id = -1;

  const double inv = (fit_[ch].good > 0) ? 1.0 / (double)fit_[ch].good : 0.0;
  double sum_in = 0.0, sum_loo = 0.0;
  int loo_n = 0;

  for (int a = 0; a < n; ++a) {
    const double u = sum_x_[ch][a] * inv, v = sum_y_[ch][a] * inv;
    r.pt[a].id = anchors_[ch][a].id;
    r.pt[a].loo_mm = -1.0;

    double wx, wy;
    if (MapThrough(H, u, v, &wx, &wy)) {
      const double dx = wx - anchors_[ch][a].wx_mm, dy = wy - anchors_[ch][a].wy_mm;
      r.pt[a].in_mm = std::sqrt(dx * dx + dy * dy);
    } else {
      r.pt[a].in_mm = -1.0;
    }
    sum_in += (r.pt[a].in_mm > 0) ? r.pt[a].in_mm * r.pt[a].in_mm : 0.0;

    if (!r.loo_valid) continue;
    cv::Mat Hf;
    if (!FitSubset(ch, a, &Hf)) continue;
    if (!MapThrough(Hf, u, v, &wx, &wy)) continue;
    const double dx = wx - anchors_[ch][a].wx_mm, dy = wy - anchors_[ch][a].wy_mm;
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
  fail_reason_ = "";
  return true;
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
  fail_reason_ = "";
  return true;
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
  H_[ch].release();
  memset(&result_[ch], 0, sizeof(result_[ch]));  // measured the matrix just dropped
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
  camera_z_mm_[ch] = cz;
  return true;
}
