#include "marker_plane.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include <opencv2/core.hpp>

#include "app_config.h"
#include "homography_mapper.h"
#include "intrinsics_calibrator.h"

namespace {

double g_height_mm = MARKER_HEIGHT_M * 1000.0;

// Tape-measured camera height. 0 = not measured; use whatever the H
// decomposition implies. See marker_plane.h for why this exists.
double g_camera_height_mm = 0.0;

bool build_K(cv::Matx33d* K, const char** reason)
{
    double fx, fy, cx, cy, d[5];
    if (!intrinsics_get(&fx, &fy, &cx, &cy, d)) {
        if (reason) *reason = "no camera intrinsics (K) loaded";
        return false;
    }
    *K = cv::Matx33d(fx, 0.0, cx,
                     0.0, fy, cy,
                     0.0, 0.0, 1.0);
    return true;
}

// Shared front half of both public functions: decompose the floor homography
// into the camera's rotation columns and translation, in world mm.
bool decompose(const double H_p2w[9], cv::Matx33d* K,
               cv::Vec3d* r1, cv::Vec3d* r2, cv::Vec3d* r3, cv::Vec3d* t,
               const char** reason)
{
    if (H_p2w == NULL) {
        if (reason) *reason = "no homography";
        return false;
    }
    // Distortion would be absorbed into R and t, biasing r3 -- and r3 IS the
    // correction. Refuse rather than emit a plausible-looking wrong answer.
    if (!homography_undistort_enabled()) {
        if (reason) *reason = "H must be fitted in undistorted pixels "
                              "(set HG_COORD_MODE undistort and re-run CALIB_START)";
        return false;
    }
    if (!build_K(K, reason))
        return false;

    const cv::Matx33d Hp2w(H_p2w[0], H_p2w[1], H_p2w[2],
                           H_p2w[3], H_p2w[4], H_p2w[5],
                           H_p2w[6], H_p2w[7], H_p2w[8]);
    if (fabs(cv::determinant(Hp2w)) < 1e-18) {
        if (reason) *reason = "homography is singular";
        return false;
    }
    const cv::Matx33d Hw2p = Hp2w.inv();          // world mm -> pixel
    const cv::Matx33d B    = K->inv() * Hw2p;     // == [r1 r2 t] up to scale

    const cv::Vec3d b1(B(0, 0), B(1, 0), B(2, 0));
    const cv::Vec3d b2(B(0, 1), B(1, 1), B(2, 1));
    const cv::Vec3d b3(B(0, 2), B(1, 2), B(2, 2));

    const double n1 = cv::norm(b1), n2 = cv::norm(b2);
    if (n1 < 1e-12 || n2 < 1e-12) {
        if (reason) *reason = "degenerate decomposition";
        return false;
    }
    // Homogeneity loses one scale factor. Averaging the two column norms is
    // steadier than trusting either alone, since neither is exactly unit once
    // fitting noise is in the matrix.
    double lambda = 2.0 / (n1 + n2);

    // Sign is also lost. The camera must be on the +Z side of the floor, so
    // t_z > 0; otherwise the whole solution is mirrored and the correction
    // would push the marker the wrong way.
    if (b3[2] < 0.0)
        lambda = -lambda;

    *r1 = b1 * lambda;
    *r2 = b2 * lambda;
    *t  = b3 * lambda;
    *r3 = (*r1).cross(*r2);

    const double n3 = cv::norm(*r3);
    if (n3 < 1e-9) {
        if (reason) *reason = "rotation columns are parallel";
        return false;
    }
    *r3 /= n3;                                    // re-normalise
    return true;
}

} // namespace

bool marker_plane_derive(const double H_floor_p2w[9], double height_mm,
                         double H_marker_p2w[9], const char** reason)
{
    if (H_marker_p2w == NULL)
        return false;

    // A marker genuinely on the floor needs no correction, and asking for one
    // would only inject decomposition noise.
    if (height_mm == 0.0) {
        memcpy(H_marker_p2w, H_floor_p2w, 9 * sizeof(double));
        return true;
    }

    cv::Matx33d K;
    cv::Vec3d r1, r2, r3, t;
    if (!decompose(H_floor_p2w, &K, &r1, &r2, &r3, &t, reason))
        return false;

    // Apply a measured camera height, if the installer supplied one.
    //
    // The parallax is governed by the RATIO h / Cz. Cz here is whatever the
    // decomposition produced, and its scale is the part distortion corrupts.
    // The obvious fix -- rescaling t so the camera lands at the measured
    // height -- is wrong: t is what reproduces the fitted floor H, and moving
    // it would make the derived matrices disagree with H_floor itself.
    //
    // Scaling the marker height instead leaves t (and therefore H_floor)
    // untouched while producing exactly the intended ratio:
    //
    //     h_eff / Cz_derived  ==  h_measured / Cz_measured
    //
    // so h_eff = h * (Cz_derived / Cz_measured).
    double effective_h = height_mm;
    const double cz_measured = g_camera_height_mm;
    if (cz_measured > 0.0) {
        const cv::Matx33d R(r1[0], r2[0], r3[0],
                            r1[1], r2[1], r3[1],
                            r1[2], r2[2], r3[2]);
        const double cz_derived = (-(R.t() * t))[2];
        if (fabs(cz_derived) < 1e-6) {
            if (reason) *reason = "derived camera height is ~0; cannot rescale";
            return false;
        }
        effective_h = height_mm * (cz_derived / cz_measured);
    }

    // Same camera, plane shifted along its own normal by the marker height.
    const cv::Vec3d tm = t + r3 * effective_h;
    const cv::Matx33d Hm_w2p = K * cv::Matx33d(r1[0], r2[0], tm[0],
                                               r1[1], r2[1], tm[1],
                                               r1[2], r2[2], tm[2]);
    if (fabs(cv::determinant(Hm_w2p)) < 1e-18) {
        if (reason) *reason = "derived marker homography is singular";
        return false;
    }
    const cv::Matx33d Hm = Hm_w2p.inv();          // back to pixel -> world mm

    // Normalise so h22 == 1: the server compares and stores these, and an
    // arbitrary scale makes two mathematically identical matrices look
    // different in a log.
    const double s = (fabs(Hm(2, 2)) > 1e-18) ? 1.0 / Hm(2, 2) : 1.0;
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            H_marker_p2w[r * 3 + c] = Hm(r, c) * s;
    return true;
}

bool marker_plane_camera_pose(const double H_floor_p2w[9],
                              double* height_mm, double* nadir_x_mm,
                              double* nadir_y_mm, const char** reason)
{
    cv::Matx33d K;
    cv::Vec3d r1, r2, r3, t;
    if (!decompose(H_floor_p2w, &K, &r1, &r2, &r3, &t, reason))
        return false;

    // Camera centre in world coordinates: C = -R^T t, with R = [r1 r2 r3].
    const cv::Matx33d R(r1[0], r2[0], r3[0],
                        r1[1], r2[1], r3[1],
                        r1[2], r2[2], r3[2]);
    const cv::Vec3d C = -(R.t() * t);

    if (height_mm)  *height_mm  = C[2];
    if (nadir_x_mm) *nadir_x_mm = C[0];
    if (nadir_y_mm) *nadir_y_mm = C[1];
    return true;
}

// --- Runtime marker height -------------------------------------------------

void marker_plane_init(void)
{
#if PERSIST_TO_MNT
    FILE* f = fopen(MARKER_HEIGHT_FILE, "r");
    if (f != NULL) {
        // "<marker_mm> [camera_mm]". The second field was added later, so a
        // file written by an older build still loads -- fscanf simply returns
        // 1 and the camera height keeps its "not measured" default.
        double mm = 0.0, cz = 0.0;
        const int got = fscanf(f, "%lf %lf", &mm, &cz);
        if (got >= 1 && mm >= 0.0 && mm < 100000.0)
            g_height_mm = mm;
        if (got >= 2 && cz >= 0.0 && cz < 100000.0)
            g_camera_height_mm = cz;
        fclose(f);
    }
#endif
}

double marker_plane_camera_height_mm(void) { return g_camera_height_mm; }

bool marker_plane_set_camera_height_mm(double mm)
{
    // 0 is meaningful here: it clears the measurement and returns to the
    // derived value, which is how an installer undoes a bad entry.
    if (!(mm >= 0.0) || mm >= 100000.0)   // also rejects NaN
        return false;
    g_camera_height_mm = mm;
    return true;
}

double marker_plane_height_mm(void) { return g_height_mm; }

bool marker_plane_set_height_mm(double mm)
{
    // Not persisted here: applying a value and committing it are separate
    // operator actions, matching HG_SAVE / CALIB_K_SAVE. A mistyped height is
    // then undone by a restart.
    if (!(mm >= 0.0) || mm >= 100000.0)   // also rejects NaN
        return false;
    g_height_mm = mm;
    return true;
}

bool marker_plane_save_height(void)
{
#if PERSIST_TO_MNT
    FILE* f = fopen(MARKER_HEIGHT_FILE, "w");
    if (f == NULL)
        return false;
    fprintf(f, "%.3f %.3f\n", g_height_mm, g_camera_height_mm);
    fclose(f);
    return true;
#else
    return false;
#endif
}
