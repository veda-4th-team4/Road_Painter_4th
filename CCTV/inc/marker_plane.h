#ifndef MARKER_PLANE_H
#define MARKER_PLANE_H

/**
 * H_marker: the homography for the plane the robot's marker actually sits on.
 *
 * Why this exists
 * ---------------
 * A homography maps ONE physical plane to the image. The floor H is fitted
 * from anchors lying on the floor, so projecting a marker that is mounted
 * 250 mm up through it puts the robot somewhere it is not -- displaced away
 * from the camera's nadir, by an amount that grows with distance from it.
 *
 *   ground = F - (h / Cz) * (F - nadir)
 *
 * ...where F is the floor-projected point, Cz the camera height and h the
 * marker height. With a camera ~1.5 m up and a marker at 250 mm, that is a
 * sixth of the distance to the nadir -- metres of error at the edge of a large
 * work area, and it is SYSTEMATIC, so no amount of averaging removes it.
 *
 * The server protocol therefore asks for two matrices: H_floor for drawing and
 * H_marker for locating the robot, with the parallax absorbed at calibration
 * time rather than corrected per-frame.
 *
 * How it is derived
 * -----------------
 * From the floor homography alone, by decomposition:
 *
 *   H_w2p = K [r1 r2 t]        (inverse of the stored pixel->world H)
 *   B     = K^-1 H_w2p         columns b1 b2 b3
 *   lambda= 1 / |b1|           (scale lost to homogeneity)
 *   r3    = r1 x r2            the plane normal, which the floor H cannot see
 *   H_marker_w2p = K [r1 r2 (h*r3 + t)]
 *
 * The floor H has no information about the third dimension; r3 recovers it
 * from the orthonormality of a rotation matrix. That is the whole trick.
 *
 * IMPORTANT LIMITATION
 * --------------------
 * K^-1 H assumes a pinhole camera. If H was fitted in RAW pixels, lens
 * distortion is absorbed into R and t, and the recovered r3 -- hence the
 * entire correction -- is biased. This refuses to run unless the homography
 * is in undistorted space. It is not a style preference; the output is wrong
 * otherwise, and wrong in a way that looks plausible.
 */

// Derive H_marker (pixel -> world mm, same direction and pixel space as the
// input) for a marker plane `height_mm` above the floor.
//
// Returns false and sets *reason on: singular H, a K that is not available,
// or a degenerate decomposition. height_mm == 0 yields a copy of the input,
// which is correct only if the marker really is on the floor.
bool marker_plane_derive(const double H_floor_p2w[9], double height_mm,
                         double H_marker_p2w[9], const char** reason);

// Recover the camera pose implied by the floor homography: height above the
// floor plane and the nadir (the floor point directly beneath the camera), in
// world mm. Useful as a sanity check -- an installer who knows the camera is
// 1.5 m up can immediately see whether the decomposition is sane, and a wrong
// nadir is the tell-tale of a bad K.
bool marker_plane_camera_pose(const double H_floor_p2w[9],
                              double* height_mm, double* nadir_x_mm,
                              double* nadir_y_mm, const char** reason);

// --- Runtime marker height -------------------------------------------------
// Field data, not a build constant: it depends on how THIS robot was built and
// is not known until the marker is physically mounted. Persisted so it
// survives a reboot, like K and H.

void   marker_plane_init(void);              // load persisted height + Cz
double marker_plane_height_mm(void);
bool   marker_plane_set_height_mm(double mm); // rejects negative / non-finite
bool   marker_plane_save_height(void);        // explicit, like HG_SAVE

// --- Measured camera height (optional override) -----------------------------
//
// The correction magnitude is the ratio h / Cz. h is measured, but Cz comes
// out of the H decomposition -- and that decomposition assumes a pinhole
// camera, so any residual distortion biases the scale of t and therefore Cz.
// A wrong Cz scales the ENTIRE parallax correction by the same factor.
//
// Giving a tape-measured camera height removes that error term. 0 means "no
// measurement, use the derived value" and is the default, so behaviour is
// unchanged until an installer supplies a number.
//
// Note what this does NOT do: it never alters H_floor. That matrix is the
// fitted ground truth for the floor plane and rescaling it would corrupt every
// coordinate the robot receives. Only the derived marker plane moves. See the
// implementation for how the two are kept independent.
double marker_plane_camera_height_mm(void);      // 0 = use derived
bool   marker_plane_set_camera_height_mm(double mm);

#endif // MARKER_PLANE_H
