#ifndef DYN_ROI_H
#define DYN_ROI_H

/**
 * Adaptive ROI that follows the detected markers.
 *
 * detectMarkers() costs time proportional to PIXELS, so once a marker is found
 * there is no reason to keep scanning the whole frame: search a small box
 * around where it just was. SEARCH scans the manual ROI (empty = full frame);
 * TRACK scans the last detection's bounding box plus an adaptive margin.
 *
 * On a miss the box widens for a few frames before giving up and returning to
 * SEARCH -- a brief occlusion or a fast move is far cheaper to recover that
 * way than by paying for a full-frame scan every time.
 *
 * Corner coordinates arrive full-frame already (ArucoProcessor::detect adds
 * the ROI origin back), so everything here is in full-frame pixels.
 *
 * Extracted from aruco_detector_cv.cpp. The tracker used to reach directly
 * into two file-scope globals of that translation unit -- the operator's ROI
 * and homography_collecting() -- which is exactly what made it unmovable.
 * Both are now arguments of roiForFrame(), so this class owns nothing but its
 * own state and can be reasoned about (and tested) on its own.
 */

#include <vector>

#include <opencv2/core.hpp>

#include "aruco_processor.h"

class DynRoiTracker {
public:
    DynRoiTracker();

    // Apply an operator command. Always restarts from a full search: the new
    // margin describes a box that the current one was not tracking with.
    // Returns false when the tracker is now off, so the caller can restore the
    // manual ROI in the detector.
    bool configure(bool on, int maxMargin, int maxMiss);

    // Restrict tracking to these marker ids. An EMPTY list means "every marker"
    // and is the default -- the box is then the union of all detections, which
    // is what this tracker did before ids could be selected.
    //
    // Why restrict at all: the union of every marker is only cheap when the
    // markers sit close together. With floor anchors spread across the frame
    // and a robot in the middle, the union is nearly the whole frame and the
    // tracker saves nothing. Naming just the robot collapses the box onto it.
    //
    // The cost is that markers outside the box are no longer detected AT ALL --
    // they stop appearing in CAM_POSE. That is the point (it is why it is
    // faster), but it means an id filter must not be left on while anything
    // else depends on seeing the other markers.
    //
    // Like configure(), this restarts from SEARCH: the box being tracked was
    // built from markers that may no longer qualify.
    void setTrackIds(const std::vector<int>& ids);

    const std::vector<int>& trackIds() const { return trackIds_; }

    // True when `id` may drive the box. Always true while the filter is empty.
    bool tracksId(int id) const;

    // ROI to scan this frame.
    //
    // `manualRoi` is the operator's ROI and doubles as the SEARCH area (an
    // empty rect means full frame). `calibCollecting` suppresses tracking
    // outright: homography calibration needs every anchor, and the anchors are
    // spread across the whole frame, so narrowing the search would drop them.
    cv::Rect roiForFrame(const cv::Rect& manualRoi, bool calibCollecting) const;

    // Fold this frame's detections into the box. Emits DYNROI_STATE on a
    // SEARCH<->TRACK transition and, rate-limited, on a material margin change.
    void update(const std::vector<ArucoProcessor::Detection>& dets,
                const cv::Size& frame);

    bool enabled()  const { return enabled_; }
    bool tracking() const { return tracking_; }
    int  margin()   const { return margin_; }
    int  maxMiss()  const { return maxMiss_; }

    // Which lens this instance belongs to, stamped once at startup (the array
    // this lives in — SampleComponent::dynroi_[kMaxChannels] — does not tell
    // an element its own index). report() sends it on DYNROI_STATE so a
    // receiver with four of these open at once (the RPi dashboard, one per
    // channel) knows which one just changed — without it, every lens's
    // SEARCH<->TRACK transition looked identical on the wire (2026-08-10).
    void setChannel(int ch) { ch_ = ch; }

private:
    void setRoiClamped(cv::Rect r, const cv::Size& frame);
    void report() const;

    struct Center {
        int         id;
        cv::Point2f point;
        Center(int markerId, const cv::Point2f& p) : id(markerId), point(p) {}
    };

    int      ch_;                // -1 = never stamped (setChannel not called)
    bool     enabled_;
    bool     tracking_;          // false = SEARCH
    cv::Rect roi_;               // valid while tracking
    int      miss_;
    int      margin_;            // operator's UPPER BOUND on the margin (px)
    int      maxMiss_;           // consecutive misses before falling back
    int      activeMargin_;      // margin chosen from marker size + motion
    float    markerPx_;          // largest marker side seen in the last hit
    float    motionPx_;          // largest same-id centre movement in the hit
    int      reportedMargin_;
    int      framesSinceReport_;
    std::vector<Center> previousCenters_;
    std::vector<int>    trackIds_;   // empty = track every marker
};

#endif // DYN_ROI_H
