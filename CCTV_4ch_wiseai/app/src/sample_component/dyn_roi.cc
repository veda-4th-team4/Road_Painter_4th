#include "dyn_roi.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <algorithm>

#include "app_config.h"
#include "pose_sender.h"

namespace {

// Tuning constants, named where they used to be bare literals in the middle of
// the update loop. Values are unchanged from aruco_detector_cv.cpp.
const int   kDefaultMargin   = 180;  // operator's upper bound at boot (px)
const int   kDefaultMaxMiss  = 4;    // misses tolerated before SEARCH

const float kAcquireMargins  = 2.f;  // marker sides of padding on first lock
const float kMotionFactor    = 1.5f; // of the last same-id movement
const float kJitterPad       = 10.f; // subpixel/corner jitter allowance (px)

const int   kAcquireFloor    = 80;   // margin floor while acquiring (px)
const int   kTrackFloor      = 50;   // margin floor while tracking (px)
const int   kMinBox          = 80;   // smallest ROI edge (px)

const int   kReportFrames    = 10;   // min frames between margin reports
const int   kReportDelta     = 10;   // min margin change worth reporting (px)

} // namespace

DynRoiTracker::DynRoiTracker()
    : ch_(-1), enabled_(false), tracking_(false), miss_(0),
      margin_(kDefaultMargin), maxMiss_(kDefaultMaxMiss),
      activeMargin_(0), markerPx_(0.f), motionPx_(0.f),
      reportedMargin_(-1), framesSinceReport_(0)
{
}

bool DynRoiTracker::configure(bool on, int maxMargin, int maxMiss)
{
    enabled_  = on;
    margin_   = maxMargin;
    maxMiss_  = maxMiss;

    // Always restart from a full search: the box currently being tracked was
    // built with the OLD margin, so keeping it would apply the new setting one
    // frame late and from the wrong starting size.
    tracking_          = false;
    miss_              = 0;
    activeMargin_      = 0;
    markerPx_          = 0.f;
    motionPx_          = 0.f;
    reportedMargin_    = -1;
    framesSinceReport_ = 0;
    previousCenters_.clear();

    return enabled_;
}

bool DynRoiTracker::tracksId(int id) const
{
    if (trackIds_.empty())
        return true;                    // no filter = every marker
    for (size_t i = 0; i < trackIds_.size(); ++i)
        if (trackIds_[i] == id)
            return true;
    return false;
}

void DynRoiTracker::setTrackIds(const std::vector<int>& ids)
{
    trackIds_ = ids;

    // Same reasoning as configure(): the current box was built from whatever
    // was visible under the OLD filter, so keeping it would track markers that
    // no longer qualify until the next miss.
    tracking_          = false;
    miss_              = 0;
    activeMargin_      = 0;
    markerPx_          = 0.f;
    motionPx_          = 0.f;
    reportedMargin_    = -1;
    framesSinceReport_ = 0;
    previousCenters_.clear();
}

cv::Rect DynRoiTracker::roiForFrame(const cv::Rect& manualRoi,
                                    bool calibCollecting) const
{
    if (calibCollecting)
        return manualRoi;
    if (!enabled_ || !tracking_)
        return manualRoi;          // SEARCH (empty rect = full frame)
    return roi_;
}

void DynRoiTracker::update(const std::vector<ArucoProcessor::Detection>& dets,
                           const cv::Size& frame)
{
    if (!enabled_)
        return;

    // Apply the id filter before anything else. A frame that is full of markers
    // the filter rejects has to behave like a MISS, not like "nothing was
    // detected" -- otherwise non-tracked markers would keep the box alive on a
    // position the tracked marker left long ago.
    std::vector<const ArucoProcessor::Detection*> hits;
    for (size_t i = 0; i < dets.size(); ++i)
        if (tracksId(dets[i].id))
            hits.push_back(&dets[i]);

    if (!hits.empty()) {
        // Union of every tracked detection: with several robots in view the box
        // grows to cover them all instead of following just one.
        float x0 = 1e9f, y0 = 1e9f, x1 = -1e9f, y1 = -1e9f;
        float largestMarker = 0.f;
        float largestMotion = 0.f;
        bool  haveMatchedCenter = false;
        std::vector<Center> currentCenters;

        for (size_t i = 0; i < hits.size(); ++i) {
            const std::vector<cv::Point2f>& c = hits[i]->corners2d;
            if (c.empty())
                continue;
            cv::Point2f center(0.f, 0.f);
            for (size_t k = 0; k < c.size(); ++k) {
                if (c[k].x < x0) x0 = c[k].x;
                if (c[k].y < y0) y0 = c[k].y;
                if (c[k].x > x1) x1 = c[k].x;
                if (c[k].y > y1) y1 = c[k].y;
                center += c[k];
            }
            center *= 1.f / (float) c.size();
            currentCenters.push_back(Center(hits[i]->id, center));

            // Median of the four side lengths is robust to one noisy corner
            // and follows perspective/zoom better than bbox width.
            if (c.size() >= 4) {
                float sides[4];
                for (int k = 0; k < 4; ++k)
                    sides[k] = (float) cv::norm(c[(k + 1) % 4] - c[k]);
                std::sort(sides, sides + 4);
                const float side = 0.5f * (sides[1] + sides[2]);
                if (side > largestMarker)
                    largestMarker = side;
            }

            for (size_t k = 0; k < previousCenters_.size(); ++k) {
                if (previousCenters_[k].id != hits[i]->id)
                    continue;
                const float movement =
                    (float) cv::norm(center - previousCenters_[k].point);
                if (movement > largestMotion)
                    largestMotion = movement;
                haveMatchedCenter = true;
                break;
            }
        }
        if (x1 < x0 || y1 < y0)
            return;                       // no usable corners

        markerPx_ = largestMarker;
        motionPx_ = haveMatchedCenter ? largestMotion : 0.f;

        // First acquisition has no velocity estimate, so start with two marker
        // sides of padding. During TRACK, reserve one marker side for
        // rotation/scale change plus 1.5x the last same-id movement. A few
        // extra pixels absorb subpixel/corner jitter. The configured margin
        // remains a hard operator-controlled upper bound.
        const float wanted = (!tracking_ || !haveMatchedCenter)
                           ? kAcquireMargins * markerPx_
                           : markerPx_ + kMotionFactor * motionPx_ + kJitterPad;
        const int safeFloor = (!tracking_ || !haveMatchedCenter)
                            ? kAcquireFloor : kTrackFloor;
        const int minMargin = margin_ < safeFloor ? margin_ : safeFloor;
        int targetMargin = (int) floor(wanted + 0.5f);
        if (targetMargin < minMargin) targetMargin = minMargin;
        if (targetMargin > margin_)   targetMargin = margin_;

        // Grow immediately when the robot accelerates; shrink gradually so one
        // slow frame cannot make the following ROI too tight.
        if (!tracking_ || activeMargin_ <= 0 || targetMargin >= activeMargin_)
            activeMargin_ = targetMargin;
        else
            activeMargin_ = (3 * activeMargin_ + targetMargin + 2) / 4;

        setRoiClamped(cv::Rect((int) x0 - activeMargin_,
                               (int) y0 - activeMargin_,
                               (int) (x1 - x0) + 2 * activeMargin_,
                               (int) (y1 - y0) + 2 * activeMargin_), frame);
        previousCenters_.swap(currentCenters);
        const bool wasTracking = tracking_;
        tracking_ = true;
        miss_     = 0;
        if (!wasTracking) {
            report();                      // SEARCH -> TRACK
            reportedMargin_    = activeMargin_;
            framesSinceReport_ = 0;
        } else {
            // Keep the dashboard useful for tuning without adding one control
            // packet per frame: report a material margin change at most once
            // per ten successful tracking frames.
            ++framesSinceReport_;
            if (framesSinceReport_ >= kReportFrames &&
                abs(activeMargin_ - reportedMargin_) >= kReportDelta) {
                report();
                reportedMargin_    = activeMargin_;
                framesSinceReport_ = 0;
            }
        }
        return;
    }

    if (!tracking_)
        return;                            // already searching

    if (++miss_ > maxMiss_) {
        tracking_ = false;                 // lost -> full search
        previousCenters_.clear();
        activeMargin_      = 0;
        markerPx_          = 0.f;
        motionPx_          = 0.f;
        reportedMargin_    = -1;
        framesSinceReport_ = 0;
        report();                          // TRACK -> SEARCH
        return;
    }
    // Widen by 50% around the same centre and try again next frame.
    const int gw = roi_.width / 2, gh = roi_.height / 2;
    setRoiClamped(cv::Rect(roi_.x - gw / 2, roi_.y - gh / 2,
                           roi_.width + gw, roi_.height + gh), frame);
}

void DynRoiTracker::setRoiClamped(cv::Rect r, const cv::Size& frame)
{
    // A box smaller than this leaves no room for the marker to move between
    // frames and just generates misses.
    if (r.width  < kMinBox) { r.x -= (kMinBox - r.width)  / 2; r.width  = kMinBox; }
    if (r.height < kMinBox) { r.y -= (kMinBox - r.height) / 2; r.height = kMinBox; }
    r &= cv::Rect(0, 0, frame.width, frame.height);
    roi_ = r;
}

void DynRoiTracker::report() const
{
    char json[256];
    snprintf(json, sizeof(json),
             "{\"type\":\"DYNROI_STATE\",\"ch\":%d,\"tracking\":%d,"
             "\"x\":%d,\"y\":%d,\"w\":%d,\"h\":%d,"
             "\"margin_used\":%d,\"marker_px\":%.1f,\"motion_px\":%.1f}",
             ch_, tracking_ ? 1 : 0, roi_.x, roi_.y, roi_.width, roi_.height,
             activeMargin_, markerPx_, motionPx_);
    pose_sender_send_control_line(json);
}
