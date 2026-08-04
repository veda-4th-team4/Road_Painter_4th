#include "detect_tuning.h"

#include <cstdio>
#include <opencv2/core.hpp>

#include "app_config.h"
#include "aruco_processor.h"

// File format version. Bump when the field count/order changes so an old file
// fails the check below and falls back to defaults instead of being misparsed
// into a nonsense ROI -- the same guard intrinsics uses.
static const int kTuningFileVersion = 1;

static bool        g_loaded     = false;
static const char* g_failReason = "";

bool detect_tuning_loaded(void) { return g_loaded; }
const char* detect_tuning_fail_reason(void) { return g_failReason; }

bool detect_tuning_init(ArucoProcessor* proc)
{
    g_loaded = false;
    if (proc == NULL)
        return false;

    FILE* f = fopen(DETECT_TUNING_FILE, "r");
    if (f == NULL)
        return false; // never saved on this camera -- defaults stand

    int version = 0, x = 0, y = 0, w = 0, h = 0, passes = 0, win = 0;
    int n = fscanf(f, "%d %d %d %d %d %d %d",
                   &version, &x, &y, &w, &h, &passes, &win);
    fclose(f);

    if (n != 7 || version != kTuningFileVersion)
        return false;
    // Reject only what cannot be meaningful. The ROI is NOT checked against the
    // frame here because the frame size is unknown until the first callback;
    // ArucoProcessor::detect() already falls back to the full frame when the
    // rect does not fit inside the image, so an oversized ROI degrades to
    // "slow but correct" rather than to "finds nothing".
    if (x < 0 || y < 0 || w < 0 || h < 0 || passes < 1 || passes > 3 || win < 3)
        return false;

    proc->setRoi(cv::Rect(x, y, w, h));
    // The scan-pass count is restored along with the ROI.
    //
    // An earlier revision deliberately dropped it, so every boot started at the
    // 3-pass default: a narrowed sweep that survives a power cycle silently
    // stops reporting markers only found at window 3 or 23, with nothing on
    // screen tying the loss to a setting changed days earlier.
    //
    // Field data since then inverted that trade-off. The 3-pass default costs
    // ~500 ms per frame on this installation against a 100 ms frame interval,
    // so the camera cannot keep up at the default and has been restarting under
    // that load. Coming back at the slowest setting after every restart makes a
    // loop out of it. A saved setting that keeps the camera inside its frame
    // budget is worth more than protection from a silently narrowed sweep --
    // the sweep at least still detects, whereas a camera that restarts detects
    // nothing at all.
    //
    // Saving stays explicit (TUNE_SAVE), so an experiment is still undone by a
    // restart unless it was deliberately committed, and TUNE_QUERY reports the
    // "persisted" flag so a stale setting remains findable.
    proc->setScanPasses(passes, win);
    g_loaded = true;
    return true;
}

bool detect_tuning_save(const ArucoProcessor* proc)
{
    if (proc == NULL) {
        g_failReason = "detector not running";
        return false;
    }
#if !PERSIST_TO_MNT
    g_failReason = "persistence disabled at build time (PERSIST_TO_MNT 0)";
    return false;
#else
    FILE* f = fopen(DETECT_TUNING_FILE, "w");
    if (f == NULL) {
        g_failReason = "write to " PERSIST_DIR " failed (see Shell tab /mnt checks)";
        return false;
    }
    const cv::Rect r = proc->roi();
    fprintf(f, "%d %d %d %d %d %d %d\n",
            kTuningFileVersion,
            r.x, r.y, r.width, r.height,
            proc->scanPasses(), proc->scanWin());
    fclose(f);
    g_loaded     = true;
    g_failReason = "";
    return true;
#endif
}

bool detect_tuning_clear(void)
{
#if !PERSIST_TO_MNT
    g_failReason = "persistence disabled at build time (PERSIST_TO_MNT 0)";
    return false;
#else
    // remove() reporting "no such file" is the desired end state, not an error,
    // so a failure only counts if the file is still readable afterwards.
    if (remove(DETECT_TUNING_FILE) != 0) {
        FILE* f = fopen(DETECT_TUNING_FILE, "r");
        if (f != NULL) {
            fclose(f);
            g_failReason = "could not remove " DETECT_TUNING_FILE;
            return false;
        }
    }
    g_loaded     = false;
    g_failReason = "";
    return true;
#endif
}
