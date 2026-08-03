#include "calib_view_store.h"

#include <mutex>
#include <deque>
#include <string>
#include <string.h>

static std::mutex g_mtx;
static std::vector<CalibViewJpeg> g_views;

static std::mutex g_progMtx;
static std::deque<std::string> g_progress;

void calib_view_store_reset(void)
{
    std::lock_guard<std::mutex> lk(g_mtx);
    g_views.clear();
}

void calib_view_store_pop_last(void)
{
    std::lock_guard<std::mutex> lk(g_mtx);
    if (!g_views.empty())
        g_views.pop_back();
}

int calib_view_store_count(void)
{
    std::lock_guard<std::mutex> lk(g_mtx);
    return (int) g_views.size();
}

void calib_view_store_add(int view, int target, int corners,
                          int width, int height,
                          std::vector<CalibViewCorner>&& points,
                          std::vector<uint8_t>&& jpeg,
                          std::vector<uint8_t>&& jpeg_plain)
{
    std::lock_guard<std::mutex> lk(g_mtx);
    CalibViewJpeg v;
    v.view       = view;
    v.target     = target;
    v.corners    = corners;
    v.width      = width;
    v.height     = height;
    v.points     = std::move(points);
    v.jpeg       = std::move(jpeg);
    v.jpeg_plain = std::move(jpeg_plain);
    g_views.push_back(std::move(v));
}

bool calib_view_store_get(int i, CalibViewJpeg& out)
{
    std::lock_guard<std::mutex> lk(g_mtx);
    if (i < 0 || i >= (int) g_views.size())
        return false;
    out = g_views[i];   // full copy, including the JPEG bytes
    return true;
}

void calib_view_progress_push(const char* line)
{
    if (line == NULL)
        return;
    std::lock_guard<std::mutex> lk(g_progMtx);
    g_progress.push_back(std::string(line));
}

bool calib_view_progress_pop(char* out, int out_len)
{
    if (out == NULL || out_len <= 0)
        return false;
    std::lock_guard<std::mutex> lk(g_progMtx);
    if (g_progress.empty())
        return false;
    const std::string& s = g_progress.front();
    strncpy(out, s.c_str(), out_len - 1);
    out[out_len - 1] = '\0';
    g_progress.pop_front();
    return true;
}
