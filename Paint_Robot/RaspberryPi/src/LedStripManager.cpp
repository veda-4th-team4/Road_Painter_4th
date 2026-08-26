#include "LedStripManager.h"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <unistd.h>

namespace {

/* Tower lamp palette. Green is the WS2812B's brightest channel, so the manual
 * colour is toned down to roughly match the perceived brightness of the others. */
constexpr uint8_t kOrange[3] = { 255, 90, 0 };
constexpr uint8_t kGreen[3]  = { 0, 200, 40 };
constexpr uint8_t kWhite[3]  = { 255, 255, 255 };

/* Comet tail for the chase animation: head, then two dimming LEDs behind it. */
constexpr float kChaseTail[3] = { 1.0f, 0.35f, 0.12f };
constexpr uint32_t kChaseTicksPerStep = 4;   // ~130ms per LED at 30fps

/* One full red -> pink -> white -> pink -> red breath, in frames. */
constexpr uint32_t kEstopPeriodTicks = 72;   // ~2.4s at 30fps

uint8_t Scale(uint8_t value, int percent)
{
    return static_cast<uint8_t>(value * percent / 100);
}

} // namespace

LedStripManager::LedStripManager(int led_count, const std::string& dev_path)
    : led_count_(led_count > 0 ? led_count : kDefaultLedCount),
      dev_path_(dev_path),
      frame_(static_cast<size_t>(led_count_) * 3, 0),
      last_sent_(static_cast<size_t>(led_count_) * 3, 0xFF)
{
}

LedStripManager::~LedStripManager()
{
    Close();
}

const char* LedStripManager::StateName(LedState state)
{
    switch (state) {
    case LedState::IDLE:    return "IDLE";
    case LedState::DRIVING: return "DRIVING";
    case LedState::MANUAL:  return "MANUAL";
    case LedState::ESTOP:   return "ESTOP";
    }
    return "?";
}

void LedStripManager::SetBrightness(int percent)
{
    brightness_.store(std::max(0, std::min(100, percent)));
}

bool LedStripManager::Open()
{
    if (fd_ >= 0)
        return true;

    /* Enforce GPIO 12 as PWM0_0 (ALT0) to fix boot-time overrides */
    std::system("raspi-gpio set 12 a0 > /dev/null 2>&1");

    fd_ = ::open(dev_path_.c_str(), O_WRONLY);
    if (fd_ < 0) {
        std::cerr << "[LED] open " << dev_path_ << " failed: "
                  << std::strerror(errno) << std::endl;
        return false;
    }

    running_.store(true);
    thread_ = std::thread(&LedStripManager::RenderLoop, this);

    std::cout << "[LED] tower lamp ready on " << dev_path_
              << " (" << led_count_ << " LEDs)" << std::endl;
    return true;
}

void LedStripManager::Close()
{
    if (running_.exchange(false)) {
        if (thread_.joinable())
            thread_.join();
    }

    if (fd_ >= 0) {
        /* Leave the strip dark rather than frozen on the last frame. */
        Fill(0, 0, 0);
        last_sent_.assign(last_sent_.size(), 0xFF);
        Flush();
        ::close(fd_);
        fd_ = -1;
    }
}

void LedStripManager::RenderLoop()
{
    uint32_t tick = 0;

    while (running_.load()) {
        Render(state_.load(), tick++);
        Flush();
        std::this_thread::sleep_for(std::chrono::milliseconds(kFrameIntervalMs));
    }
}

void LedStripManager::Render(LedState state, uint32_t tick)
{
    switch (state) {
    case LedState::IDLE:    RenderIdle();          break;
    case LedState::DRIVING: RenderDriving(tick);   break;
    case LedState::MANUAL:  RenderManual();        break;
    case LedState::ESTOP:   RenderEstop(tick);     break;
    }
}

void LedStripManager::RenderIdle()
{
    Fill(kWhite[0], kWhite[1], kWhite[2]);
}

void LedStripManager::RenderManual()
{
    Fill(kGreen[0], kGreen[1], kGreen[2]);
}

/* A single orange pixel running along the strip with a short fading tail. */
void LedStripManager::RenderDriving(uint32_t tick)
{
    std::fill(frame_.begin(), frame_.end(), 0);

    const int head = static_cast<int>((tick / kChaseTicksPerStep) % led_count_);

    for (size_t t = 0; t < sizeof(kChaseTail) / sizeof(kChaseTail[0]); t++) {
        int index = head - static_cast<int>(t);
        while (index < 0)
            index += led_count_;

        SetPixel(index,
                 static_cast<uint8_t>(kOrange[0] * kChaseTail[t]),
                 static_cast<uint8_t>(kOrange[1] * kChaseTail[t]),
                 static_cast<uint8_t>(kOrange[2] * kChaseTail[t]));
    }
}

/*
 * Breathe between red and white. Interpolating red (255,0,0) towards white
 * passes through pink (255,128,128) on its own, so a plain triangle wave gives
 * the red -> pink -> white gradient and back with no special-casing.
 */
void LedStripManager::RenderEstop(uint32_t tick)
{
    const uint32_t half = kEstopPeriodTicks / 2;
    const uint32_t phase = tick % kEstopPeriodTicks;
    const float ramp = static_cast<float>(phase) / static_cast<float>(half);
    const float t = (phase < half) ? ramp : 2.0f - ramp;

    const uint8_t level = static_cast<uint8_t>(255.0f * t);
    Fill(255, level, level);
}

void LedStripManager::Fill(uint8_t r, uint8_t g, uint8_t b)
{
    for (int i = 0; i < led_count_; i++)
        SetPixel(i, r, g, b);
}

void LedStripManager::SetPixel(int index, uint8_t r, uint8_t g, uint8_t b)
{
    if (index < 0 || index >= led_count_)
        return;

    const int percent = brightness_.load();
    const size_t base = static_cast<size_t>(index) * 3;

    frame_[base + 0] = Scale(r, percent);
    frame_[base + 1] = Scale(g, percent);
    frame_[base + 2] = Scale(b, percent);
}

bool LedStripManager::Flush()
{
    if (fd_ < 0)
        return false;

    /* Static states render an identical frame every tick; skip those writes so
     * the driver is not asked to run a DMA transfer 30 times a second. */
    if (frame_ == last_sent_)
        return true;

    if (::lseek(fd_, 0, SEEK_SET) < 0) {
        if (!write_failed_logged_) {
            std::cerr << "[LED] lseek failed: " << std::strerror(errno) << std::endl;
            write_failed_logged_ = true;
        }
        return false;
    }

    const ssize_t written = ::write(fd_, frame_.data(), frame_.size());
    if (written != static_cast<ssize_t>(frame_.size())) {
        if (!write_failed_logged_) {
            std::cerr << "[LED] write failed: " << std::strerror(errno) << std::endl;
            write_failed_logged_ = true;
        }
        return false;
    }

    last_sent_ = frame_;
    return true;
}
