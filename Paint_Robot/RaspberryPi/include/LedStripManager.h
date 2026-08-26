#ifndef __LED_STRIP_MANAGER_H__
#define __LED_STRIP_MANAGER_H__

#include <atomic>
#include <cstdint>
#include <string>
#include <thread>
#include <vector>

/**
 * @brief Tower lamp states surfaced to the operator (SR-MPU-002).
 */
enum class LedState {
    IDLE,     // 대기        - 흰색 고정
    DRIVING,  // 자율 주행   - 주황 chase
    MANUAL,   // 수동 조작   - 초록 고정
    ESTOP,    // 비상 정지   - 빨강 <-> 흰색 (핑크 경유) 그라데이션
};

/**
 * @brief Userspace half of the WS2812B tower lamp driver.
 *
 * The kernel module owns the PWM/DMA timing and exposes /dev/led_strip, which
 * takes three bytes (R, G, B) per LED. This class owns the animation: a
 * background thread renders frames at a fixed rate and writes them down, so
 * callers only ever set a state and never block on I/O.
 *
 * Usage:
 *     LedStripManager led(7);
 *     led.Open();
 *     led.SetState(LedState::DRIVING);
 */
class LedStripManager {
public:
    static constexpr int kDefaultLedCount = 5;
    static constexpr int kFrameIntervalMs = 33;   // ~30 fps

    explicit LedStripManager(int led_count = kDefaultLedCount,
                             const std::string& dev_path = "/dev/led_strip");
    ~LedStripManager();

    LedStripManager(const LedStripManager&) = delete;
    LedStripManager& operator=(const LedStripManager&) = delete;

    /** Opens the device and starts the render thread. */
    bool Open();

    /** Blanks the strip, stops the render thread and closes the device. */
    void Close();

    bool IsOpen() const { return fd_ >= 0; }

    /** Cheap and thread-safe; the render thread picks the change up next frame. */
    void SetState(LedState state) { state_.store(state); }
    LedState GetState() const { return state_.load(); }

    /**
     * Caps output so the strip stays within the supply budget. A WS2812B draws
     * up to 60mA at full white, which the Pi 5V pin cannot sustain for long
     * runs, so the default is deliberately low.
     */
    void SetBrightness(int percent);
    int GetBrightness() const { return brightness_.load(); }

    static const char* StateName(LedState state);

private:
    void RenderLoop();
    void Render(LedState state, uint32_t tick);

    void RenderIdle();
    void RenderDriving(uint32_t tick);
    void RenderManual();
    void RenderEstop(uint32_t tick);

    void Fill(uint8_t r, uint8_t g, uint8_t b);
    void SetPixel(int index, uint8_t r, uint8_t g, uint8_t b);
    bool Flush();

    int fd_ = -1;
    int led_count_;
    std::string dev_path_;

    std::vector<uint8_t> frame_;      // being rendered
    std::vector<uint8_t> last_sent_;  // suppresses redundant writes

    std::atomic<LedState> state_{LedState::IDLE};
    std::atomic<int> brightness_{25};
    std::atomic<bool> running_{false};
    std::thread thread_;
    bool write_failed_logged_ = false;
};

#endif /* __LED_STRIP_MANAGER_H__ */
