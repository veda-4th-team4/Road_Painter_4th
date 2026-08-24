#ifndef __AUDIO_STRIP_MANAGER_H__
#define __AUDIO_STRIP_MANAGER_H__

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

#include "AudioStripDevice.h"

/**
 * @brief Sounds available to the robot, mapped 1:1 to the .wav files in
 *        audio/wav_files/ (see that project's README "Sound Catalog"). Keep this enum and
 *        SoundName() in sync with that table when either one changes.
 */
enum class SoundId {
    EMERGENCY_STOP,
    LOW_BATTERY,
    PERSON_IN_ZONE,
    POWER_OFF,
    POWER_ON,
    SIGNAL_LOST,
    SNAPSHOT,
    SYSTEM_ERROR,
    TASK_COMPLETE,
    TASK_START,
};

/**
 * @brief Userspace half of the raw I2S audio driver (audio_strip_driver.c).
 *
 * The kernel module owns the PCM/DMA timing and exposes /dev/audio_strip,
 * which takes raw 16-bit LE stereo PCM in fixed-size chunks. This class
 * owns turning a WAV file into that PCM stream: a background thread waits
 * for Play() requests and, when one arrives, reads the requested WAV file
 * and writes it to the device in period-sized chunks - callers only ever
 * request a sound and never block on file or device I/O.
 *
 * Structural note vs. LedStripManager: LedStripManager's thread renders
 * continuously (~30fps) because LED state is "what to show right now, held
 * until it changes". Audio is edge-triggered instead - Play() names a
 * one-shot event, not a state to hold - so this thread is idle (blocked on
 * a condition variable) except while a sound is actually playing, and a
 * request arriving while something is already playing is accepted only if
 * it has a higher priority. Safety announcements can therefore preempt a
 * routine message without allowing a burst of equal-priority messages to
 * build a queue.
 *
 * Usage:
 *     AudioStripManager audio;
 *     audio.Open();
 *     audio.Play(SoundId::TASK_START);
 */
class AudioStripManager {
public:
    static constexpr size_t kPeriodBytes = AUDIO_STRIP_PERIOD_BYTES;

    explicit AudioStripManager(std::string wav_dir = DefaultWavDir(),
                               std::string dev_path = "/dev/audio_strip");
    ~AudioStripManager();

    AudioStripManager(const AudioStripManager&) = delete;
    AudioStripManager& operator=(const AudioStripManager&) = delete;

    /** Opens the device and starts the playback thread. */
    bool Open();

    /** Stops any playback in progress, stops the thread, closes the device. */
    void Close();

    bool IsOpen() const { return fd_ >= 0; }

    /**
     * Cheap and thread-safe, like LedStripManager::SetState(). Requests
     * playback of one sound. Higher-priority safety sounds preempt the
     * current sound; equal/lower-priority requests are dropped while busy.
     */
    bool Play(SoundId sound);

    bool IsPlaying() const { return playing_.load(); }
    bool IsBusy() const { return pending_.load() || playing_.load(); }

    /** Software gain applied to 16-bit samples before they reach the driver. */
    void SetVolume(int percent);
    int GetVolume() const { return volume_.load(); }

    static const char* SoundName(SoundId sound);
    static bool ParseSoundName(const std::string& name, SoundId& sound);

    /** Repository/install path, optionally overridden by ROADPAINTER_AUDIO_DIR. */
    static std::string DefaultWavDir();

private:
    void WorkerLoop();
    bool PlayFile(const std::string& path, uint64_t generation);
    bool DropDeviceStream();
    static int Priority(SoundId sound);

    int fd_ = -1;
    std::string wav_dir_;
    std::string dev_path_;

    std::atomic<bool> running_{false};
    std::atomic<bool> playing_{false};
    std::atomic<bool> pending_{false};
    std::atomic<int> volume_{100};
    std::atomic<uint64_t> playback_generation_{1};

    std::mutex mutex_;
    std::condition_variable cv_;
    bool has_request_ = false;
    SoundId requested_sound_ = SoundId::TASK_START;
    int active_priority_ = -1;

    std::thread thread_;
};

#endif /* __AUDIO_STRIP_MANAGER_H__ */
