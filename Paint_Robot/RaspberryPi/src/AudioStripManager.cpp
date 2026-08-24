#include "AudioStripManager.h"

#include <algorithm>
#include <array>
#include <cerrno>
#include <climits>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <system_error>
#include <sys/stat.h>
#include <unistd.h>
#include <vector>

namespace {

#pragma pack(push, 1)
struct RiffHeader {
    char riff_id[4];
    uint32_t riff_size;
    char wave_id[4];
};

struct ChunkHeader {
    char id[4];
    uint32_t size;
};

struct FmtChunk {
    uint16_t audio_format;
    uint16_t num_channels;
    uint32_t sample_rate;
    uint32_t byte_rate;
    uint16_t block_align;
    uint16_t bits_per_sample;
};
#pragma pack(pop)

constexpr uint32_t kExpectedSampleRate = 44100;
constexpr uint16_t kExpectedBlockAlign = 4;

struct SoundEntry {
    const char* name;
    SoundId id;
};

constexpr SoundEntry kSounds[] = {
    {"emergency_stop", SoundId::EMERGENCY_STOP},
    {"low_battery", SoundId::LOW_BATTERY},
    {"person_in_zone", SoundId::PERSON_IN_ZONE},
    {"power_off", SoundId::POWER_OFF},
    {"power_on", SoundId::POWER_ON},
    {"signal_lost", SoundId::SIGNAL_LOST},
    {"snapshot", SoundId::SNAPSHOT},
    {"system_error", SoundId::SYSTEM_ERROR},
    {"task_complete", SoundId::TASK_COMPLETE},
    {"task_start", SoundId::TASK_START},
};

bool ChunkIs(const ChunkHeader& header, const char* id)
{
    return std::memcmp(header.id, id, 4) == 0;
}

bool IsDirectory(const std::string& path)
{
    struct stat st {};
    return ::stat(path.c_str(), &st) == 0 && S_ISDIR(st.st_mode);
}

std::string ParentDirectory(const std::string& path)
{
    const std::string::size_type slash = path.find_last_of('/');
    if (slash == std::string::npos)
        return ".";
    if (slash == 0)
        return "/";
    return path.substr(0, slash);
}

bool ReadExact(int fd, void* buffer, size_t count)
{
    auto* out = static_cast<uint8_t*>(buffer);
    size_t done = 0;
    while (done < count) {
        const ssize_t n = ::read(fd, out + done, count - done);
        if (n > 0) {
            done += static_cast<size_t>(n);
            continue;
        }
        if (n < 0 && errno == EINTR)
            continue;
        return false;
    }
    return true;
}

} // namespace

std::string AudioStripManager::DefaultWavDir()
{
    if (const char* override_dir = std::getenv("ROADPAINTER_AUDIO_DIR")) {
        if (*override_dir)
            return override_dir;
    }

    std::vector<std::string> candidates;
#ifdef ROADPAINTER_AUDIO_DIR
    candidates.emplace_back(ROADPAINTER_AUDIO_DIR);
#endif

    std::array<char, PATH_MAX> executable{};
    const ssize_t length = ::readlink("/proc/self/exe", executable.data(),
                                      executable.size() - 1);
    if (length > 0) {
        executable[static_cast<size_t>(length)] = '\0';
        const std::string bin_dir = ParentDirectory(executable.data());
        candidates.push_back(bin_dir + "/../audio/wav_files");
        candidates.push_back(bin_dir + "/audio/wav_files");
    }

    candidates.emplace_back("audio/wav_files");
    candidates.emplace_back("../audio/wav_files");
    candidates.emplace_back("/opt/road-painter/audio/wav_files");

    for (const std::string& candidate : candidates) {
        if (IsDirectory(candidate))
            return candidate;
    }

    return candidates.empty() ? "/opt/road-painter/audio/wav_files"
                              : candidates.front();
}

const char* AudioStripManager::SoundName(SoundId sound)
{
    for (const SoundEntry& entry : kSounds) {
        if (entry.id == sound)
            return entry.name;
    }
    return "unknown";
}

bool AudioStripManager::ParseSoundName(const std::string& name, SoundId& sound)
{
    for (const SoundEntry& entry : kSounds) {
        if (name == entry.name) {
            sound = entry.id;
            return true;
        }
    }
    return false;
}

int AudioStripManager::Priority(SoundId sound)
{
    switch (sound) {
    case SoundId::EMERGENCY_STOP: return 100;
    case SoundId::PERSON_IN_ZONE: return 90;
    case SoundId::LOW_BATTERY:
    case SoundId::SIGNAL_LOST:
    case SoundId::SYSTEM_ERROR:   return 70;
    case SoundId::POWER_OFF:      return 60;
    case SoundId::SNAPSHOT:
    case SoundId::TASK_COMPLETE:
    case SoundId::TASK_START:     return 10;
    case SoundId::POWER_ON:       return 5;
    }
    return 0;
}

AudioStripManager::AudioStripManager(std::string wav_dir, std::string dev_path)
    : wav_dir_(std::move(wav_dir)), dev_path_(std::move(dev_path))
{
}

AudioStripManager::~AudioStripManager()
{
    Close();
}

bool AudioStripManager::Open()
{
    if (fd_ >= 0)
        return true;

    // Ensure BCM2711 I2S pins (GPIO 18, 19, 21) are firmly set to ALT0 (PCM_CLK, PCM_FS, PCM_DOUT)
    ::system("raspi-gpio set 18 a0 2>/dev/null; raspi-gpio set 19 a0 2>/dev/null; raspi-gpio set 21 a0 2>/dev/null; pinctrl set 18 a0 2>/dev/null; pinctrl set 19 a0 2>/dev/null; pinctrl set 21 a0 2>/dev/null");

    if (!IsDirectory(wav_dir_)) {
        std::cerr << "[Audio] WAV directory not found: " << wav_dir_ << std::endl;
        return false;
    }
    for (const SoundEntry& entry : kSounds) {
        const std::string path = wav_dir_ + "/" + entry.name + ".wav";
        if (::access(path.c_str(), R_OK) != 0) {
            std::cerr << "[Audio] required WAV is not readable: " << path
                      << std::endl;
            return false;
        }
    }

    fd_ = ::open(dev_path_.c_str(), O_WRONLY | O_CLOEXEC);
    if (fd_ < 0) {
        std::cerr << "[Audio] open " << dev_path_ << " failed: "
                  << std::strerror(errno) << std::endl;
        return false;
    }

    if (::ioctl(fd_, AUDIO_STRIP_IOC_DROP) < 0) {
        std::cerr << "[Audio] " << dev_path_
                  << " does not support the required DRAIN/DROP API: "
                  << std::strerror(errno) << std::endl;
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    running_.store(true);
    try {
        thread_ = std::thread(&AudioStripManager::WorkerLoop, this);
    } catch (const std::system_error& error) {
        std::cerr << "[Audio] playback thread failed: " << error.what()
                  << std::endl;
        running_.store(false);
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    std::cout << "[Audio] ready on " << dev_path_
              << " (wav_dir=" << wav_dir_ << ")" << std::endl;
    return true;
}

void AudioStripManager::Close()
{
    if (running_.exchange(false)) {
        playback_generation_.fetch_add(1);
        DropDeviceStream();
        {
            std::lock_guard<std::mutex> lock(mutex_);
            has_request_ = false;
            pending_.store(false);
        }
        cv_.notify_one();
        if (thread_.joinable())
            thread_.join();
    }

    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
    playing_.store(false);
    pending_.store(false);
}

void AudioStripManager::SetVolume(int percent)
{
    volume_.store(std::max(0, std::min(100, percent)));
}

bool AudioStripManager::DropDeviceStream()
{
    if (fd_ < 0)
        return false;
    if (::ioctl(fd_, AUDIO_STRIP_IOC_DROP) == 0)
        return true;
    if (errno != ECANCELED)
        std::cerr << "[Audio] DROP failed: " << std::strerror(errno) << std::endl;
    return false;
}

bool AudioStripManager::Play(SoundId sound)
{
    bool preempt = false;
    const int priority = Priority(sound);

    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!running_.load() || fd_ < 0)
            return false;

        int busy_priority = -1;
        if (playing_.load())
            busy_priority = active_priority_;
        if (has_request_)
            busy_priority = std::max(busy_priority, Priority(requested_sound_));
        if (busy_priority >= priority)
            return false;

        requested_sound_ = sound;
        has_request_ = true;
        pending_.store(true);
        if (playing_.load()) {
            playback_generation_.fetch_add(1);
            preempt = true;
        }
    }

    if (preempt)
        DropDeviceStream();
    cv_.notify_one();
    return true;
}

void AudioStripManager::WorkerLoop()
{
    while (running_.load()) {
        SoundId sound;
        uint64_t generation;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            cv_.wait(lock, [this] { return has_request_ || !running_.load(); });
            if (!running_.load())
                break;
            sound = requested_sound_;
            has_request_ = false;
            pending_.store(false);
            active_priority_ = Priority(sound);
            generation = playback_generation_.load();
            playing_.store(true);
        }

        const std::string path = wav_dir_ + "/" + SoundName(sound) + ".wav";
        if (!PlayFile(path, generation) &&
            generation == playback_generation_.load() && running_.load()) {
            std::cerr << "[Audio] failed to play " << path << std::endl;
        }

        {
            std::lock_guard<std::mutex> lock(mutex_);
            active_priority_ = -1;
            playing_.store(false);
        }
    }
}

bool AudioStripManager::PlayFile(const std::string& path, uint64_t generation)
{
    const auto cancelled = [this, generation] {
        return !running_.load() || generation != playback_generation_.load();
    };

    const int wav_fd = ::open(path.c_str(), O_RDONLY | O_CLOEXEC);
    if (wav_fd < 0) {
        std::cerr << "[Audio] open " << path << " failed: "
                  << std::strerror(errno) << std::endl;
        return false;
    }

    RiffHeader riff{};
    if (!ReadExact(wav_fd, &riff, sizeof(riff)) ||
        std::memcmp(riff.riff_id, "RIFF", 4) != 0 ||
        std::memcmp(riff.wave_id, "WAVE", 4) != 0) {
        std::cerr << "[Audio] " << path << ": not a RIFF/WAVE file" << std::endl;
        ::close(wav_fd);
        return false;
    }

    FmtChunk fmt{};
    bool have_fmt = false;
    off_t data_offset = -1;
    uint32_t data_size = 0;

    for (;;) {
        ChunkHeader chunk{};
        if (!ReadExact(wav_fd, &chunk, sizeof(chunk)))
            break;

        if (ChunkIs(chunk, "fmt ")) {
            const size_t to_read = std::min<size_t>(chunk.size, sizeof(fmt));
            if (!ReadExact(wav_fd, &fmt, to_read))
                break;
            if (chunk.size > to_read &&
                ::lseek(wav_fd, chunk.size - to_read, SEEK_CUR) < 0)
                break;
            if ((chunk.size & 1U) && ::lseek(wav_fd, 1, SEEK_CUR) < 0)
                break;
            have_fmt = chunk.size >= sizeof(FmtChunk);
        } else if (ChunkIs(chunk, "data")) {
            data_offset = ::lseek(wav_fd, 0, SEEK_CUR);
            data_size = chunk.size;
            break;
        } else if (::lseek(wav_fd, chunk.size + (chunk.size & 1U), SEEK_CUR) < 0) {
            break;
        }
    }

    if (!have_fmt || data_offset < 0 || data_size == 0) {
        std::cerr << "[Audio] " << path << ": missing fmt/data or empty data"
                  << std::endl;
        ::close(wav_fd);
        return false;
    }
    if (fmt.audio_format != 1 || fmt.num_channels != 2 ||
        fmt.bits_per_sample != 16 || fmt.block_align != kExpectedBlockAlign ||
        fmt.sample_rate != kExpectedSampleRate || data_size % kExpectedBlockAlign != 0) {
        std::cerr << "[Audio] " << path
                  << ": expected 44100 Hz 16-bit stereo PCM" << std::endl;
        ::close(wav_fd);
        return false;
    }

    if (::lseek(wav_fd, data_offset, SEEK_SET) < 0) {
        ::close(wav_fd);
        return false;
    }

    alignas(int16_t) std::array<uint8_t, kPeriodBytes> buffer{};
    uint32_t remaining = data_size;
    bool submitted = false;
    bool ok = true;

    while (!cancelled() && remaining > 0) {
        const size_t wanted = std::min<size_t>(remaining, buffer.size());
        if (!ReadExact(wav_fd, buffer.data(), wanted)) {
            ok = false;
            break;
        }
        if (wanted < buffer.size())
            std::fill(buffer.begin() + wanted, buffer.end(), 0);

        const int volume = volume_.load();
        if (volume != 100) {
            auto* samples = reinterpret_cast<int16_t*>(buffer.data());
            const size_t sample_count = wanted / sizeof(int16_t);
            for (size_t i = 0; i < sample_count; i++)
                samples[i] = static_cast<int16_t>(static_cast<int>(samples[i]) * volume / 100);
        }

        ssize_t written;
        do {
            written = ::write(fd_, buffer.data(), buffer.size());
        } while (written < 0 && errno == EINTR && !cancelled());

        if (written != static_cast<ssize_t>(buffer.size())) {
            if (!(cancelled() || errno == ECANCELED))
                std::cerr << "[Audio] write failed: " << std::strerror(errno) << std::endl;
            ok = false;
            break;
        }

        submitted = true;
        remaining -= static_cast<uint32_t>(wanted);
    }

    ::close(wav_fd);

    if (cancelled())
        return false;
    if (!ok || !submitted) {
        DropDeviceStream();
        return false;
    }

    if (::ioctl(fd_, AUDIO_STRIP_IOC_DRAIN) < 0) {
        if (errno != ECANCELED)
            std::cerr << "[Audio] DRAIN failed: " << std::strerror(errno) << std::endl;
        DropDeviceStream();
        return false;
    }
    return true;
}
