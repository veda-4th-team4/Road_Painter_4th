/*
 * Standalone audio_strip test tool.
 *
 * Exercises AudioStripManager without the network/UART stack, so playback
 * can be checked before wiring it into robot_exec. Mirrors led_test.cpp.
 */

#include "AudioStripManager.h"

#include <atomic>
#include <cerrno>
#include <chrono>
#include <climits>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>

namespace {

std::atomic<bool> g_running{true};

void OnSignal(int)
{
    g_running.store(false);
}

void Usage(const char* prog)
{
    std::cerr
        << "usage: " << prog << " [sound] [options]\n"
        << "\n"
        << "  sound       emergency_stop | low_battery | person_in_zone |\n"
        << "              power_off | power_on | signal_lost | snapshot |\n"
        << "              system_error | task_complete | task_start\n"
        << "              (omit to cycle through all ten, waiting for each\n"
        << "              to finish before moving on)\n"
        << "  --wav-dir   override the wav_files/ directory (default: "
        << AudioStripManager::DefaultWavDir() << ")\n"
        << "  --device    override the character device (default: /dev/audio_strip)\n"
        << "  -n N        repeat the selected sound N times (default: 1)\n"
        << "  -v PERCENT  software volume, 0-100 (default: 100)\n"
        << "  --list      print the sound catalog and exit\n"
        << "\n"
        << "Ctrl+C to stop.\n";
}

bool ParseSound(const std::string& name, SoundId* out)
{
    return AudioStripManager::ParseSoundName(name, *out);
}

bool ParseInteger(const char* text, int minimum, int maximum, int* out)
{
    errno = 0;
    char* end = nullptr;
    const long value = std::strtol(text, &end, 10);
    if (errno == ERANGE || end == text || !end || *end != '\0' ||
        value < minimum || value > maximum)
        return false;
    *out = static_cast<int>(value);
    return true;
}

const SoundId kAllSounds[] = {
    SoundId::EMERGENCY_STOP, SoundId::LOW_BATTERY, SoundId::PERSON_IN_ZONE,
    SoundId::POWER_OFF,      SoundId::POWER_ON,     SoundId::SIGNAL_LOST,
    SoundId::SNAPSHOT,       SoundId::SYSTEM_ERROR, SoundId::TASK_COMPLETE,
    SoundId::TASK_START,
};

} // namespace

int main(int argc, char** argv)
{
    std::string wav_dir = AudioStripManager::DefaultWavDir();
    std::string dev_path = "/dev/audio_strip";
    bool cycle = true;
    SoundId fixed_sound = SoundId::TASK_START;
    int repeat = 1;
    int volume = 100;

    for (int i = 1; i < argc; i++) {
        const std::string arg = argv[i];

        if (arg == "-h" || arg == "--help") {
            Usage(argv[0]);
            return 0;
        }
        if (arg == "--wav-dir" && i + 1 < argc) {
            wav_dir = argv[++i];
            continue;
        }
        if (arg == "--device" && i + 1 < argc) {
            dev_path = argv[++i];
            continue;
        }
        if ((arg == "-n" || arg == "--repeat") && i + 1 < argc) {
            if (!ParseInteger(argv[++i], 1, INT_MAX, &repeat)) {
                std::cerr << "repeat must be at least 1\n";
                return 1;
            }
            continue;
        }
        if ((arg == "-v" || arg == "--volume") && i + 1 < argc) {
            if (!ParseInteger(argv[++i], 0, 100, &volume)) {
                std::cerr << "volume must be between 0 and 100\n";
                return 1;
            }
            continue;
        }
        if (arg == "--list") {
            for (const SoundId s : kAllSounds)
                std::cout << AudioStripManager::SoundName(s) << "\n";
            return 0;
        }
        if (ParseSound(arg, &fixed_sound)) {
            cycle = false;
            continue;
        }

        std::cerr << "unknown argument: " << arg << "\n\n";
        Usage(argv[0]);
        return 1;
    }

    std::signal(SIGINT, OnSignal);
    std::signal(SIGTERM, OnSignal);

    AudioStripManager audio(wav_dir, dev_path);
    audio.SetVolume(volume);
    if (!audio.Open()) {
        std::cerr << "Is the driver loaded, and do you have write permission "
                     "on /dev/audio_strip?\n";
        return 1;
    }

    auto wait_until_done = [&audio]() {
        while (audio.IsBusy() && g_running.load())
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
    };

    if (!cycle) {
        std::cout << "sound = " << AudioStripManager::SoundName(fixed_sound)
                  << std::endl;
        for (int i = 0; i < repeat && g_running.load(); i++) {
            if (!audio.Play(fixed_sound))
                std::cerr << "Play() dropped (already playing?)\n";
            wait_until_done();
        }
    } else {
        std::cout << "cycling all sounds (Ctrl+C to stop)" << std::endl;
        for (const SoundId s : kAllSounds) {
            if (!g_running.load())
                break;
            std::cout << "  -> " << AudioStripManager::SoundName(s) << std::endl;
            if (!audio.Play(s))
                std::cerr << "Play() dropped (already playing?)\n";
            wait_until_done();
            std::this_thread::sleep_for(std::chrono::milliseconds(300));
        }
    }

    std::cout << "\ndone" << std::endl;
    audio.Close();
    return 0;
}
