/*
 * Standalone tower lamp test tool.
 *
 * Exercises LedStripManager without the network/UART stack, so the LED
 * behaviour can be checked before wiring it into robot_exec.
 */

#include "LedStripManager.h"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstring>
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
        << "usage: " << prog << " [state] [--leds N] [--brightness P]\n"
        << "\n"
        << "  state       idle | driving | manual | estop\n"
        << "              (omit to cycle through all four, 6s each)\n"
        << "  --leds      LED count (default " << LedStripManager::kDefaultLedCount << ")\n"
        << "  --brightness  0-100 percent (default 25)\n"
        << "\n"
        << "Ctrl+C to stop; the strip is blanked on exit.\n";
}

bool ParseState(const std::string& name, LedState* out)
{
    if (name == "idle")    { *out = LedState::IDLE;    return true; }
    if (name == "driving") { *out = LedState::DRIVING; return true; }
    if (name == "manual")  { *out = LedState::MANUAL;  return true; }
    if (name == "estop")   { *out = LedState::ESTOP;   return true; }
    return false;
}

} // namespace

int main(int argc, char** argv)
{
    int led_count = LedStripManager::kDefaultLedCount;
    int brightness = 25;
    bool cycle = true;
    LedState fixed_state = LedState::IDLE;

    for (int i = 1; i < argc; i++) {
        const std::string arg = argv[i];

        if (arg == "-h" || arg == "--help") {
            Usage(argv[0]);
            return 0;
        }
        if (arg == "--leds" && i + 1 < argc) {
            led_count = std::atoi(argv[++i]);
            continue;
        }
        if (arg == "--brightness" && i + 1 < argc) {
            brightness = std::atoi(argv[++i]);
            continue;
        }
        if (ParseState(arg, &fixed_state)) {
            cycle = false;
            continue;
        }

        std::cerr << "unknown argument: " << arg << "\n\n";
        Usage(argv[0]);
        return 1;
    }

    std::signal(SIGINT, OnSignal);
    std::signal(SIGTERM, OnSignal);

    LedStripManager led(led_count);
    led.SetBrightness(brightness);

    if (!led.Open()) {
        std::cerr << "Is the driver loaded, and do you have write permission "
                     "on /dev/led_strip?\n";
        return 1;
    }

    if (!cycle) {
        std::cout << "state = " << LedStripManager::StateName(fixed_state)
                  << " (Ctrl+C to stop)" << std::endl;
        led.SetState(fixed_state);

        while (g_running.load())
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
    } else {
        const LedState sequence[] = {
            LedState::IDLE, LedState::DRIVING, LedState::MANUAL, LedState::ESTOP,
        };
        const int kHoldMs = 6000;
        size_t index = 0;

        std::cout << "cycling states, 6s each (Ctrl+C to stop)" << std::endl;

        while (g_running.load()) {
            const LedState state = sequence[index % 4];
            std::cout << "  -> " << LedStripManager::StateName(state) << std::endl;
            led.SetState(state);

            for (int waited = 0; waited < kHoldMs && g_running.load(); waited += 100)
                std::this_thread::sleep_for(std::chrono::milliseconds(100));

            index++;
        }
    }

    std::cout << "\nblanking strip" << std::endl;
    led.Close();
    return 0;
}
