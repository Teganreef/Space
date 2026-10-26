#pragma once

#include <array>
#include <cstdint>

namespace sfs {

// One tick is one second of simulated time.
inline constexpr double kDt = 1.0;

// Simplified LEO geometry: 90 minute orbit, eclipse for the last ~35%,
// one 10 minute ground pass per orbit.
inline constexpr uint32_t kOrbitTicks = 5400;
inline constexpr uint32_t kEclipseStart = 3500;
inline constexpr uint32_t kPassStart = 1000;
inline constexpr uint32_t kPassEnd = 1600;
inline constexpr uint32_t kMissionTicks = 5 * kOrbitTicks;

constexpr bool in_eclipse(uint32_t tick) { return (tick % kOrbitTicks) >= kEclipseStart; }

constexpr bool in_pass(uint32_t tick) {
    const uint32_t p = tick % kOrbitTicks;
    return p >= kPassStart && p < kPassEnd;
}

enum class Mode : uint8_t { Boot, Nominal, Degraded, Safe, Recovery, Invalid };

const char* to_string(Mode m);

// What the flight computer sees each tick, after any sensor faults.
struct SensorFrame {
    float sun_a = 0.0F;  // sun angle from sensor A [rad]
    float sun_b = 0.0F;  // sun angle from sensor B [rad]
    bool sun_a_valid = false;
    bool sun_b_valid = false;
    float gyro = 0.0F;         // body rate [rad/s]
    float batt_v = 0.0F;       // battery pack voltage [V]
    float net_power_w = 0.0F;  // solar in minus load out [W]
};

// A scheduled payload operation. Layout has no padding so it can live in
// CRC-protected and bit-flip-injected memory without uninitialised bytes.
struct PayloadWindow {
    uint32_t start = 0;
    uint16_t duration = 0;
    uint16_t seq = 0;  // 0 = generated onboard, not from a ground command
    uint8_t value = 0;
    uint8_t flags = 0;
    uint16_t spare = 0;
};

// Fault classes a controller can report. Reported on the tick a detection
// fires, not continuously, so detection latency can be measured per event.
enum DetectBit : uint8_t {
    kDetAttitude = 1U << 0,
    kDetGyro = 1U << 1,
    kDetBattery = 1U << 2,
    kDetPower = 1U << 3,
    kDetComm = 1U << 4,
    kDetMemory = 1U << 5,
};

inline constexpr std::size_t kMaxExecPerTick = 8;

struct ControlOutput {
    Mode mode = Mode::Boot;
    uint8_t mode_raw = 0;
    bool payload_on = false;
    float torque = 0.0F;
    uint8_t detected = 0;
    uint8_t n_executed = 0;
    std::array<uint16_t, kMaxExecPerTick> executed_seq{};
    std::array<uint32_t, kMaxExecPerTick> executed_age{};
};

}  // namespace sfs
