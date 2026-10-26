#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

#include "bdio/packet.hpp"
#include "bdio/rng.hpp"
#include "bdio/types.hpp"

namespace bdio {

enum class FaultKind : uint8_t {
    SunBias,        // sensor reports a plausible but offset angle
    SunStuck,       // sensor freezes on its last value and still claims valid
    SunNoise,       // sensor noise grows by an order of magnitude
    GyroBiasJump,   // gyro bias steps to a new value
    BattBias,       // battery voltage sensor offset
    BattStuck,      // battery voltage sensor freezes
    CellLoss,       // real loss of battery capacity and charge
    CommBlackout,   // no uplink frames arrive
    CommDuplicate,  // frames delivered twice
    CommDelay,      // frames delivered late
    CommCorrupt,    // bits flipped in uplink frames
    Seu,            // single event upset in flight computer memory
    Count,
};

const char* to_string(FaultKind k);

// Which detection bits a controller may raise to be credited with detecting
// a fault kind.
// Zero means the fault is not scored for detection latency (a blackout is
// silence, not something with an onset a controller can observe directly).
uint8_t detect_class(FaultKind k);

enum class Scenario : uint8_t { Nominal, Sensor, Power, Comms, Seu, Combined, SeuStorm, Count };

const char* to_string(Scenario s);
bool parse_scenario(std::string_view name, Scenario& out);

struct FaultEvent {
    FaultKind kind = FaultKind::Count;
    uint8_t target = 0;     // which sun sensor for sun faults
    uint32_t onset = 0;
    uint32_t duration = 0;  // 0 = permanent
    float magnitude = 0.0F;
};

inline constexpr std::size_t kMaxEvents = 4096;

// Generates a reproducible fault timeline from (scenario, seed) and applies
// it during the run. All randomness after construction comes from a separate
// stream so that adding a query never changes the generated timeline.
class FaultInjector {
public:
    FaultInjector(Scenario scenario, uint64_t seed, uint32_t mission_ticks);

    std::span<const FaultEvent> events() const { return {events_.data(), n_events_}; }
    static bool active(const FaultEvent& e, uint32_t tick) {
        return tick >= e.onset && (e.duration == 0 || tick < e.onset + e.duration);
    }

    // Sensor faults plus rare single-sample glitches that are present in every
    // scenario, including Nominal. Glitches are not fault events: a controller
    // that drops into safe mode for one bad sample is generating false alarms.
    void apply_sensor_faults(uint32_t tick, SensorFrame& frame);

    // Fraction of capacity lost if a cell fails on this tick, else 0.
    double cell_loss_at(uint32_t tick) const;

    bool blackout(uint32_t tick) const { return any_active(FaultKind::CommBlackout, tick); }
    bool duplicate(uint32_t tick) const { return any_active(FaultKind::CommDuplicate, tick); }
    uint32_t delay(uint32_t tick) const;
    bool corrupt(uint32_t tick) const { return any_active(FaultKind::CommCorrupt, tick); }
    void corrupt_frame(Frame& frame);

    // Flips one random bit in `region` if an upset is scheduled for this tick.
    bool apply_seu(uint32_t tick, std::span<std::byte> region);

    // True if any fault started within `window` ticks before `tick` or is
    // active now. Used to decide whether a safe-mode entry was justified.
    bool any_fault_near(uint32_t tick, uint32_t window) const;

private:
    void add(const FaultEvent& e);
    bool any_active(FaultKind k, uint32_t tick) const;
    uint32_t random_onset(uint32_t lo, uint32_t hi);
    FaultEvent random_sensor_fault(uint32_t onset);
    FaultEvent random_power_fault(uint32_t onset);
    void add_seus(double rate_per_tick, uint32_t mission_ticks);

    void index_events();

    std::array<FaultEvent, kMaxEvents> events_{};
    std::size_t n_events_ = 0;
    // Per-tick queries only walk the events that can matter: the handful of
    // non-upset faults, and upsets via a cursor over their sorted onsets.
    std::array<uint16_t, kMaxEvents> fault_idx_{};
    std::size_t n_faults_ = 0;
    std::array<uint32_t, kMaxEvents> seu_onsets_{};
    std::size_t n_seu_ = 0;
    std::size_t seu_cursor_ = 0;
    std::array<float, kMaxEvents> latched_{};
    std::array<bool, kMaxEvents> is_latched_{};
    Rng gen_;      // builds the timeline
    Rng runtime_;  // glitches, noise, bit positions
};

}  // namespace bdio
