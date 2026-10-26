#include "bdio/faults.hpp"

#include <algorithm>
#include <cmath>

namespace bdio {

namespace {
constexpr double kSunGlitchProb = 1.0 / 1500.0;
constexpr double kBattGlitchProb = 1.0 / 3000.0;
}  // namespace

const char* to_string(FaultKind k) {
    switch (k) {
        case FaultKind::SunBias: return "sun_bias";
        case FaultKind::SunStuck: return "sun_stuck";
        case FaultKind::SunNoise: return "sun_noise";
        case FaultKind::GyroBiasJump: return "gyro_bias_jump";
        case FaultKind::BattBias: return "batt_bias";
        case FaultKind::BattStuck: return "batt_stuck";
        case FaultKind::CellLoss: return "cell_loss";
        case FaultKind::CommBlackout: return "comm_blackout";
        case FaultKind::CommDuplicate: return "comm_duplicate";
        case FaultKind::CommDelay: return "comm_delay";
        case FaultKind::CommCorrupt: return "comm_corrupt";
        case FaultKind::Seu: return "seu";
        case FaultKind::Count: break;
    }
    return "unknown";
}

uint8_t detect_class(FaultKind k) {
    switch (k) {
        case FaultKind::SunBias:
        case FaultKind::SunStuck:
        case FaultKind::SunNoise: return kDetAttitude;
        case FaultKind::GyroBiasJump: return kDetGyro;
        // From inside the spacecraft a low-reading voltage sensor and a real
        // loss of charge look the same, so either report counts.
        case FaultKind::BattBias:
        case FaultKind::BattStuck:
        case FaultKind::CellLoss: return kDetBattery | kDetPower;
        case FaultKind::Seu: return kDetMemory;
        default: return 0;
    }
}

const char* to_string(Scenario s) {
    switch (s) {
        case Scenario::Nominal: return "nominal";
        case Scenario::Sensor: return "sensor";
        case Scenario::Power: return "power";
        case Scenario::Comms: return "comms";
        case Scenario::Seu: return "seu";
        case Scenario::Combined: return "combined";
        case Scenario::SeuStorm: return "seu_storm";
        case Scenario::Count: break;
    }
    return "unknown";
}

bool parse_scenario(std::string_view name, Scenario& out) {
    for (uint8_t i = 0; i < static_cast<uint8_t>(Scenario::Count); ++i) {
        const auto s = static_cast<Scenario>(i);
        if (name == to_string(s)) {
            out = s;
            return true;
        }
    }
    return false;
}

FaultInjector::FaultInjector(Scenario scenario, uint64_t seed, uint32_t mission_ticks)
    : gen_(seed ^ 0xFA017FA017ULL), runtime_(seed ^ 0x5EED5EEDULL) {
    const uint32_t lo = 600;
    const uint32_t hi = mission_ticks > 3000 ? mission_ticks - 2000 : lo + 1;

    switch (scenario) {
        case Scenario::Nominal:
            break;
        case Scenario::Sensor: {
            const uint32_t n = gen_.range(1, 2);
            for (uint32_t i = 0; i < n; ++i) {
                add(random_sensor_fault(random_onset(lo, hi)));
            }
            break;
        }
        case Scenario::Power:
            add(random_power_fault(random_onset(lo, hi)));
            if (gen_.chance(0.6)) {
                add({FaultKind::CellLoss, 0, random_onset(lo, hi), 0, 0.3F});
            }
            break;
        case Scenario::Comms: {
            add({FaultKind::CommBlackout, 0, random_onset(lo, hi / 2), gen_.range(6000, 11000), 0.0F});
            if (gen_.chance(0.7)) {
                add({FaultKind::CommDuplicate, 0, random_onset(lo, hi), gen_.range(3000, 6000), 0.0F});
            }
            if (gen_.chance(0.7)) {
                add({FaultKind::CommDelay, 0, random_onset(lo, hi), gen_.range(3000, 6000),
                     static_cast<float>(gen_.range(120, 900))});
            }
            if (gen_.chance(0.7)) {
                add({FaultKind::CommCorrupt, 0, random_onset(lo, hi), gen_.range(3000, 6000), 0.0F});
            }
            break;
        }
        case Scenario::Seu:
            add_seus(1.0 / 400.0, mission_ticks);
            break;
        case Scenario::Combined: {
            // The interesting case: several failures landing close together,
            // while the ground cannot help.
            const uint32_t t0 = random_onset(lo, hi / 2);
            add({FaultKind::CommBlackout, 0, t0, gen_.range(6000, 11000), 0.0F});
            add(random_sensor_fault(t0 + gen_.range(0, 900)));
            add(random_power_fault(t0 + gen_.range(0, 900)));
            if (gen_.chance(0.5)) {
                add({FaultKind::CommCorrupt, 0, random_onset(lo, hi), gen_.range(3000, 6000), 0.0F});
            }
            add_seus(1.0 / 1500.0, mission_ticks);
            break;
        }
        case Scenario::SeuStorm:
            // Far beyond a realistic LEO upset rate. Exists to find where
            // per-cycle TMR scrubbing stops being enough.
            add_seus(1.0 / 10.0, mission_ticks);
            break;
        case Scenario::Count:
            break;
    }
}

void FaultInjector::add(const FaultEvent& e) {
    if (n_events_ < kMaxEvents) {
        events_[n_events_++] = e;
    }
}

uint32_t FaultInjector::random_onset(uint32_t lo, uint32_t hi) { return gen_.range(lo, std::max(lo, hi)); }

FaultEvent FaultInjector::random_sensor_fault(uint32_t onset) {
    FaultEvent e;
    e.onset = onset;
    e.target = static_cast<uint8_t>(gen_.range(0, 1));
    e.duration = gen_.chance(0.7) ? 0 : gen_.range(1200, 3000);
    switch (gen_.range(0, 3)) {
        case 0:
            e.kind = FaultKind::SunBias;
            e.magnitude = static_cast<float>(gen_.uniform(0.25, 0.5) * (gen_.chance(0.5) ? 1.0 : -1.0));
            break;
        case 1:
            e.kind = FaultKind::SunStuck;
            break;
        case 2:
            e.kind = FaultKind::SunNoise;
            e.magnitude = 0.3F;
            e.duration = gen_.range(300, 900);
            break;
        default:
            e.kind = FaultKind::GyroBiasJump;
            e.magnitude = static_cast<float>(gen_.uniform(0.002, 0.004) * (gen_.chance(0.5) ? 1.0 : -1.0));
            e.duration = 0;
            break;
    }
    return e;
}

FaultEvent FaultInjector::random_power_fault(uint32_t onset) {
    FaultEvent e;
    e.onset = onset;
    e.duration = 0;
    if (gen_.chance(0.5)) {
        e.kind = FaultKind::BattBias;
        // Reading high is the dangerous direction: it hides a draining battery.
        e.magnitude = gen_.chance(0.8) ? static_cast<float>(gen_.uniform(0.5, 0.8))
                                       : static_cast<float>(-gen_.uniform(0.4, 0.6));
    } else {
        e.kind = FaultKind::BattStuck;
    }
    return e;
}

void FaultInjector::add_seus(double rate_per_tick, uint32_t mission_ticks) {
    // Poisson arrivals via exponential inter-arrival times.
    double t = 0.0;
    while (true) {
        t += -std::log(1.0 - gen_.uniform()) / rate_per_tick;
        if (t >= static_cast<double>(mission_ticks)) {
            break;
        }
        add({FaultKind::Seu, 0, static_cast<uint32_t>(t), 1, 0.0F});
    }
}

bool FaultInjector::any_active(FaultKind k, uint32_t tick) const {
    for (std::size_t i = 0; i < n_events_; ++i) {
        if (events_[i].kind == k && active(events_[i], tick)) {
            return true;
        }
    }
    return false;
}

void FaultInjector::apply_sensor_faults(uint32_t tick, SensorFrame& f) {
    for (std::size_t i = 0; i < n_events_; ++i) {
        const FaultEvent& e = events_[i];
        if (!active(e, tick)) {
            is_latched_[i] = false;
            continue;
        }
        float* sun = e.target == 0 ? &f.sun_a : &f.sun_b;
        bool* sun_valid = e.target == 0 ? &f.sun_a_valid : &f.sun_b_valid;
        switch (e.kind) {
            case FaultKind::SunBias:
                if (*sun_valid) {
                    *sun += e.magnitude;
                }
                break;
            case FaultKind::SunStuck:
                if (!is_latched_[i]) {
                    latched_[i] = *sun_valid ? *sun : 0.0F;
                    is_latched_[i] = true;
                }
                *sun = latched_[i];
                *sun_valid = true;
                break;
            case FaultKind::SunNoise:
                if (*sun_valid) {
                    *sun += static_cast<float>(runtime_.normal(e.magnitude));
                }
                break;
            case FaultKind::GyroBiasJump:
                f.gyro += e.magnitude;
                break;
            case FaultKind::BattBias:
                f.batt_v += e.magnitude;
                break;
            case FaultKind::BattStuck:
                if (!is_latched_[i]) {
                    latched_[i] = f.batt_v;
                    is_latched_[i] = true;
                }
                f.batt_v = latched_[i];
                break;
            default:
                break;
        }
    }

    // Transient single-sample glitches, present in every scenario.
    if (f.sun_a_valid && runtime_.chance(kSunGlitchProb)) {
        f.sun_a += static_cast<float>(runtime_.uniform(0.5, 1.4) * (runtime_.chance(0.5) ? 1.0 : -1.0));
    }
    if (f.sun_b_valid && runtime_.chance(kSunGlitchProb)) {
        f.sun_b += static_cast<float>(runtime_.uniform(0.5, 1.4) * (runtime_.chance(0.5) ? 1.0 : -1.0));
    }
    if (runtime_.chance(kBattGlitchProb)) {
        f.batt_v += static_cast<float>(runtime_.uniform(0.9, 1.4) * (runtime_.chance(0.5) ? 1.0 : -1.0));
    }
}

double FaultInjector::cell_loss_at(uint32_t tick) const {
    for (std::size_t i = 0; i < n_events_; ++i) {
        if (events_[i].kind == FaultKind::CellLoss && events_[i].onset == tick) {
            return events_[i].magnitude;
        }
    }
    return 0.0;
}

uint32_t FaultInjector::delay(uint32_t tick) const {
    for (std::size_t i = 0; i < n_events_; ++i) {
        if (events_[i].kind == FaultKind::CommDelay && active(events_[i], tick)) {
            return static_cast<uint32_t>(events_[i].magnitude);
        }
    }
    return 0;
}

void FaultInjector::corrupt_frame(Frame& frame) {
    const uint32_t flips = runtime_.range(1, 3);
    for (uint32_t i = 0; i < flips; ++i) {
        const uint32_t bit = runtime_.range(0, kFrameSize * 8 - 1);
        frame[bit / 8] = static_cast<uint8_t>(frame[bit / 8] ^ (1U << (bit % 8)));
    }
}

bool FaultInjector::apply_seu(uint32_t tick, std::span<std::byte> region) {
    if (region.empty()) {
        return false;
    }
    bool flipped = false;
    for (std::size_t i = 0; i < n_events_; ++i) {
        if (events_[i].kind == FaultKind::Seu && events_[i].onset == tick) {
            const auto bit = static_cast<std::size_t>(runtime_.range(0, static_cast<uint32_t>(region.size() * 8 - 1)));
            region[bit / 8] ^= static_cast<std::byte>(1U << (bit % 8));
            flipped = true;
        }
    }
    return flipped;
}

bool FaultInjector::any_fault_near(uint32_t tick, uint32_t window) const {
    for (std::size_t i = 0; i < n_events_; ++i) {
        const FaultEvent& e = events_[i];
        const bool recent = e.onset <= tick && tick - e.onset <= window;
        if (recent || active(e, tick)) {
            return true;
        }
    }
    return false;
}

}  // namespace bdio
