#include "bdio/harness.hpp"

#include <bitset>
#include <cinttypes>
#include <memory>

#include "bdio/hardened_controller.hpp"
#include "bdio/naive_controller.hpp"
#include "bdio/packet.hpp"
#include "bdio/scheduler.hpp"
#include "bdio/spacecraft.hpp"

namespace bdio {

namespace {

constexpr double kInitialSoc = 0.55;
constexpr std::size_t kMaxPending = 64;
constexpr uint32_t kDuplicateLag = 5;
constexpr uint32_t kEpisodeMerge = 60;

struct Pending {
    Frame frame{};
    uint32_t deliver_at = 0;
    bool used = false;
};

// Ground segment: during each pass, uplinks three payload windows, one every
// 60 seconds, picked from the same target catalog the spacecraft carries:
// two later in the current orbit and one early in the next. The choice is
// drawn from its own random stream so both controllers see the same plan.
class Ground {
public:
    explicit Ground(uint64_t seed) : rng_(seed ^ 0x6E0DULL) {}

    bool command_at(uint32_t tick, Command& cmd) {
        if (!in_pass(tick)) {
            return false;
        }
        const uint32_t into_pass = tick % kOrbitTicks - kPassStart;
        if (into_pass % 60 != 30 || into_pass / 60 >= 3) {
            return false;
        }
        const uint32_t orbit = tick / kOrbitTicks;
        const uint32_t idx = into_pass / 60;
        if (idx == 0) {
            pick_targets();
        }
        const Candidate& c = kOnboardTargets[picks_[idx]];
        const uint32_t orbit_start = (idx == 2 ? orbit + 1 : orbit) * kOrbitTicks;
        cmd.seq = ++seq_;
        cmd.send_tick = tick;
        cmd.type = CommandType::AddPayloadWindow;
        cmd.start = orbit_start + c.offset;
        cmd.duration = c.duration;
        cmd.value = c.value;
        return true;
    }

private:
    void pick_targets() {
        // Catalog indices 2..5 are after the pass; choose two that do not
        // overlap. Indices 0..1 are early next orbit; choose one.
        do {
            picks_[0] = static_cast<uint8_t>(rng_.range(2, 5));
            picks_[1] = static_cast<uint8_t>(rng_.range(2, 5));
        } while (picks_[0] == picks_[1] || overlaps(kOnboardTargets[picks_[0]], kOnboardTargets[picks_[1]]));
        picks_[2] = static_cast<uint8_t>(rng_.range(0, 1));
    }

    Rng rng_;
    uint16_t seq_ = 0;
    std::array<uint8_t, 3> picks_{};
};

// Science is credited for payload time inside any catalog target window,
// whether the ground or the onboard scheduler asked for it.
bool in_catalog_window(uint32_t tick) {
    const uint32_t p = tick % kOrbitTicks;
    for (const Candidate& c : kOnboardTargets) {
        if (p >= c.offset && p < c.offset + c.duration) {
            return true;
        }
    }
    return false;
}

}  // namespace

const char* to_string(ControllerKind k) { return k == ControllerKind::Naive ? "naive" : "hardened"; }

const char* violation_name(int v) {
    switch (v) {
        case kDeepDischarge: return "deep_discharge";
        case kPayloadLowPower: return "payload_low_power";
        case kInvalidMode: return "invalid_mode";
        case kPayloadInSafe: return "payload_in_safe";
        case kDuplicateExec: return "duplicate_exec";
        case kStaleExec: return "stale_exec";
        default: return "unknown";
    }
}

RunMetrics run_mission(const RunOptions& opts) {
    RunMetrics m;
    m.scenario = opts.scenario;
    m.seed = opts.seed;
    m.kind = opts.kind;
    m.ticks = opts.ticks;

    Spacecraft sc(SpacecraftParams{}, opts.seed, kInitialSoc);
    FaultInjector inj(opts.scenario, opts.seed, opts.ticks);
    Ground ground(opts.seed);

    std::unique_ptr<Controller> ctrl;
    if (opts.kind == ControllerKind::Naive) {
        ctrl = std::make_unique<NaiveController>();
    } else {
        ctrl = std::make_unique<HardenedController>();
    }

    std::array<Pending, kMaxPending> pending{};
    auto bits = std::make_unique<std::bitset<65536>>();  // executed sequence numbers

    const auto events = inj.events();
    std::array<bool, kMaxEvents> detected{};
    std::array<bool, kViolationCount> prev_violation{};
    std::array<uint32_t, kViolationCount> last_violation{};
    std::array<bool, kViolationCount> ever_violated{};
    Mode prev_mode = Mode::Boot;
    uint32_t safe_since = 0;
    bool in_safe_episode = false;

    if (opts.csv != nullptr) {
        std::fprintf(opts.csv,
                     "tick,mode,payload,soc_true,soc_est,theta_true,theta_est,sun_a,sun_b,sun_a_valid,"
                     "sun_b_valid,batt_v,gyro,detected\n");
    }
    auto log_event = [&](uint32_t tick, const char* type, const char* detail) {
        if (opts.events != nullptr) {
            std::fprintf(opts.events, "%u,%s,%s\n", tick, type, detail);
        }
    };
    if (opts.events != nullptr) {
        std::fprintf(opts.events, "tick,type,detail\n");
    }

    for (uint32_t t = 0; t < opts.ticks; ++t) {
        // Faults that change the physical spacecraft.
        if (const double loss = inj.cell_loss_at(t); loss > 0.0) {
            sc.lose_capacity(loss);
        }
        for (const FaultEvent& e : events) {
            if (e.onset == t && e.kind != FaultKind::Seu) {
                log_event(t, "fault", to_string(e.kind));
            }
        }
        if (inj.apply_seu(t, ctrl->seu_region())) {
            log_event(t, "fault", "seu");
        }

        // Uplink through the faulty channel.
        Command cmd;
        if (ground.command_at(t, cmd)) {
            const uint32_t end = std::min(cmd.start + cmd.duration, opts.ticks);
            if (cmd.start < end) {
                m.planned_science_ticks += end - cmd.start;
            }
            if (!inj.blackout(t)) {
                Frame frame = encode(cmd);
                if (inj.corrupt(t)) {
                    inj.corrupt_frame(frame);
                }
                const uint32_t deliver = t + inj.delay(t);
                const int copies = inj.duplicate(t) ? 2 : 1;
                for (int c = 0; c < copies; ++c) {
                    for (Pending& p : pending) {
                        if (!p.used) {
                            p = {frame, deliver + static_cast<uint32_t>(c) * kDuplicateLag, true};
                            break;
                        }
                    }
                }
            }
        }
        std::array<Frame, kMaxExecPerTick> rx{};
        std::size_t n_rx = 0;
        for (Pending& p : pending) {
            if (p.used && p.deliver_at == t && n_rx < rx.size()) {
                rx[n_rx++] = p.frame;
                p.used = false;
            }
        }

        SensorFrame sensors = sc.sense(t);
        inj.apply_sensor_faults(t, sensors);
        const ControlOutput out = ctrl->step(t, sensors, std::span<const Frame>(rx.data(), n_rx));

        // Invariants, judged against the truth.
        const double soc = sc.soc();
        std::array<bool, kViolationCount> now{};
        now[kDeepDischarge] = soc < 0.10;
        now[kPayloadLowPower] = out.payload_on && soc < 0.20;
        now[kInvalidMode] = out.mode == Mode::Invalid;
        now[kPayloadInSafe] = out.payload_on && out.mode == Mode::Safe;
        for (int v = 0; v < kDuplicateExec; ++v) {
            const bool new_episode = !ever_violated[v] || t - last_violation[v] > kEpisodeMerge;
            if (now[v] && !prev_violation[v] && new_episode) {
                ++m.violations[v];
                log_event(t, "violation", violation_name(v));
            }
            if (now[v]) {
                ever_violated[v] = true;
                last_violation[v] = t;
            }
            prev_violation[v] = now[v];
        }
        for (std::size_t i = 0; i < out.n_executed; ++i) {
            const uint16_t seq = out.executed_seq[i];
            if (seq != 0 && bits->test(seq)) {
                ++m.violations[kDuplicateExec];
                log_event(t, "violation", violation_name(kDuplicateExec));
            }
            bits->set(seq);
            if (out.executed_age[i] > kMaxCommandAge) {
                ++m.violations[kStaleExec];
                log_event(t, "violation", violation_name(kStaleExec));
            }
        }

        // Detection latency, per scored fault event.
        for (std::size_t i = 0; i < events.size(); ++i) {
            const FaultEvent& e = events[i];
            const uint8_t cls = detect_class(e.kind);
            if (cls == 0 || detected[i] || t < e.onset) {
                continue;
            }
            if ((out.detected & cls) != 0) {
                detected[i] = true;
                const uint32_t latency = t - e.onset;
                ++m.faults_detected;
                m.latency_sum += latency;
                auto& k = m.by_kind[static_cast<std::size_t>(e.kind)];
                ++k[1];
                k[2] += latency;
                m.latency_max = std::max(m.latency_max, latency);
                log_event(t, "detected", to_string(e.kind));
            }
        }

        // Safe mode bookkeeping.
        if (out.mode == Mode::Safe && prev_mode != Mode::Safe) {
            ++m.safe_entries;
            const bool justified = soc < 0.30 || inj.any_fault_near(t, 300);
            if (!justified) {
                ++m.false_safe_entries;
            }
            log_event(t, "safe_entry", justified ? "justified" : "unjustified");
            if (!in_safe_episode) {
                in_safe_episode = true;
                safe_since = t;
            }
        }
        if (in_safe_episode && (out.mode == Mode::Nominal || out.mode == Mode::Degraded)) {
            in_safe_episode = false;
            ++m.recoveries;
            m.recovery_ticks_sum += t - safe_since;
        }
        if (out.mode != prev_mode) {
            log_event(t, "mode", to_string(out.mode));
        }
        prev_mode = out.mode;

        // Science: payload running inside a target window.
        if (out.payload_on && in_catalog_window(t)) {
            ++m.science_ticks;
        }
        ++m.mode_ticks[static_cast<std::size_t>(out.mode)];
        m.min_soc = std::min(m.min_soc, soc);

        if (opts.csv != nullptr) {
            const Truth& tr = sc.truth();
            std::fprintf(opts.csv, "%u,%s,%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%d,%d,%.3f,%.6f,%u\n", t,
                         to_string(out.mode), out.payload_on ? 1 : 0, soc, ctrl->soc_estimate(), tr.theta,
                         ctrl->theta_estimate(), sensors.sun_a, sensors.sun_b, sensors.sun_a_valid ? 1 : 0,
                         sensors.sun_b_valid ? 1 : 0, sensors.batt_v, sensors.gyro, out.detected);
        }

        sc.step(t, out);
    }

    for (std::size_t i = 0; i < events.size(); ++i) {
        if (detect_class(events[i].kind) != 0) {
            ++m.faults_scored;
            ++m.by_kind[static_cast<std::size_t>(events[i].kind)][0];
        }
    }
    for (const uint32_t v : m.violations) {
        m.total_violations += v;
    }
    m.final_soc = sc.soc();
    return m;
}

std::string to_json(const RunMetrics& m) {
    char buf[1024];
    std::string s;
    std::snprintf(buf, sizeof(buf),
                  "{\"scenario\":\"%s\",\"seed\":%" PRIu64 ",\"controller\":\"%s\",\"ticks\":%u,"
                  "\"total_violations\":%u,\"violations\":{",
                  to_string(m.scenario), m.seed, to_string(m.kind), m.ticks, m.total_violations);
    s += buf;
    for (int v = 0; v < kViolationCount; ++v) {
        std::snprintf(buf, sizeof(buf), "%s\"%s\":%u", v == 0 ? "" : ",", violation_name(v), m.violations[v]);
        s += buf;
    }
    std::snprintf(buf, sizeof(buf),
                  "},\"faults_scored\":%u,\"faults_detected\":%u,\"latency_sum\":%" PRIu64
                  ",\"latency_max\":%u,\"safe_entries\":%u,\"false_safe_entries\":%u,\"recoveries\":%u,"
                  "\"recovery_ticks_sum\":%" PRIu64
                  ",\"planned_science_ticks\":%u,\"science_ticks\":%u,\"min_soc\":%.4f,\"final_soc\":%.4f,"
                  "\"mode_ticks\":{",
                  m.faults_scored, m.faults_detected, m.latency_sum, m.latency_max, m.safe_entries,
                  m.false_safe_entries, m.recoveries, m.recovery_ticks_sum, m.planned_science_ticks, m.science_ticks,
                  m.min_soc, m.final_soc);
    s += buf;
    for (std::size_t i = 0; i < m.mode_ticks.size(); ++i) {
        std::snprintf(buf, sizeof(buf), "%s\"%s\":%u", i == 0 ? "" : ",", to_string(static_cast<Mode>(i)),
                      m.mode_ticks[i]);
        s += buf;
    }
    s += "},\"by_kind\":{";
    bool first = true;
    for (std::size_t k = 0; k < m.by_kind.size(); ++k) {
        if (m.by_kind[k][0] == 0) {
            continue;
        }
        std::snprintf(buf, sizeof(buf), "%s\"%s\":[%u,%u,%u]", first ? "" : ",",
                      to_string(static_cast<FaultKind>(k)), m.by_kind[k][0], m.by_kind[k][1], m.by_kind[k][2]);
        s += buf;
        first = false;
    }
    s += "}}";
    return s;
}

const char* to_string(Mode m) {
    switch (m) {
        case Mode::Boot: return "boot";
        case Mode::Nominal: return "nominal";
        case Mode::Degraded: return "degraded";
        case Mode::Safe: return "safe";
        case Mode::Recovery: return "recovery";
        case Mode::Invalid: return "invalid";
    }
    return "unknown";
}

}  // namespace bdio
