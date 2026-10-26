#include "bdio/hardened_controller.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

#include "bdio/crc16.hpp"
#include "bdio/scheduler.hpp"
#include "bdio/spacecraft.hpp"

namespace bdio {

namespace {

constexpr float kNominalCapacityJ = 144000.0F;
constexpr double kPayloadW = 8.0;
constexpr float kPi = 3.14159265F;

// Attitude estimation.
constexpr float kSunAgree = 0.08F;        // two sensors this close are consistent
constexpr float kSunSingleGate = 0.15F;   // lone sensor accepted within this of prediction
constexpr uint16_t kSunOutlierPersist = 5;
constexpr uint16_t kSunFlatPersist = 10;  // identical samples in a row means frozen
constexpr uint16_t kSunRehab = 300;
constexpr float kAttGain = 0.2F;
constexpr float kBiasGain = 0.002F;
constexpr float kGyroBiasLimit = 0.0012F;
constexpr float kBiasLearnGate = 0.05F;

// Battery estimation.
constexpr float kBattDisagree = 0.12F;
constexpr uint16_t kBattHighPersist = 30;  // claims of extra energy need to persist
constexpr uint16_t kBattLowPersist = 10;   // claims of missing energy are believed sooner
constexpr uint16_t kBattFlatPersist = 10;
constexpr uint16_t kBattRehab = 600;

// Mode logic.
constexpr float kSafeEnter = 0.25F;
constexpr uint16_t kSafePersist = 10;
constexpr float kSafeExit = 0.45F;
constexpr uint16_t kSafeExitPersist = 120;
constexpr uint32_t kRecoveryHold = 300;
constexpr uint16_t kDegradedClear = 60;
constexpr uint32_t kCommLossTicks = 8100;  // 1.5 orbits without a valid frame

bool plausible_batt(float v) { return v >= 5.9F && v <= 8.5F; }

float wrap(float a) { return std::remainder(a, 2.0F * kPi); }

}  // namespace

Mode decode_hardened_mode(uint8_t raw) {
    switch (raw) {
        case kModeBoot: return Mode::Boot;
        case kModeNominal: return Mode::Nominal;
        case kModeDegraded: return Mode::Degraded;
        case kModeSafe: return Mode::Safe;
        case kModeRecovery: return Mode::Recovery;
        default: return Mode::Invalid;
    }
}

HardenedController::HardenedController() {
    crit_.mode.set(kModeBoot);
    crit_.payload_on.set(0);
    crit_.last_seq.set(0);
    commit_plan();
}

std::span<std::byte> HardenedController::seu_region() {
    return {reinterpret_cast<std::byte*>(&crit_), sizeof(crit_)};
}

uint16_t HardenedController::table_crc(const PlanTable& t) {
    const auto* base = reinterpret_cast<const std::byte*>(&t);
    uint16_t crc = crc16(std::span<const std::byte>(base, offsetof(PlanTable, crc)));
    return crc16(std::span<const std::byte>(base + offsetof(PlanTable, windows), sizeof(t.windows)), crc);
}

bool HardenedController::table_ok(const PlanTable& t) { return t.n <= t.windows.size() && table_crc(t) == t.crc; }

// Every edit goes to the primary, is sealed, then mirrored to the backup.
void HardenedController::commit_plan() {
    crit_.plan.crc = table_crc(crit_.plan);
    crit_.plan_backup = crit_.plan;
}

void HardenedController::set_mode(uint8_t code) {
    crit_.mode.set(code);
    mode_ticks_ = 0;
}

void HardenedController::add_window(const PayloadWindow& w) {
    PlanTable& t = crit_.plan;
    if (t.n < t.windows.size()) {
        t.windows[t.n++] = w;
    } else {
        std::size_t oldest = 0;
        for (std::size_t i = 1; i < t.windows.size(); ++i) {
            if (t.windows[i].start + t.windows[i].duration <
                t.windows[oldest].start + t.windows[oldest].duration) {
                oldest = i;
            }
        }
        t.windows[oldest] = w;
    }
    commit_plan();
}

bool HardenedController::window_active(uint32_t tick) const {
    for (std::size_t i = 0; i < crit_.plan.n; ++i) {
        const PayloadWindow& w = crit_.plan.windows[i];
        if (tick >= w.start && tick < w.start + w.duration) {
            return true;
        }
    }
    return false;
}

bool HardenedController::future_window_in_orbit(uint32_t tick) const {
    const uint32_t orbit_end = (tick / kOrbitTicks + 1) * kOrbitTicks;
    for (std::size_t i = 0; i < crit_.plan.n; ++i) {
        const PayloadWindow& w = crit_.plan.windows[i];
        if (w.start + w.duration > tick && w.start < orbit_end) {
            return true;
        }
    }
    return false;
}

void HardenedController::check_memory(ControlOutput& out) {
    bool fixed_mode = false;
    bool fixed_payload = false;
    bool fixed_seq = false;
    const uint8_t mode = crit_.mode.vote(&fixed_mode);
    crit_.payload_on.vote(&fixed_payload);
    crit_.last_seq.vote(&fixed_seq);
    if (fixed_mode || fixed_payload || fixed_seq) {
        out.detected |= kDetMemory;
    }
    if (decode_hardened_mode(mode) == Mode::Invalid) {
        // Two copies hit before a scrub. Do not guess: go safe.
        set_mode(kModeSafe);
        out.detected |= kDetMemory;
    }
    const bool primary_ok = table_ok(crit_.plan);
    const bool backup_ok = table_ok(crit_.plan_backup);
    if (primary_ok && backup_ok) {
        return;
    }
    out.detected |= kDetMemory;
    if (primary_ok) {
        crit_.plan_backup = crit_.plan;
    } else if (backup_ok) {
        crit_.plan = crit_.plan_backup;
    } else {
        // Both copies damaged in the same cycle. Dropping the plan costs
        // science; executing a corrupted one could cost the spacecraft.
        crit_.plan = PlanTable{};
        commit_plan();
    }
}

void HardenedController::handle_uplink(uint32_t tick, std::span<const Frame> rx, ControlOutput& out) {
    for (const Frame& frame : rx) {
        Command cmd;
        if (decode(frame, cmd) != DecodeStatus::Ok) {
            out.detected |= kDetComm;
            continue;
        }
        if (cmd.seq <= crit_.last_seq.vote()) {
            out.detected |= kDetComm;  // replay or duplicate
            continue;
        }
        if (cmd.send_tick > tick || tick - cmd.send_tick > kMaxCommandAge) {
            out.detected |= kDetComm;  // stale or from the future
            continue;
        }
        if (cmd.duration == 0 || cmd.duration > 3600) {
            out.detected |= kDetComm;
            continue;
        }
        crit_.last_seq.set(cmd.seq);
        last_rx_tick_ = tick;
        if (cmd.start + cmd.duration > tick) {
            add_window({cmd.start, cmd.duration, cmd.seq, cmd.value, 0, 0});
        }
        if (out.n_executed < kMaxExecPerTick) {
            out.executed_seq[out.n_executed] = cmd.seq;
            out.executed_age[out.n_executed] = tick - cmd.send_tick;
            ++out.n_executed;
        }
    }
}

void HardenedController::estimate_attitude(const SensorFrame& s, ControlOutput& out) {
    const float omega = s.gyro - bias_est_;
    const float prop = wrap(theta_est_ + omega * static_cast<float>(kDt));

    const std::array<float, 2> x{s.sun_a, s.sun_b};
    const std::array<bool, 2> valid{s.sun_a_valid, s.sun_b_valid};

    // Frozen-sensor check. Real sensors are noisy; bit-identical consecutive
    // samples mean the value is not being updated.
    for (std::size_t i = 0; i < 2; ++i) {
        if (!valid[i]) {
            sun_flat_count_[i] = 0;
            continue;
        }
        sun_flat_count_[i] = (x[i] == sun_prev_[i]) ? static_cast<uint16_t>(sun_flat_count_[i] + 1) : 0;
        sun_prev_[i] = x[i];
        if (sun_flat_count_[i] >= kSunFlatPersist && !sun_failed_[i]) {
            sun_failed_[i] = true;
            sun_good_count_[i] = 0;
            out.detected |= kDetAttitude;
        }
    }

    // Rehabilitation of a failed sensor that has been behaving again.
    for (std::size_t i = 0; i < 2; ++i) {
        if (!sun_failed_[i] || !valid[i]) {
            continue;
        }
        const bool good = std::abs(x[i] - prop) < 0.05F && sun_flat_count_[i] == 0;
        sun_good_count_[i] = good ? static_cast<uint16_t>(sun_good_count_[i] + 1) : 0;
        if (sun_good_count_[i] >= kSunRehab) {
            sun_failed_[i] = false;
            sun_outlier_count_[i] = 0;
        }
    }

    const bool use_a = valid[0] && !sun_failed_[0];
    const bool use_b = valid[1] && !sun_failed_[1];
    bool have = false;
    bool sensors_agree = false;
    float meas = 0.0F;

    if (use_a && use_b) {
        if (std::abs(x[0] - x[1]) < kSunAgree) {
            meas = 0.5F * (x[0] + x[1]);
            have = true;
            sensors_agree = true;
            sun_outlier_count_ = {0, 0};
        } else {
            const std::size_t near = std::abs(x[0] - prop) <= std::abs(x[1] - prop) ? 0 : 1;
            const std::size_t far = 1 - near;
            if (std::abs(x[near] - prop) < kSunAgree) {
                meas = x[near];
                have = true;
                sun_outlier_count_[near] = 0;
                if (++sun_outlier_count_[far] >= kSunOutlierPersist) {
                    sun_failed_[far] = true;
                    sun_good_count_[far] = 0;
                    out.detected |= kDetAttitude;
                }
            }
            // Neither sensor matches the prediction: skip the update rather
            // than pick a side on a single sample.
        }
    } else if (use_a || use_b) {
        const std::size_t i = use_a ? 0 : 1;
        if (std::abs(x[i] - prop) < kSunSingleGate) {
            meas = x[i];
            have = true;
            single_disagree_count_ = 0;
        } else if (++single_disagree_count_ >= 30) {
            // One sensor left and the gyro-propagated estimate has drifted
            // away from it. The gyro integrates error; the sensor does not.
            meas = x[i];
            theta_est_ = x[i];
            single_disagree_count_ = 0;
            return;
        }
    }

    if (have && !att_initialized_) {
        // First trusted measurement: take it directly. Filtering from an
        // arbitrary initial guess produces a large innovation that the bias
        // estimator would misread as a gyro fault.
        if (sensors_agree) {
            theta_est_ = meas;
            att_initialized_ = true;
        }
        return;
    }

    if (have) {
        const float innov = wrap(meas - prop);
        theta_est_ = wrap(prop + kAttGain * innov);
        // Only learn bias while tracking. A large innovation means something
        // else is going on (slew, recovering a sensor) and is not bias. With
        // one sensor left this still works for gyro bias: a constant sensor
        // offset is absorbed by the angle estimate, a rate error is not.
        if (std::abs(innov) < kBiasLearnGate) {
            bias_est_ -= kBiasGain * innov;
        }
    } else {
        theta_est_ = prop;
    }

    if (!gyro_flagged_ && std::abs(bias_est_) > kGyroBiasLimit) {
        gyro_flagged_ = true;
        out.detected |= kDetGyro;
    }
}

void HardenedController::estimate_battery(const SensorFrame& s, ControlOutput& out) {
    const bool plausible = plausible_batt(s.batt_v);
    if (plausible) {
        soc_v_ = (s.batt_v - static_cast<float>(kBattVEmpty)) / static_cast<float>(kBattVSpan);
    }

    if (boot_samples_ < kBootTicks) {
        // Seed the coulomb counter from the average boot voltage.
        soc_cc_ = boot_samples_ == 0 ? soc_v_ : soc_cc_ + (soc_v_ - soc_cc_) / static_cast<float>(boot_samples_ + 1);
        ++boot_samples_;
        soc_est_ = soc_cc_;
        last_batt_v_ = s.batt_v;
        return;
    }

    soc_cc_ += s.net_power_w * static_cast<float>(kDt) / kNominalCapacityJ;
    soc_cc_ = std::clamp(soc_cc_, 0.0F, 1.0F);

    batt_flat_count_ = (s.batt_v == last_batt_v_) ? static_cast<uint16_t>(batt_flat_count_ + 1) : 0;
    last_batt_v_ = s.batt_v;
    if (batt_flat_count_ >= kBattFlatPersist && !batt_suspect_) {
        batt_suspect_ = true;
        batt_good_count_ = 0;
        batt_offset_ = 0.0F;
        out.detected |= kDetBattery;
    }

    const float diff = soc_v_ - soc_cc_;
    if (!batt_suspect_) {
        batt_high_count_ = diff > kBattDisagree ? static_cast<uint16_t>(batt_high_count_ + 1) : 0;
        batt_low_count_ = diff < -kBattDisagree ? static_cast<uint16_t>(batt_low_count_ + 1) : 0;
        if (batt_high_count_ >= kBattHighPersist) {
            batt_suspect_ = true;
            batt_good_count_ = 0;
            batt_offset_ = diff;
            out.detected |= kDetBattery;
        } else if (batt_low_count_ >= kBattLowPersist) {
            // Less energy than we thought. Believe it and re-anchor.
            soc_cc_ = soc_v_;
            batt_low_count_ = 0;
            out.detected |= kDetPower;
        } else if (std::abs(diff) < 0.05F) {
            // Slowly bleed off integration drift from current-sensor noise.
            soc_cc_ += 0.002F * diff;
        }
        soc_est_ = plausible ? soc_v_ : soc_cc_;
    } else {
        // Treat a biased sensor as offset rather than useless: track its
        // offset against the coulomb counter while the two move together.
        // A real loss of stored energy still shows up as a step in the
        // corrected reading, which a constant sensor offset cannot produce.
        const bool live = plausible && batt_flat_count_ == 0;
        const float corrected = soc_v_ - batt_offset_;
        const float resid = corrected - soc_cc_;
        if (live && std::abs(resid) < 0.05F) {
            batt_offset_ += 0.01F * resid;
        }
        batt_low_count_ = (live && resid < -kBattDisagree) ? static_cast<uint16_t>(batt_low_count_ + 1) : 0;
        if (batt_low_count_ >= kBattLowPersist) {
            soc_cc_ = corrected;
            batt_low_count_ = 0;
            out.detected |= kDetPower;
        }
        const bool good = live && std::abs(batt_offset_) < 0.03F && std::abs(diff) < 0.03F;
        batt_good_count_ = good ? static_cast<uint16_t>(batt_good_count_ + 1) : 0;
        if (batt_good_count_ >= kBattRehab) {
            batt_suspect_ = false;
            batt_offset_ = 0.0F;
        }
        soc_est_ = live ? std::min(corrected, soc_cc_) : soc_cc_;
    }
}

void HardenedController::update_mode(uint32_t tick) {
    comm_lost_ = tick > kCommLossTicks && tick - last_rx_tick_ > kCommLossTicks;

    low_soc_count_ = soc_est_ < kSafeEnter ? static_cast<uint16_t>(low_soc_count_ + 1) : 0;
    const bool low = low_soc_count_ >= kSafePersist;
    const bool att_ok = !(sun_failed_[0] && sun_failed_[1]);
    const bool reduced = sun_failed_[0] || sun_failed_[1] || gyro_flagged_ || batt_suspect_ || comm_lost_;
    clear_count_ = reduced ? 0 : static_cast<uint16_t>(std::min<int>(clear_count_ + 1, 60000));

    ++mode_ticks_;
    switch (decode_hardened_mode(crit_.mode.vote())) {
        case Mode::Boot:
            if (mode_ticks_ >= kBootTicks) {
                set_mode(kModeNominal);
            }
            break;
        case Mode::Nominal:
            if (low || !att_ok) {
                set_mode(kModeSafe);
            } else if (reduced) {
                set_mode(kModeDegraded);
            }
            break;
        case Mode::Degraded:
            if (low || !att_ok) {
                set_mode(kModeSafe);
            } else if (clear_count_ >= kDegradedClear) {
                set_mode(kModeNominal);
            }
            break;
        case Mode::Safe:
            recover_count_ = (soc_est_ > kSafeExit && att_ok) ? static_cast<uint16_t>(recover_count_ + 1) : 0;
            if (recover_count_ >= kSafeExitPersist) {
                recover_count_ = 0;
                set_mode(kModeRecovery);
            }
            break;
        case Mode::Recovery:
            if (low || !att_ok) {
                set_mode(kModeSafe);
            } else if (mode_ticks_ >= kRecoveryHold) {
                set_mode(reduced ? kModeDegraded : kModeNominal);
            }
            break;
        case Mode::Invalid:
            set_mode(kModeSafe);
            break;
    }
}

void HardenedController::onboard_schedule(uint32_t tick) {
    const uint32_t orbit = tick / kOrbitTicks;
    if (!comm_lost_ || orbit == last_sched_orbit_ || future_window_in_orbit(tick)) {
        return;
    }
    last_sched_orbit_ = orbit;

    const uint32_t orbit_start = orbit * kOrbitTicks;
    std::array<Candidate, kOnboardTargets.size()> open{};
    std::size_t n_open = 0;
    for (const Candidate& c : kOnboardTargets) {
        if (orbit_start + c.offset > tick) {
            open[n_open++] = c;
        }
    }
    const double budget = std::max(0.0F, soc_est_ - 0.40F) * kNominalCapacityJ;
    std::array<uint8_t, kOnboardTargets.size()> chosen{};
    const std::size_t n = greedy_schedule(std::span<const Candidate>(open.data(), n_open), budget, kPayloadW, chosen);
    for (std::size_t i = 0; i < n; ++i) {
        const Candidate& c = open[chosen[i]];
        add_window({orbit_start + c.offset, c.duration, 0, c.value, 1, 0});
    }
}

bool HardenedController::update_payload(uint32_t tick) {
    const Mode mode = decode_hardened_mode(crit_.mode.vote());
    const bool was_on = crit_.payload_on.vote() != 0;
    bool on = false;
    if ((mode == Mode::Nominal || mode == Mode::Degraded) && window_active(tick)) {
        const float on_at = mode == Mode::Nominal ? 0.35F : 0.50F;
        const float off_at = mode == Mode::Nominal ? 0.30F : 0.45F;
        on = was_on ? soc_est_ > off_at : soc_est_ > on_at;
    }
    // Recomputed and rewritten every cycle, so a flipped latch cannot persist.
    crit_.payload_on.set(on ? 1 : 0);
    return on;
}

ControlOutput HardenedController::step(uint32_t tick, const SensorFrame& s, std::span<const Frame> rx) {
    ControlOutput out;
    check_memory(out);
    handle_uplink(tick, rx, out);
    estimate_attitude(s, out);
    estimate_battery(s, out);
    update_mode(tick);
    onboard_schedule(tick);
    const bool payload = update_payload(tick);

    const float omega = s.gyro - bias_est_;
    const bool att_ok = !(sun_failed_[0] && sun_failed_[1]);
    out.torque = att_ok ? pd_torque(theta_est_, omega, payload ? kPayloadPointing : 0.0F)
                        : std::clamp(-kKd * omega, -kMaxTorque, kMaxTorque);
    out.mode_raw = crit_.mode.vote();
    out.mode = decode_hardened_mode(out.mode_raw);
    out.payload_on = payload;
    return out;
}

}  // namespace bdio
