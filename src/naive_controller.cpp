#include "sfs/naive_controller.hpp"

#include <algorithm>
#include <cmath>

#include "sfs/spacecraft.hpp"

namespace sfs {

namespace {

enum NaiveMode : uint8_t { kBoot = 0, kNominal = 1, kSafe = 2 };

Mode decode_mode(uint8_t raw) {
    switch (raw) {
        case kBoot: return Mode::Boot;
        case kNominal: return Mode::Nominal;
        case kSafe: return Mode::Safe;
        default: return Mode::Invalid;
    }
}

bool out_of_range(const SensorFrame& s) {
    return (s.sun_a_valid && std::abs(s.sun_a) > 1.3F) || s.batt_v < 5.9F || s.batt_v > 8.5F;
}

}  // namespace

float pd_torque(float theta_est, float omega_est, float target) {
    const float u = -kKp * (theta_est - target) - kKd * omega_est;
    return std::clamp(u, -kMaxTorque, kMaxTorque);
}

NaiveController::NaiveController() { state_.mode = kBoot; }

std::span<std::byte> NaiveController::seu_region() {
    return {reinterpret_cast<std::byte*>(&state_), sizeof(state_)};
}

void NaiveController::add_window(const PayloadWindow& w) {
    if (state_.n_plan < state_.plan.size()) {
        state_.plan[state_.n_plan++] = w;
        return;
    }
    // Full: overwrite the slot that ends earliest.
    std::size_t oldest = 0;
    for (std::size_t i = 1; i < state_.plan.size(); ++i) {
        if (state_.plan[i].start + state_.plan[i].duration <
            state_.plan[oldest].start + state_.plan[oldest].duration) {
            oldest = i;
        }
    }
    state_.plan[oldest] = w;
}

bool NaiveController::window_active(uint32_t tick) const {
    const std::size_t n = std::min<std::size_t>(state_.n_plan, state_.plan.size());
    for (std::size_t i = 0; i < n; ++i) {
        const PayloadWindow& w = state_.plan[i];
        if (tick >= w.start && tick < w.start + w.duration) {
            return true;
        }
    }
    return false;
}

ControlOutput NaiveController::step(uint32_t tick, const SensorFrame& s, std::span<const Frame> rx) {
    ControlOutput out;

    // Commands: parsed and executed as received.
    for (const Frame& frame : rx) {
        Command cmd;
        decode_unchecked(frame, cmd);
        add_window({cmd.start, cmd.duration, cmd.seq, cmd.value, 0, 0});
        if (out.n_executed < kMaxExecPerTick) {
            out.executed_seq[out.n_executed] = cmd.seq;
            out.executed_age[out.n_executed] = tick >= cmd.send_tick ? tick - cmd.send_tick : 0;
            ++out.n_executed;
        }
    }

    // Estimation: primary sun sensor when it has a reading, gyro otherwise.
    theta_est_ = s.sun_a_valid ? s.sun_a : theta_est_ + s.gyro * static_cast<float>(kDt);
    soc_est_ = (s.batt_v - static_cast<float>(kBattVEmpty)) / static_cast<float>(kBattVSpan);

    // Mode logic. Unknown mode values fall through untouched.
    ++mode_ticks_;
    switch (state_.mode) {
        case kBoot:
            if (mode_ticks_ >= kBootTicks) {
                state_.mode = kNominal;
                mode_ticks_ = 0;
            }
            break;
        case kNominal:
            if (soc_est_ < 0.25F || out_of_range(s)) {
                if (soc_est_ < 0.25F) {
                    out.detected |= kDetPower;
                }
                state_.mode = kSafe;
                state_.payload_on = 0;
                mode_ticks_ = 0;
            } else if (!state_.payload_on && window_active(tick) && soc_est_ > 0.30F) {
                state_.payload_on = 1;
            } else if (state_.payload_on && (!window_active(tick) || soc_est_ < 0.30F)) {
                state_.payload_on = 0;
            }
            break;
        case kSafe:
            if (soc_est_ >= 0.25F && !out_of_range(s)) {
                state_.mode = kNominal;
                mode_ticks_ = 0;
            }
            break;
        default:
            break;
    }

    const float target = state_.payload_on ? kPayloadPointing : 0.0F;
    out.torque = pd_torque(theta_est_, s.gyro, target);
    out.mode_raw = state_.mode;
    out.mode = decode_mode(state_.mode);
    out.payload_on = state_.payload_on != 0;
    return out;
}

}  // namespace sfs
