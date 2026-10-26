#pragma once

#include <cmath>

#include "bdio/packet.hpp"
#include "bdio/spacecraft.hpp"
#include "bdio/types.hpp"

namespace bdio::testing {

// A healthy sensor frame. Values carry a tiny tick-dependent jitter because
// the hardened controller treats bit-identical consecutive samples as a
// frozen sensor, which is exactly what a constant test input looks like.
inline SensorFrame healthy(uint32_t tick, float theta = 0.0F, float soc = 0.6F) {
    const float j = 1e-4F * std::sin(static_cast<float>(tick) * 0.7F);
    SensorFrame f;
    f.sun_a_valid = true;
    f.sun_b_valid = true;
    f.sun_a = theta + j;
    f.sun_b = theta - j;
    f.gyro = 2e-4F + 0.1F * j;
    f.batt_v = static_cast<float>(kBattVEmpty + kBattVSpan * soc) + 10.0F * j;
    f.net_power_w = 0.0F;
    return f;
}

inline Frame window_cmd(uint16_t seq, uint32_t send_tick, uint32_t start, uint16_t duration) {
    return encode({seq, send_tick, CommandType::AddPayloadWindow, start, duration, 5});
}

}  // namespace bdio::testing
