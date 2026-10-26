#include "sfs/spacecraft.hpp"

#include <algorithm>
#include <cmath>

namespace sfs {

Spacecraft::Spacecraft(const SpacecraftParams& params, uint64_t seed, double initial_soc)
    : params_(params), rng_(seed) {
    truth_.capacity_j = params_.capacity_j;
    truth_.energy_j = initial_soc * params_.capacity_j;
    truth_.theta = rng_.uniform(-0.3, 0.3);
    truth_.omega = rng_.uniform(-0.005, 0.005);
}

void Spacecraft::step(uint32_t tick, const ControlOutput& cmd) {
    // Attitude: semi-implicit Euler on a unit-inertia rigid body.
    truth_.disturbance += rng_.normal(params_.disturbance_sigma);
    truth_.disturbance = std::clamp(truth_.disturbance, -5e-4, 5e-4);
    const double u = std::clamp(static_cast<double>(cmd.torque), -params_.max_torque, params_.max_torque);
    truth_.omega += (u + truth_.disturbance) * kDt;
    truth_.theta += truth_.omega * kDt;
    truth_.theta = std::remainder(truth_.theta, 2.0 * 3.14159265358979323846);

    // Power.
    truth_.payload_on = cmd.payload_on;
    const bool eclipse = in_eclipse(tick);
    truth_.solar_w = eclipse ? 0.0 : params_.solar_max_w * std::max(0.0, std::cos(truth_.theta));
    truth_.load_w = params_.bus_w + (cmd.payload_on ? params_.payload_w : 0.0) +
                    (in_pass(tick) ? params_.radio_w : 0.0) + (eclipse ? params_.heater_w : 0.0);
    last_net_w_ = truth_.solar_w - truth_.load_w;
    truth_.energy_j = std::clamp(truth_.energy_j + last_net_w_ * kDt, 0.0, truth_.capacity_j);
}

SensorFrame Spacecraft::sense(uint32_t tick) {
    SensorFrame f;
    const bool sun_visible = !in_eclipse(tick) && std::abs(truth_.theta) < params_.sun_fov_rad;
    f.sun_a_valid = sun_visible;
    f.sun_b_valid = sun_visible;
    f.sun_a = sun_visible ? static_cast<float>(truth_.theta + rng_.normal(params_.sun_noise)) : 0.0F;
    f.sun_b = sun_visible ? static_cast<float>(truth_.theta + rng_.normal(params_.sun_noise)) : 0.0F;
    f.gyro = static_cast<float>(truth_.omega + params_.gyro_bias + rng_.normal(params_.gyro_noise));
    f.batt_v = static_cast<float>(kBattVEmpty + kBattVSpan * soc() + rng_.normal(params_.batt_v_noise));
    f.net_power_w = static_cast<float>(last_net_w_ + rng_.normal(params_.power_noise));
    return f;
}

void Spacecraft::lose_capacity(double fraction) {
    truth_.capacity_j *= (1.0 - fraction);
    truth_.energy_j *= (1.0 - fraction);
    // The failed cell also takes charge with it: the remaining pack ends up
    // noticeably lower than the coulomb counter believes.
    truth_.energy_j *= 0.6;
}

}  // namespace sfs
