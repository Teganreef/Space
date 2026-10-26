#pragma once

#include <cstdint>

#include "bdio/rng.hpp"
#include "bdio/types.hpp"

namespace bdio {

// Deliberately small physics: one rotation axis (angle between the solar
// panel normal and the sun) and an energy-balance battery. Enough for sensor
// faults to cause mispointing, mispointing to cost power, and power to matter.
struct SpacecraftParams {
    double capacity_j = 144000.0;  // 40 Wh pack
    double solar_max_w = 10.0;
    double bus_w = 3.0;
    double payload_w = 8.0;
    double radio_w = 4.0;
    double heater_w = 1.0;
    double max_torque = 0.01;  // [rad/s^2], unit inertia
    double disturbance_sigma = 2e-5;
    double sun_fov_rad = 1.2;  // sun sensors only see the sun inside +/- this angle

    // Sensor characteristics.
    double sun_noise = 0.01;
    double gyro_noise = 5e-5;
    double gyro_bias = 2e-4;
    double batt_v_noise = 0.01;
    double power_noise = 0.05;
};

// Battery voltage model: 2S lithium-ion, linear in state of charge.
inline constexpr double kBattVEmpty = 6.0;
inline constexpr double kBattVSpan = 2.4;

struct Truth {
    double theta = 0.0;
    double omega = 0.0;
    double energy_j = 0.0;
    double capacity_j = 0.0;
    double solar_w = 0.0;
    double load_w = 0.0;
    double disturbance = 0.0;
    bool payload_on = false;
};

class Spacecraft {
public:
    Spacecraft(const SpacecraftParams& params, uint64_t seed, double initial_soc);

    // Advance the truth state by one tick using the controller's commands.
    void step(uint32_t tick, const ControlOutput& cmd);

    // Ideal sensor readings plus noise. Faults are applied afterwards by the
    // fault injector, which owns the decision of what lies and how.
    SensorFrame sense(uint32_t tick);

    // Instantaneous loss of part of the pack (shorted cell).
    void lose_capacity(double fraction);

    double soc() const { return truth_.energy_j / truth_.capacity_j; }
    double soc_of_nominal() const { return truth_.energy_j / params_.capacity_j; }
    const Truth& truth() const { return truth_; }
    const SpacecraftParams& params() const { return params_; }

private:
    SpacecraftParams params_;
    Truth truth_;
    Rng rng_;
    double last_net_w_ = 0.0;
};

}  // namespace bdio
