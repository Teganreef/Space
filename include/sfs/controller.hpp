#pragma once

#include <cstddef>
#include <span>

#include "sfs/packet.hpp"
#include "sfs/types.hpp"

namespace sfs {

// Flight software interface. step() runs once per tick and must not allocate:
// all state is sized at construction. seu_region() exposes the bytes that the
// fault injector is allowed to corrupt, which is the controller's critical
// discrete state (mode, output latches, command table).
class Controller {
public:
    virtual ~Controller() = default;

    virtual ControlOutput step(uint32_t tick, const SensorFrame& sensors, std::span<const Frame> rx) = 0;
    virtual std::span<std::byte> seu_region() = 0;
    virtual const char* name() const = 0;

    // Estimates exposed for the flight recorder only.
    virtual float soc_estimate() const = 0;
    virtual float theta_estimate() const = 0;
};

// Attitude control gains shared by both controllers so that differences in
// results come from fault handling, not tuning.
inline constexpr float kKp = 0.01F;
inline constexpr float kKd = 0.15F;
inline constexpr float kMaxTorque = 0.01F;
inline constexpr float kPayloadPointing = 0.6F;  // payload target is off-sun
inline constexpr uint32_t kBootTicks = 10;
inline constexpr uint32_t kMaxCommandAge = 60;

float pd_torque(float theta_est, float omega_est, float target);

}  // namespace sfs
