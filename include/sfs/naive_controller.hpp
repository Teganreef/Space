#pragma once

#include <array>
#include <type_traits>

#include "sfs/controller.hpp"

namespace sfs {

// Baseline flight software: functionally correct when nothing goes wrong.
// It trusts its primary sensors, trusts the link layer, keeps one copy of
// its state, and reacts to any out-of-range reading immediately. Every
// shortcut here is a common one, which is the point of the comparison.
class NaiveController final : public Controller {
public:
    NaiveController();

    ControlOutput step(uint32_t tick, const SensorFrame& sensors, std::span<const Frame> rx) override;
    std::span<std::byte> seu_region() override;
    const char* name() const override { return "naive"; }
    float soc_estimate() const override { return soc_est_; }
    float theta_estimate() const override { return theta_est_; }

    struct State {
        uint8_t mode = 0;
        uint8_t payload_on = 0;
        uint8_t n_plan = 0;
        uint8_t spare = 0;
        std::array<PayloadWindow, 8> plan{};
    };
    static_assert(std::has_unique_object_representations_v<State>);

private:
    void add_window(const PayloadWindow& w);
    bool window_active(uint32_t tick) const;

    State state_;
    uint32_t mode_ticks_ = 0;
    float theta_est_ = 0.0F;
    float soc_est_ = 0.5F;
};

}  // namespace sfs
