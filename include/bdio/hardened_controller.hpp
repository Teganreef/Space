#pragma once

#include <array>
#include <type_traits>

#include "bdio/controller.hpp"
#include "bdio/tmr.hpp"

namespace bdio {

// Mode encodings chosen with a minimum pairwise Hamming distance of 4, so a
// single bit flip can never turn one valid mode into another valid mode.
inline constexpr uint8_t kModeBoot = 0x0F;
inline constexpr uint8_t kModeNominal = 0x3C;
inline constexpr uint8_t kModeDegraded = 0x55;
inline constexpr uint8_t kModeSafe = 0xA5;
inline constexpr uint8_t kModeRecovery = 0xC3;

Mode decode_hardened_mode(uint8_t raw);

// Fault-tolerant flight software. Differences from the baseline:
//  - critical discrete state in TMR, scrubbed every cycle
//  - command table protected by CRC; corruption discards it
//  - uplink frames checked for CRC, replay (sequence number) and age
//  - two sun sensors cross-checked against gyro propagation; a sensor is
//    only declared failed after a persistent disagreement
//  - battery voltage cross-checked against coulomb counting with
//    asymmetric trust: readings claiming less energy are believed quickly,
//    readings claiming more energy have to keep agreeing to be believed
//  - mode transitions use persistence and hysteresis
//  - degraded mode with an onboard scheduler when the ground goes quiet
class HardenedController final : public Controller {
public:
    HardenedController();

    ControlOutput step(uint32_t tick, const SensorFrame& sensors, std::span<const Frame> rx) override;
    std::span<std::byte> seu_region() override;
    const char* name() const override { return "hardened"; }
    float soc_estimate() const override { return soc_est_; }
    float theta_estimate() const override { return theta_est_; }

    struct Critical {
        Tmr<uint8_t> mode;
        Tmr<uint8_t> payload_on;
        Tmr<uint16_t> last_seq;
        uint8_t n_plan = 0;
        uint8_t spare = 0;
        uint16_t crc = 0;
        std::array<PayloadWindow, 8> plan{};
    };
    static_assert(std::has_unique_object_representations_v<Critical>);

    // Test hooks.
    Critical& critical() { return crit_; }
    bool sun_failed(int i) const { return sun_failed_[static_cast<std::size_t>(i)]; }
    bool battery_suspect() const { return batt_suspect_; }
    bool comm_lost() const { return comm_lost_; }

private:
    uint16_t plan_crc() const;
    void reseal_plan();
    void add_window(const PayloadWindow& w);
    bool window_active(uint32_t tick) const;
    bool future_window_in_orbit(uint32_t tick) const;
    void set_mode(uint8_t code);

    void check_memory(ControlOutput& out);
    void handle_uplink(uint32_t tick, std::span<const Frame> rx, ControlOutput& out);
    void estimate_attitude(const SensorFrame& s, ControlOutput& out);
    void estimate_battery(const SensorFrame& s, ControlOutput& out);
    void update_mode(uint32_t tick);
    void onboard_schedule(uint32_t tick);
    bool update_payload(uint32_t tick);

    Critical crit_;

    uint32_t mode_ticks_ = 0;
    uint32_t boot_samples_ = 0;

    float theta_est_ = 0.0F;
    float bias_est_ = 0.0F;
    std::array<bool, 2> sun_failed_{};
    std::array<uint16_t, 2> sun_outlier_count_{};
    std::array<uint16_t, 2> sun_flat_count_{};
    std::array<uint16_t, 2> sun_good_count_{};
    std::array<float, 2> sun_prev_{};
    uint16_t single_disagree_count_ = 0;
    bool gyro_flagged_ = false;
    bool att_initialized_ = false;

    float soc_v_ = 0.5F;
    float soc_cc_ = 0.5F;
    float soc_est_ = 0.5F;
    float last_batt_v_ = 0.0F;
    bool batt_suspect_ = false;
    float batt_offset_ = 0.0F;
    uint16_t batt_high_count_ = 0;
    uint16_t batt_low_count_ = 0;
    uint16_t batt_flat_count_ = 0;
    uint16_t batt_good_count_ = 0;

    uint16_t low_soc_count_ = 0;
    uint16_t recover_count_ = 0;
    uint16_t clear_count_ = 0;

    uint32_t last_rx_tick_ = 0;
    bool comm_lost_ = false;
    uint32_t last_sched_orbit_ = 0xFFFFFFFF;
};

}  // namespace bdio
