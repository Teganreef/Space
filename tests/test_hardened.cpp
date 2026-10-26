#include <gtest/gtest.h>

#include <algorithm>
#include <array>

#include "bdio/hardened_controller.hpp"
#include "sim_helpers.hpp"

using namespace bdio;
using bdio::testing::healthy;
using bdio::testing::window_cmd;

namespace {

class Hardened : public ::testing::Test {
protected:
    // Steps until past boot so tests start in nominal mode.
    void SetUp() override {
        for (; tick_ < 20; ++tick_) {
            last_ = ctrl_.step(tick_, healthy(tick_), {});
        }
        ASSERT_EQ(last_.mode, Mode::Nominal);
    }

    ControlOutput step(const SensorFrame& s) {
        last_ = ctrl_.step(tick_, s, {});
        ++tick_;
        return last_;
    }
    ControlOutput step() { return step(healthy(tick_)); }
    ControlOutput step_rx(const Frame& f) {
        last_ = ctrl_.step(tick_, healthy(tick_), std::span<const Frame>(&f, 1));
        ++tick_;
        return last_;
    }

    HardenedController ctrl_;
    uint32_t tick_ = 0;
    ControlOutput last_;
};

}  // namespace

TEST_F(Hardened, ExecutesValidCommand) {
    const auto out = step_rx(window_cmd(1, tick_, tick_ + 100, 300));
    EXPECT_EQ(out.n_executed, 1);
    EXPECT_EQ(out.executed_seq[0], 1);
}

TEST_F(Hardened, RejectsDuplicateSequenceNumber) {
    step_rx(window_cmd(5, tick_, tick_ + 100, 300));
    const auto out = step_rx(window_cmd(5, tick_, tick_ + 100, 300));
    EXPECT_EQ(out.n_executed, 0);
    EXPECT_TRUE(out.detected & kDetComm);
}

TEST_F(Hardened, RejectsOlderSequenceNumber) {
    step_rx(window_cmd(9, tick_, tick_ + 100, 300));
    const auto out = step_rx(window_cmd(8, tick_, tick_ + 100, 300));
    EXPECT_EQ(out.n_executed, 0);
}

TEST_F(Hardened, RejectsStaleCommand) {
    const auto out = step_rx(window_cmd(3, tick_ - kMaxCommandAge - 1, tick_ + 100, 300));
    EXPECT_EQ(out.n_executed, 0);
    EXPECT_TRUE(out.detected & kDetComm);
}

TEST_F(Hardened, RejectsCorruptedFrame) {
    Frame f = window_cmd(4, tick_, tick_ + 100, 300);
    f[13] ^= 0x40;
    const auto out = step_rx(f);
    EXPECT_EQ(out.n_executed, 0);
    EXPECT_TRUE(out.detected & kDetComm);
}

TEST_F(Hardened, RunsPayloadInsideCommandedWindow) {
    step_rx(window_cmd(1, tick_, tick_ + 5, 50));
    for (int i = 0; i < 10; ++i) {
        step();
    }
    EXPECT_TRUE(last_.payload_on);
    for (int i = 0; i < 60; ++i) {
        step();
    }
    EXPECT_FALSE(last_.payload_on);
}

TEST_F(Hardened, SingleUpsetInModeIsCorrected) {
    ctrl_.critical().mode.b ^= 0x10;
    const auto out = step();
    EXPECT_TRUE(out.detected & kDetMemory);
    EXPECT_EQ(out.mode, Mode::Nominal);
    EXPECT_EQ(ctrl_.critical().mode.b, kModeNominal);
}

TEST_F(Hardened, MatchingUpsetsInTwoCopiesFailSafe) {
    ctrl_.critical().mode.a ^= 0x01;
    ctrl_.critical().mode.c ^= 0x01;
    const auto out = step();
    EXPECT_TRUE(out.detected & kDetMemory);
    EXPECT_EQ(out.mode, Mode::Safe);
    EXPECT_FALSE(out.payload_on);
}

TEST_F(Hardened, FlippedPayloadLatchDoesNotPersist) {
    ctrl_.critical().payload_on.set(1);  // all three copies, no window active
    const auto out = step();
    EXPECT_FALSE(out.payload_on);
}

TEST_F(Hardened, DamagedPlanCopyIsRepaired) {
    step_rx(window_cmd(1, tick_, tick_ + 5, 100));
    ctrl_.critical().plan.windows[0].duration ^= 0x0100;
    const auto out = step();
    EXPECT_TRUE(out.detected & kDetMemory);
    EXPECT_EQ(ctrl_.critical().plan.windows[0].duration, 100);
    for (int i = 0; i < 10; ++i) {
        step();
    }
    EXPECT_TRUE(last_.payload_on);
}

TEST_F(Hardened, BothPlanCopiesDamagedDropsPlan) {
    step_rx(window_cmd(1, tick_, tick_ + 5, 100));
    ctrl_.critical().plan.windows[0].start ^= 1;
    ctrl_.critical().plan_backup.windows[0].value ^= 1;
    step();
    EXPECT_EQ(ctrl_.critical().plan.n, 0);
    for (int i = 0; i < 10; ++i) {
        step();
    }
    EXPECT_FALSE(last_.payload_on);
}

TEST_F(Hardened, FrozenSunSensorIsIsolated) {
    uint8_t seen = 0;
    for (int i = 0; i < 20; ++i) {
        SensorFrame s = healthy(tick_);
        s.sun_a = 0.0123F;
        seen |= step(s).detected;
    }
    EXPECT_TRUE(seen & kDetAttitude);
    EXPECT_TRUE(ctrl_.sun_failed(0));
    EXPECT_FALSE(ctrl_.sun_failed(1));
    EXPECT_EQ(last_.mode, Mode::Degraded);
}

TEST_F(Hardened, BiasedSunSensorIsIsolated) {
    for (int i = 0; i < 10; ++i) {
        SensorFrame s = healthy(tick_);
        s.sun_b += 0.4F;
        step(s);
    }
    EXPECT_TRUE(ctrl_.sun_failed(1));
    EXPECT_FALSE(ctrl_.sun_failed(0));
    EXPECT_NEAR(ctrl_.theta_estimate(), 0.0F, 0.02F);
}

TEST_F(Hardened, SingleGlitchIsIgnored) {
    SensorFrame s = healthy(tick_);
    s.sun_a += 1.2F;
    s.batt_v += 1.3F;
    const auto out = step(s);
    EXPECT_EQ(out.detected, 0);
    for (int i = 0; i < 30; ++i) {
        step();
    }
    EXPECT_FALSE(ctrl_.sun_failed(0));
    EXPECT_EQ(last_.mode, Mode::Nominal);
}

TEST_F(Hardened, HighReadingBatterySensorIsDistrusted) {
    uint8_t seen = 0;
    for (int i = 0; i < 40; ++i) {
        SensorFrame s = healthy(tick_, 0.0F, 0.6F);
        s.batt_v += 0.6F;  // claims ~25% more charge than there is
        seen |= step(s).detected;
    }
    EXPECT_TRUE(seen & kDetBattery);
    EXPECT_TRUE(ctrl_.battery_suspect());
    EXPECT_LT(ctrl_.soc_estimate(), 0.65F);
}

TEST_F(Hardened, LowBatteryEntersSafeThenRecovers) {
    for (int i = 0; i < 30; ++i) {
        step(healthy(tick_, 0.0F, 0.15F));
    }
    EXPECT_EQ(last_.mode, Mode::Safe);
    EXPECT_FALSE(last_.payload_on);

    // Recharge with voltage and measured current telling the same story.
    // (An instant jump in voltage with no current behind it is treated as a
    // sensor fault, which is the point of the cross-check.)
    const float charge_w = 200.0F;
    float soc = 0.15F;
    bool saw_recovery = false;
    for (int i = 0; i < 1500; ++i) {
        soc = std::min(0.9F, soc + charge_w / 144000.0F);
        SensorFrame s = healthy(tick_, 0.0F, soc);
        s.net_power_w = soc < 0.9F ? charge_w : 0.0F;
        const Mode m = step(s).mode;
        saw_recovery = saw_recovery || m == Mode::Recovery;
    }
    EXPECT_TRUE(saw_recovery);
    EXPECT_FALSE(ctrl_.battery_suspect());
    EXPECT_EQ(last_.mode, Mode::Nominal);
}

TEST_F(Hardened, ShortDipDoesNotTriggerSafeMode) {
    for (int i = 0; i < 5; ++i) {
        step(healthy(tick_, 0.0F, 0.15F));
    }
    EXPECT_EQ(last_.mode, Mode::Nominal);
}
