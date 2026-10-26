#include <gtest/gtest.h>

#include "bdio/naive_controller.hpp"
#include "sim_helpers.hpp"

using namespace bdio;
using bdio::testing::healthy;
using bdio::testing::window_cmd;

// These document the baseline's weaknesses rather than bugs to fix. If one
// of them starts failing the comparison in the campaign changes meaning.

TEST(Naive, ExecutesDuplicates) {
    NaiveController c;
    const Frame f = window_cmd(1, 20, 100, 50);
    uint32_t t = 0;
    for (; t < 20; ++t) {
        c.step(t, healthy(t), {});
    }
    EXPECT_EQ(c.step(t, healthy(t), std::span<const Frame>(&f, 1)).n_executed, 1);
    ++t;
    EXPECT_EQ(c.step(t, healthy(t), std::span<const Frame>(&f, 1)).n_executed, 1);
}

TEST(Naive, OneBadSampleTriggersSafeMode) {
    NaiveController c;
    uint32_t t = 0;
    for (; t < 20; ++t) {
        c.step(t, healthy(t), {});
    }
    SensorFrame s = healthy(t);
    s.batt_v += 1.3F;
    EXPECT_EQ(c.step(t, s, {}).mode, Mode::Safe);
}

TEST(Naive, SingleUpsetCanCorruptMode) {
    NaiveController c;
    uint32_t t = 0;
    for (; t < 20; ++t) {
        c.step(t, healthy(t), {});
    }
    c.seu_region()[0] ^= std::byte{0x40};
    EXPECT_EQ(c.step(t, healthy(t), {}).mode, Mode::Invalid);
}
