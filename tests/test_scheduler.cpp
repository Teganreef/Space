#include <gtest/gtest.h>

#include "bdio/scheduler.hpp"

using namespace bdio;

TEST(Greedy, RespectsBudgetAndConflicts) {
    std::array<uint8_t, kOnboardTargets.size()> out{};
    const double payload_w = 8.0;
    for (const double budget : {0.0, 2000.0, 5000.0, 12000.0, 1e9}) {
        const std::size_t n = greedy_schedule(kOnboardTargets, budget, payload_w, out);
        double spent = 0.0;
        for (std::size_t i = 0; i < n; ++i) {
            spent += kOnboardTargets[out[i]].duration * payload_w;
            for (std::size_t j = i + 1; j < n; ++j) {
                EXPECT_FALSE(overlaps(kOnboardTargets[out[i]], kOnboardTargets[out[j]]));
            }
        }
        EXPECT_LE(spent, budget);
    }
}

TEST(Greedy, ZeroBudgetSchedulesNothing) {
    std::array<uint8_t, kOnboardTargets.size()> out{};
    EXPECT_EQ(greedy_schedule(kOnboardTargets, 0.0, 8.0, out), 0U);
}

TEST(Greedy, TakesHighestDensityFirst) {
    std::array<uint8_t, kOnboardTargets.size()> out{};
    ASSERT_GT(greedy_schedule(kOnboardTargets, 1e9, 8.0, out), 0U);
    // Index 1 (6 value / 500 s) has the best value per joule in the catalog.
    EXPECT_EQ(out[0], 1);
}
