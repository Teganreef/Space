#include <gtest/gtest.h>

#include "bdio/harness.hpp"

using namespace bdio;

namespace {
RunMetrics run(Scenario s, uint64_t seed, ControllerKind k) {
    RunOptions o;
    o.scenario = s;
    o.seed = seed;
    o.kind = k;
    return run_mission(o);
}
}  // namespace

TEST(Harness, SameSeedReproducesTheRunExactly) {
    for (const auto k : {ControllerKind::Naive, ControllerKind::Hardened}) {
        const std::string a = to_json(run(Scenario::Combined, 77, k));
        const std::string b = to_json(run(Scenario::Combined, 77, k));
        EXPECT_EQ(a, b);
    }
}

TEST(Harness, DifferentSeedsDiffer) {
    EXPECT_NE(to_json(run(Scenario::Combined, 1, ControllerKind::Hardened)),
              to_json(run(Scenario::Combined, 2, ControllerKind::Hardened)));
}

TEST(Harness, BothControllersSeeTheSameFaults) {
    const RunMetrics n = run(Scenario::Sensor, 11, ControllerKind::Naive);
    const RunMetrics h = run(Scenario::Sensor, 11, ControllerKind::Hardened);
    EXPECT_EQ(n.faults_scored + n.faults_unobservable, h.faults_scored + h.faults_unobservable);
    EXPECT_EQ(n.planned_science_ticks, h.planned_science_ticks);
}

TEST(Harness, NominalHardenedRunIsClean) {
    for (uint64_t seed = 1; seed <= 10; ++seed) {
        const RunMetrics m = run(Scenario::Nominal, seed, ControllerKind::Hardened);
        EXPECT_EQ(m.total_violations, 0U) << seed;
        EXPECT_EQ(m.safe_entries, 0U) << seed;
        EXPECT_EQ(m.mode_ticks[static_cast<std::size_t>(Mode::Degraded)], 0U) << seed;
    }
}

// Regression: before the estimator was initialised from the first agreeing
// measurement, the start-up transient was learned as gyro bias and about
// half of all nominal runs spent the whole mission in degraded mode.
TEST(Harness, StartupTransientIsNotMistakenForGyroFault) {
    for (uint64_t seed = 1; seed <= 40; ++seed) {
        const RunMetrics m = run(Scenario::Nominal, seed, ControllerKind::Hardened);
        EXPECT_EQ(m.mode_ticks[static_cast<std::size_t>(Mode::Degraded)], 0U) << seed;
    }
}

TEST(Harness, HardenedSurvivesUpsetStormWithoutViolations) {
    for (uint64_t seed = 1; seed <= 5; ++seed) {
        EXPECT_EQ(run(Scenario::SeuStorm, seed, ControllerKind::Hardened).total_violations, 0U) << seed;
    }
}
