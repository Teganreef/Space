// Verifies the "no dynamic allocation after initialisation" rule for the
// flight code by replacing global operator new for this test binary and
// counting calls made while only controller step() is running.

#include <gtest/gtest.h>

#include <atomic>
#include <cstdlib>
#include <new>

#include "bdio/faults.hpp"
#include "bdio/hardened_controller.hpp"
#include "bdio/naive_controller.hpp"
#include "bdio/spacecraft.hpp"
#include "sim_helpers.hpp"

namespace {
std::atomic<long> g_allocs{0};
}

void* operator new(std::size_t n) {
    ++g_allocs;
    if (void* p = std::malloc(n == 0 ? 1 : n)) {
        return p;
    }
    throw std::bad_alloc();
}
void operator delete(void* p) noexcept { std::free(p); }
void operator delete(void* p, std::size_t) noexcept { std::free(p); }

namespace {

template <typename Ctrl>
long allocations_during_mission() {
    using namespace bdio;
    Spacecraft sc(SpacecraftParams{}, 3, 0.55);
    FaultInjector inj(Scenario::Combined, 3, kMissionTicks);
    Ctrl ctrl;
    const Frame f = bdio::testing::window_cmd(1, 1030, 1800, 400);

    std::array<SensorFrame, 1> scratch{};
    const long before = g_allocs.load();
    for (uint32_t t = 0; t < kMissionTicks; ++t) {
        inj.apply_seu(t, ctrl.seu_region());
        scratch[0] = sc.sense(t);
        inj.apply_sensor_faults(t, scratch[0]);
        const bool send = t == 1030;
        const ControlOutput out = ctrl.step(t, scratch[0], send ? std::span<const Frame>(&f, 1) : std::span<const Frame>{});
        sc.step(t, out);
    }
    return g_allocs.load() - before;
}

}  // namespace

TEST(NoAlloc, HardenedStepNeverAllocates) { EXPECT_EQ(allocations_during_mission<bdio::HardenedController>(), 0); }

TEST(NoAlloc, NaiveStepNeverAllocates) { EXPECT_EQ(allocations_during_mission<bdio::NaiveController>(), 0); }
