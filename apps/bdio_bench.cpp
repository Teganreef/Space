// Measures the execution time of one control cycle, reporting the tail
// rather than the mean. For a periodic control loop the worst case is what
// matters: the average says nothing about whether a deadline gets missed.
//
// Numbers depend on the host. This runs on a desktop OS with no real-time
// guarantees, so treat the tail as an upper bound that includes scheduler noise.

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <vector>

#include "bdio/faults.hpp"
#include "bdio/hardened_controller.hpp"
#include "bdio/naive_controller.hpp"
#include "bdio/spacecraft.hpp"

namespace {

template <typename Ctrl>
void bench(const char* name, uint32_t cycles) {
    using clock = std::chrono::steady_clock;
    bdio::Spacecraft sc(bdio::SpacecraftParams{}, 7, 0.7);
    bdio::FaultInjector inj(bdio::Scenario::Combined, 7, cycles);
    Ctrl ctrl;
    std::vector<double> ns;
    ns.reserve(cycles);

    for (uint32_t t = 0; t < cycles; ++t) {
        inj.apply_seu(t, ctrl.seu_region());
        bdio::SensorFrame s = sc.sense(t);
        inj.apply_sensor_faults(t, s);
        const auto t0 = clock::now();
        const bdio::ControlOutput out = ctrl.step(t, s, {});
        const auto t1 = clock::now();
        ns.push_back(static_cast<double>(std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count()));
        sc.step(t, out);
    }

    std::sort(ns.begin(), ns.end());
    auto pct = [&](double p) { return ns[static_cast<std::size_t>(p * static_cast<double>(ns.size() - 1))]; };
    std::printf("%-9s cycles=%u  p50=%.0fns  p99=%.0fns  p99.9=%.0fns  max=%.0fns\n", name, cycles, pct(0.50),
                pct(0.99), pct(0.999), ns.back());
}

}  // namespace

int main(int argc, char** argv) {
    const uint32_t cycles = argc > 1 ? static_cast<uint32_t>(std::strtoul(argv[1], nullptr, 10)) : 1000000;
    bench<bdio::NaiveController>("naive", cycles);
    bench<bdio::HardenedController>("hardened", cycles);
    return 0;
}
