#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

namespace bdio {

// A payload opportunity the spacecraft knows about without the ground: a
// fixed target list, expressed as offsets into each orbit's sunlit arc.
struct Candidate {
    uint32_t offset = 0;
    uint16_t duration = 0;
    uint8_t value = 0;
};

inline constexpr std::array<Candidate, 6> kOnboardTargets{{
    {200, 400, 4},
    {700, 500, 6},
    {1700, 600, 7},
    {2300, 400, 3},
    {2700, 500, 5},
    {3100, 300, 2},
}};

constexpr bool overlaps(const Candidate& a, const Candidate& b) {
    return a.offset < b.offset + b.duration && b.offset < a.offset + a.duration;
}

// Onboard fallback scheduler: highest value per joule first, skipping
// anything that overlaps an earlier pick or exceeds the energy budget.
// Not optimal (this is a knapsack with conflicts); it is cheap, bounded and
// deterministic, which is what flight software needs when the ground is gone.
// Writes chosen indices into `out` and returns how many were chosen.
std::size_t greedy_schedule(std::span<const Candidate> candidates, double budget_j, double payload_w,
                            std::span<uint8_t> out);

}  // namespace bdio
