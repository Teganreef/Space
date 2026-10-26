#include "sfs/scheduler.hpp"

#include <algorithm>

namespace sfs {

std::size_t greedy_schedule(std::span<const Candidate> candidates, double budget_j, double payload_w,
                            std::span<uint8_t> out) {
    constexpr std::size_t kMax = 16;
    const std::size_t n = std::min({candidates.size(), kMax, out.size()});

    std::array<uint8_t, kMax> order{};
    for (std::size_t i = 0; i < n; ++i) {
        order[i] = static_cast<uint8_t>(i);
    }
    // Insertion sort by value density, stable on ties. n is tiny and this
    // keeps the routine allocation free.
    auto density = [&](uint8_t i) {
        return static_cast<double>(candidates[i].value) / (candidates[i].duration * payload_w);
    };
    for (std::size_t i = 1; i < n; ++i) {
        const uint8_t key = order[i];
        std::size_t j = i;
        while (j > 0 && density(order[j - 1]) < density(key)) {
            order[j] = order[j - 1];
            --j;
        }
        order[j] = key;
    }

    std::size_t chosen = 0;
    double spent = 0.0;
    for (std::size_t k = 0; k < n; ++k) {
        const Candidate& c = candidates[order[k]];
        const double cost = c.duration * payload_w;
        if (spent + cost > budget_j) {
            continue;
        }
        bool conflict = false;
        for (std::size_t m = 0; m < chosen; ++m) {
            conflict = conflict || overlaps(c, candidates[out[m]]);
        }
        if (!conflict) {
            out[chosen++] = order[k];
            spent += cost;
        }
    }
    return chosen;
}

}  // namespace sfs
