#pragma once

#include <cmath>
#include <cstdint>

namespace bdio {

// xoshiro256** seeded through splitmix64. Implemented here instead of using
// <random> distributions because those are not guaranteed to produce the same
// sequence across standard library implementations, and every fault campaign
// has to be reproducible from its seed alone.
class Rng {
public:
    explicit Rng(uint64_t seed) {
        uint64_t x = seed;
        for (auto& word : s_) {
            word = splitmix(x);
        }
    }

    uint64_t next() {
        const uint64_t result = rotl(s_[1] * 5, 7) * 9;
        const uint64_t t = s_[1] << 17;
        s_[2] ^= s_[0];
        s_[3] ^= s_[1];
        s_[1] ^= s_[2];
        s_[0] ^= s_[3];
        s_[2] ^= t;
        s_[3] = rotl(s_[3], 45);
        return result;
    }

    // Uniform in [0, 1).
    double uniform() { return static_cast<double>(next() >> 11) * 0x1.0p-53; }

    double uniform(double lo, double hi) { return lo + (hi - lo) * uniform(); }

    // Uniform integer in [lo, hi].
    uint32_t range(uint32_t lo, uint32_t hi) {
        return lo + static_cast<uint32_t>(uniform() * static_cast<double>(hi - lo + 1));
    }

    bool chance(double p) { return uniform() < p; }

    double normal(double sigma) {
        // Box-Muller; one value per call keeps the stream simple to reason about.
        double u1 = uniform();
        if (u1 < 1e-300) {
            u1 = 1e-300;
        }
        const double u2 = uniform();
        return sigma * std::sqrt(-2.0 * std::log(u1)) * std::cos(2.0 * 3.14159265358979323846 * u2);
    }

private:
    static uint64_t rotl(uint64_t x, int k) { return (x << k) | (x >> (64 - k)); }

    static uint64_t splitmix(uint64_t& x) {
        uint64_t z = (x += 0x9E3779B97F4A7C15ULL);
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
        return z ^ (z >> 31);
    }

    uint64_t s_[4]{};
};

}  // namespace bdio
