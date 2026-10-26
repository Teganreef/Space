#include <gtest/gtest.h>

#include <bit>

#include "sfs/hardened_controller.hpp"
#include "sfs/tmr.hpp"

using namespace sfs;

TEST(Tmr, CorrectsAnySingleBitInAnyCopy) {
    for (int copy = 0; copy < 3; ++copy) {
        for (int bit = 0; bit < 16; ++bit) {
            Tmr<uint16_t> t;
            t.set(0xBEEF);
            uint16_t& target = copy == 0 ? t.a : (copy == 1 ? t.b : t.c);
            target = static_cast<uint16_t>(target ^ (1U << bit));
            bool corrected = false;
            EXPECT_EQ(t.vote(&corrected), 0xBEEF);
            EXPECT_TRUE(corrected);
            EXPECT_EQ(t.a, 0xBEEF);
            EXPECT_EQ(t.b, 0xBEEF);
            EXPECT_EQ(t.c, 0xBEEF);
        }
    }
}

TEST(Tmr, CorrectsDifferentBitsInTwoCopies) {
    Tmr<uint8_t> t;
    t.set(0x3C);
    t.a ^= 0x01;
    t.b ^= 0x80;
    EXPECT_EQ(t.vote(), 0x3C);
}

TEST(Tmr, SameBitInTwoCopiesIsTheKnownLimit) {
    Tmr<uint8_t> t;
    t.set(0x3C);
    t.a ^= 0x01;
    t.b ^= 0x01;
    EXPECT_EQ(t.vote(), 0x3D);
}

TEST(Tmr, CleanValueReportsNoCorrection) {
    Tmr<uint32_t> t;
    t.set(123);
    bool corrected = true;
    EXPECT_EQ(t.vote(&corrected), 123U);
    EXPECT_FALSE(corrected);
}

TEST(ModeCodes, MinimumHammingDistanceIsFour) {
    constexpr std::array<uint8_t, 5> codes{kModeBoot, kModeNominal, kModeDegraded, kModeSafe, kModeRecovery};
    for (std::size_t i = 0; i < codes.size(); ++i) {
        for (std::size_t j = i + 1; j < codes.size(); ++j) {
            EXPECT_GE(std::popcount(static_cast<unsigned>(codes[i] ^ codes[j])), 4) << i << "," << j;
        }
    }
}

TEST(ModeCodes, NoSingleFlipProducesAnotherValidMode) {
    constexpr std::array<uint8_t, 5> codes{kModeBoot, kModeNominal, kModeDegraded, kModeSafe, kModeRecovery};
    for (const uint8_t c : codes) {
        for (int bit = 0; bit < 8; ++bit) {
            EXPECT_EQ(decode_hardened_mode(static_cast<uint8_t>(c ^ (1U << bit))), Mode::Invalid);
        }
    }
}
