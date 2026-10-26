#include <gtest/gtest.h>

#include <string_view>

#include "bdio/crc16.hpp"
#include "bdio/packet.hpp"

using namespace bdio;

TEST(Crc16, MatchesCcittFalseCheckValue) {
    constexpr std::string_view check = "123456789";
    const auto* p = reinterpret_cast<const uint8_t*>(check.data());
    EXPECT_EQ(crc16(std::span<const uint8_t>(p, check.size())), 0x29B1);
}

TEST(Packet, RoundTrip) {
    const Command in{0x1234, 98765, CommandType::AddPayloadWindow, 123456, 600, 7};
    Command out;
    ASSERT_EQ(decode(encode(in), out), DecodeStatus::Ok);
    EXPECT_EQ(out.seq, in.seq);
    EXPECT_EQ(out.send_tick, in.send_tick);
    EXPECT_EQ(out.start, in.start);
    EXPECT_EQ(out.duration, in.duration);
    EXPECT_EQ(out.value, in.value);
}

TEST(Packet, EverySingleBitErrorIsRejected) {
    const Frame good = encode({42, 1000, CommandType::AddPayloadWindow, 5000, 400, 3});
    for (std::size_t bit = 0; bit < kFrameSize * 8; ++bit) {
        Frame bad = good;
        bad[bit / 8] = static_cast<uint8_t>(bad[bit / 8] ^ (1U << (bit % 8)));
        Command out;
        EXPECT_NE(decode(bad, out), DecodeStatus::Ok) << "bit " << bit;
    }
}

TEST(Packet, EveryDoubleBitErrorIsRejected) {
    // CRC-16/CCITT detects all 2-bit errors in frames this short.
    const Frame good = encode({7, 2000, CommandType::AddPayloadWindow, 9000, 500, 9});
    for (std::size_t a = 0; a < kFrameSize * 8; ++a) {
        for (std::size_t b = a + 1; b < kFrameSize * 8; ++b) {
            Frame bad = good;
            bad[a / 8] = static_cast<uint8_t>(bad[a / 8] ^ (1U << (a % 8)));
            bad[b / 8] = static_cast<uint8_t>(bad[b / 8] ^ (1U << (b % 8)));
            Command out;
            ASSERT_NE(decode(bad, out), DecodeStatus::Ok) << "bits " << a << "," << b;
        }
    }
}

TEST(Packet, UncheckedDecodeAcceptsGarbage) {
    Frame f = encode({1, 10, CommandType::AddPayloadWindow, 100, 300, 1});
    f[14] ^= 0xFF;  // corrupt duration
    Command out;
    EXPECT_EQ(decode_unchecked(f, out), DecodeStatus::Ok);
    EXPECT_NE(out.duration, 300);
}
