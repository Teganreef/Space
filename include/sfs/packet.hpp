#pragma once

#include <array>
#include <cstdint>

namespace sfs {

// Uplink command frame. Fixed 18 byte big-endian layout:
//
//   0  sync      u16  0xEB90
//   2  seq       u16  monotonically increasing per command
//   4  send_tick u32  ground time the frame was sent
//   8  type      u8
//   9  start     u32  payload window start tick
//  13  duration  u16  payload window length [ticks]
//  15  value     u8   science value of the window
//  16  crc       u16  CRC-16/CCITT-FALSE over bytes 0..15
inline constexpr std::size_t kFrameSize = 18;
inline constexpr uint16_t kSyncWord = 0xEB90;

using Frame = std::array<uint8_t, kFrameSize>;

enum class CommandType : uint8_t { AddPayloadWindow = 0x01 };

struct Command {
    uint16_t seq = 0;
    uint32_t send_tick = 0;
    CommandType type = CommandType::AddPayloadWindow;
    uint32_t start = 0;
    uint16_t duration = 0;
    uint8_t value = 0;
};

enum class DecodeStatus : uint8_t { Ok, BadSync, BadCrc, BadType };

Frame encode(const Command& cmd);

// Full validation: sync word, CRC, known command type.
DecodeStatus decode(const Frame& frame, Command& out);

// Parses fields without checking the CRC or type. Exists so the naive
// controller can model software that trusts the link layer.
DecodeStatus decode_unchecked(const Frame& frame, Command& out);

}  // namespace sfs
