#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

namespace bdio {

// CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF), the variant used by CCSDS.
namespace detail {
constexpr std::array<uint16_t, 256> make_crc_table() {
    std::array<uint16_t, 256> table{};
    for (uint32_t i = 0; i < 256; ++i) {
        uint16_t crc = static_cast<uint16_t>(i << 8);
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x8000U) ? static_cast<uint16_t>((crc << 1) ^ 0x1021U)
                                  : static_cast<uint16_t>(crc << 1);
        }
        table[i] = crc;
    }
    return table;
}
inline constexpr auto kCrcTable = make_crc_table();
}  // namespace detail

constexpr uint16_t crc16(std::span<const uint8_t> data, uint16_t crc = 0xFFFF) {
    for (const uint8_t byte : data) {
        crc = static_cast<uint16_t>((crc << 8) ^ detail::kCrcTable[((crc >> 8) ^ byte) & 0xFFU]);
    }
    return crc;
}

inline uint16_t crc16(std::span<const std::byte> data, uint16_t crc = 0xFFFF) {
    return crc16(std::span<const uint8_t>(reinterpret_cast<const uint8_t*>(data.data()), data.size()), crc);
}

}  // namespace bdio
