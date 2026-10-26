#include "sfs/packet.hpp"

#include <span>

#include "sfs/crc16.hpp"

namespace sfs {

namespace {

void put16(Frame& f, std::size_t at, uint16_t v) {
    f[at] = static_cast<uint8_t>(v >> 8);
    f[at + 1] = static_cast<uint8_t>(v & 0xFFU);
}

void put32(Frame& f, std::size_t at, uint32_t v) {
    f[at] = static_cast<uint8_t>(v >> 24);
    f[at + 1] = static_cast<uint8_t>((v >> 16) & 0xFFU);
    f[at + 2] = static_cast<uint8_t>((v >> 8) & 0xFFU);
    f[at + 3] = static_cast<uint8_t>(v & 0xFFU);
}

uint16_t get16(const Frame& f, std::size_t at) {
    return static_cast<uint16_t>((f[at] << 8) | f[at + 1]);
}

uint32_t get32(const Frame& f, std::size_t at) {
    return (static_cast<uint32_t>(f[at]) << 24) | (static_cast<uint32_t>(f[at + 1]) << 16) |
           (static_cast<uint32_t>(f[at + 2]) << 8) | static_cast<uint32_t>(f[at + 3]);
}

void parse_fields(const Frame& f, Command& out) {
    out.seq = get16(f, 2);
    out.send_tick = get32(f, 4);
    out.type = static_cast<CommandType>(f[8]);
    out.start = get32(f, 9);
    out.duration = get16(f, 13);
    out.value = f[15];
}

}  // namespace

Frame encode(const Command& cmd) {
    Frame f{};
    put16(f, 0, kSyncWord);
    put16(f, 2, cmd.seq);
    put32(f, 4, cmd.send_tick);
    f[8] = static_cast<uint8_t>(cmd.type);
    put32(f, 9, cmd.start);
    put16(f, 13, cmd.duration);
    f[15] = cmd.value;
    put16(f, 16, crc16(std::span<const uint8_t>(f.data(), kFrameSize - 2)));
    return f;
}

DecodeStatus decode(const Frame& frame, Command& out) {
    if (get16(frame, 0) != kSyncWord) {
        return DecodeStatus::BadSync;
    }
    if (crc16(std::span<const uint8_t>(frame.data(), kFrameSize - 2)) != get16(frame, 16)) {
        return DecodeStatus::BadCrc;
    }
    if (frame[8] != static_cast<uint8_t>(CommandType::AddPayloadWindow)) {
        return DecodeStatus::BadType;
    }
    parse_fields(frame, out);
    return DecodeStatus::Ok;
}

DecodeStatus decode_unchecked(const Frame& frame, Command& out) {
    parse_fields(frame, out);
    return DecodeStatus::Ok;
}

}  // namespace sfs
