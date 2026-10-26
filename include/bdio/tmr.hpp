#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <type_traits>

namespace bdio {

// Software triple modular redundancy. Three copies of a value; reads go
// through a bitwise majority vote and the voted value is written back to all
// three copies (scrubbing), so a single upset never accumulates into two.
//
// Limitation: if the same bit is flipped in two copies between scrubs the vote
// returns the wrong value. Scrubbing every control cycle makes that window one
// tick wide.
template <typename T>
    requires std::is_trivially_copyable_v<T>
struct Tmr {
    T a{};
    T b{};
    T c{};

    void set(const T& v) { a = b = c = v; }

    // Returns the majority value. Sets *corrected if any copy disagreed.
    T vote(bool* corrected = nullptr) {
        std::array<uint8_t, sizeof(T)> ba{};
        std::array<uint8_t, sizeof(T)> bb{};
        std::array<uint8_t, sizeof(T)> bc{};
        std::memcpy(ba.data(), &a, sizeof(T));
        std::memcpy(bb.data(), &b, sizeof(T));
        std::memcpy(bc.data(), &c, sizeof(T));

        std::array<uint8_t, sizeof(T)> out{};
        bool differed = false;
        for (std::size_t i = 0; i < sizeof(T); ++i) {
            out[i] = static_cast<uint8_t>((ba[i] & bb[i]) | (bb[i] & bc[i]) | (ba[i] & bc[i]));
            differed = differed || ba[i] != out[i] || bb[i] != out[i] || bc[i] != out[i];
        }

        T result{};
        std::memcpy(&result, out.data(), sizeof(T));
        if (differed) {
            set(result);
        }
        if (corrected != nullptr) {
            *corrected = differed;
        }
        return result;
    }
};

}  // namespace bdio
