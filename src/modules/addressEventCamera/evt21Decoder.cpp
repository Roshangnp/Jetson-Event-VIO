#include "evt21Decoder.h"

// EVT2.1 64-bit word layout:
//   [63:60] type
//   EVT_NEG / EVT_POS: [59:54] timestamp LSBs, [53:43] x (aligned on 32),
//                      [42:32] y, [31:0] valid mask for pixels x..x+31
//   EVT_TIME_HIGH:     [59:32] timestamp MSBs (bits 33..6 of the timestamp)

namespace {

constexpr unsigned TYPE_SHIFT = 60;
constexpr evt21_word TYPE_MASK = 0xF;

constexpr unsigned TIME_HIGH_SHIFT = 32;
constexpr evt21_word TIME_HIGH_MASK = 0xFFFFFFF;

constexpr unsigned TIME_LOW_SHIFT = 54;
constexpr evt21_word TIME_LOW_MASK = 0x3F;
constexpr unsigned TIME_LOW_BITS = 6;

} // namespace

evt21_type get_evt21_word_type(evt21_word word) {
    return static_cast<evt21_type>((word >> TYPE_SHIFT) & TYPE_MASK);
}

timestamp_us get_time_stamp(evt21_word time_high_word, evt21_word time_low_word) {
    const evt21_word high = (time_high_word >> TIME_HIGH_SHIFT) & TIME_HIGH_MASK;
    const evt21_word low = (time_low_word >> TIME_LOW_SHIFT) & TIME_LOW_MASK;
    // The full timestamp is 34 bits; timestamp_us keeps the low 32 and wraps
    // roughly every 71 minutes.
    return static_cast<timestamp_us>((high << TIME_LOW_BITS) | low);
}
