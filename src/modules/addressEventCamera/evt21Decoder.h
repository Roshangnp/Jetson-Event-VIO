#pragma once

#include <cstddef>
#include <cstdint>

#include "types.h"

evt21_type get_evt21_word_type(evt21_word word);

timestamp_us get_time_stamp(evt21_word time_high_word, evt21_word time_low_word);

// Decoded 28-bit TIME_HIGH value (timestamp bits 33..6); one tick is 64 us.
std::size_t get_time_high(evt21_word time_high_word);

// Ticks from start to current, both from get_time_high(). Handles the counter
// wrapping (about every 4.8 hours).
std::size_t get_time_high_duration(std::size_t start, std::size_t current);

// The following apply only to NEGATIVE / POSITIVE words.

// x of the first pixel in the word's group of 32 (always a multiple of 32).
std::size_t get_x(evt21_word word);

std::size_t get_y(evt21_word word);

// Bit i set means pixel x + i fired.
std::uint32_t get_valid_mask(evt21_word word);

// True if the word's 32-pixel group lies on the sensor (x aligned on 32,
// x..x+31 < WIDTH, y < HEIGHT). A false result means the word is corrupted.
bool is_valid_event(evt21_word word);
