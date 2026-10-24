#pragma once

#include "types.h"

evt21_type get_evt21_word_type(evt21_word word);

timestamp_us get_time_stamp(evt21_word time_high_word, evt21_word time_low_word);

