#pragma once

#include <cstddef>
#include <cstdint>
#include "types.h"

class histogram_buffer{
    public:
    histogram_buffer();
    ~histogram_buffer();

    bool updateHistogram(evt21_word word);
    histogram_array* getHistogram();
    bool isReady();
    void resetHistogram();


    private:
    evt21_word timeHigh_start_;
    uint8_t time_wdith_;
    histogram_array hist_;
    bool ready_;
};