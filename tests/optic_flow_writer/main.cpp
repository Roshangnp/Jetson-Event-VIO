#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <exception>
#include <iostream>
#include <string>

#include "types.h"
#include "evt21Streamer.h"
#include "histograms.h"
#include "histogram_buffer.h"
#include "optic_flow.h"
#include "writeFlows.h"

// ~2.4 MB and ~4.9 MB, too big for the stack
static histograms hist;
static optic_flow flow;

// log every Nth flow_field: one is ~1.2 MB, so logging every window (~390/s) would be ~470 MB/s
constexpr size_t LOG_EVERY_N = 20;

int main(int argc, char* argv[]){
    bool use_gpu = true;
    if (argc == 4 && std::strcmp(argv[3], "cpu") == 0) {
        use_gpu = false;
    } else if (argc != 3 && !(argc == 4 && std::strcmp(argv[3], "gpu") == 0)) {
        std::cerr << "usage: " << argv[0] << " <video-device> <sensor-device> [gpu|cpu]\n";
        return 1;
    }

    try {
        Evt21Streamer streamer;
        writeFlows writer;
        writer.initialize("./flow_recording.bin");

        streamer.initialize(argv[1], argv[2]);

        streamer.start();
        writer.start();
        size_t flow_count = 0;
        double flow_seconds = 0.0;
        for (int i = 0; i<100; i++){
            evt21_buffer buffer = streamer.getBuffer();
            if (buffer == nullptr) {
                // no camera buffer within the timeout, try again
                continue;
            }
            size_t word_count = streamer.wordCount();
            hist.updateHistograms(buffer,word_count);
            streamer.releaseBuffer();

            // one camera buffer can complete several windows, so compute flow for every
            // pair in order until none are left
            for (histograms::histogram_buffer_pair hist_pair = hist.dequeueNextHistogramPair();
                 hist_pair[0] != nullptr;
                 hist_pair = hist.dequeueNextHistogramPair()) {
                const auto begin = std::chrono::steady_clock::now();
                const flow_field& flow_field_ref = use_gpu ? flow.run_LK_method_GPU(hist_pair)
                                                           : flow.run_LK_method_CPU(hist_pair);
                flow_seconds += std::chrono::duration<double>(std::chrono::steady_clock::now() - begin).count();
                if (flow_count % LOG_EVERY_N == 0) {
                    writer.tryWrite(flow_field_ref, flow.getTimeHigh());
                }
                ++flow_count;
                hist.queueHistogramPair();
            }
        }

        writer.stop();
        flow.stop();
        std::cout << (use_gpu ? "GPU" : "CPU") << " flows " << flow_count
                  << " (mean " << (flow_count ? flow_seconds / flow_count * 1e6 : 0.0) << " us)"
                  << ", logging every " << LOG_EVERY_N
                  << ", written " << writer.writtenCount()
                  << ", dropped " << writer.droppedCount()
                  << ", bad words " << hist.getBadWordCount() << '\n';
    } catch (const std::exception& error) {
        flow.stop();
        std::cerr << error.what() << '\n';
        return 1;
    }
    return 0;
}
