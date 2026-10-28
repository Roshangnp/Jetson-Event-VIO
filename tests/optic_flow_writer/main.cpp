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
#include "writeHistograms.h"

// ~2.4 MB and ~4.9 MB, too big for the stack
static histograms hist;
static optic_flow flow;

// log every Nth flow_field and the histogram pair it came from: one flow_field is ~1.2 MB, so
// logging every window (~390/s) would be ~470 MB/s
constexpr size_t LOG_EVERY_N = 20;

// the pipeline runs this long without logging before recording starts, so the startup burst and
// blackout (~250 ms) and the first GPU call (CUDA setup, ~70 ms) are over
constexpr auto WARMUP_TIME = std::chrono::seconds(3);

// camera buffers recorded once the warm-up is over
constexpr int RECORD_BUFFER_COUNT = 100;

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
        writeFlows flow_writer;
        writeHistograms hist_writer;
        flow_writer.initialize("./flow_recording.bin");
        hist_writer.initialize("./histogram_recording.bin");

        streamer.initialize(argv[1], argv[2]);

        streamer.start();
        flow_writer.start();
        hist_writer.start();

        std::cout << "warming up for " << std::chrono::duration<double>(WARMUP_TIME).count() << " s" << std::endl;
        const auto warmup_end = std::chrono::steady_clock::now() + WARMUP_TIME;
        bool recording = false;
        int recorded_buffers = 0;
        size_t warmup_flow_count = 0;
        size_t flow_count = 0;
        double flow_seconds = 0.0;
        while (recorded_buffers < RECORD_BUFFER_COUNT) {
            if (!recording && std::chrono::steady_clock::now() >= warmup_end) {
                recording = true;
                std::cout << "logging started" << std::endl;
            }
            if (recording) {
                // a timed-out getBuffer() counts too, like histogram_buffer_writer
                ++recorded_buffers;
            }

            evt21_buffer buffer = streamer.getBuffer();
            if (buffer == nullptr) {
                // no camera buffer within the timeout, try again
                continue;
            }
            size_t word_count = streamer.wordCount();
            hist.updateHistograms(buffer,word_count);
            streamer.releaseBuffer();

            // one camera buffer can complete several windows, so compute flow for every
            // pair in order until none are left. Flow also runs during the warm-up, so the
            // recorded timings are steady state.
            for (histograms::histogram_buffer_pair hist_pair = hist.dequeueNextHistogramPair();
                 hist_pair[0] != nullptr;
                 hist_pair = hist.dequeueNextHistogramPair()) {
                const auto begin = std::chrono::steady_clock::now();
                const flow_field& flow_field_ref = use_gpu ? flow.run_LK_method_GPU(hist_pair)
                                                           : flow.run_LK_method_CPU(hist_pair);
                const double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - begin).count();

                if (!recording) {
                    ++warmup_flow_count;
                } else {
                    flow_seconds += seconds;
                    if (flow_count % LOG_EVERY_N == 0) {
                        flow_writer.tryWrite(flow_field_ref, flow.getTimeHigh());
                        // the pair this flow came from, older first
                        hist_writer.tryWrite(*hist_pair[1]);
                        hist_writer.tryWrite(*hist_pair[0]);
                    }
                    ++flow_count;
                }
                hist.queueHistogramPair();
            }
        }

        flow_writer.stop();
        hist_writer.stop();
        flow.stop();
        std::cout << (use_gpu ? "GPU" : "CPU") << " flows " << flow_count
                  << " (mean " << (flow_count ? flow_seconds / flow_count * 1e6 : 0.0) << " us)"
                  << " after " << warmup_flow_count << " warm-up flows"
                  << ", logging every " << LOG_EVERY_N
                  << ", flows written " << flow_writer.writtenCount()
                  << " dropped " << flow_writer.droppedCount()
                  << ", histograms written " << hist_writer.writtenCount()
                  << " dropped " << hist_writer.droppedCount()
                  << ", bad words " << hist.getBadWordCount() << '\n';
    } catch (const std::exception& error) {
        flow.stop();
        std::cerr << error.what() << '\n';
        return 1;
    }
    return 0;
}
