#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <string>

#include "types.h"
#include "evt21Streamer.h"
#include "histograms.h"
#include "histogram_buffer.h"
#include "writeHistograms.h"

// ~1 MB, too big for the stack
static histograms hist;

int main(int argc, char* argv[]){
    if (argc != 3) {
        std::cerr << "usage: " << argv[0] << " <video-device> <sensor-device>\n";
        return 1;
    }

    try {
        Evt21Streamer streamer;
        writeHistograms writer;
        writer.initialize("./histogram_recording.bin");

        streamer.initialize(argv[1], argv[2]);

        streamer.start();
        writer.start();
        for (int i = 0; i<100; i++){
            evt21_buffer buffer = streamer.getBuffer();
            if (buffer == nullptr) {
                // no camera buffer within the timeout, try again
                continue;
            }
            size_t word_count = streamer.wordCount();
            hist.updateHistograms(buffer,word_count);
            streamer.releaseBuffer();

            histograms::histogram_buffer_pair hist_pair = hist.dequeueHistogramPair();
            if (hist_pair[0] == nullptr) {
                // no new complete pair yet
                continue;
            }

            writer.tryWrite(*hist_pair[0]);
            hist.queueHistogramPair();
        }

        writer.stop();
        std::cout << "written " << writer.writtenCount()
                  << ", dropped " << writer.droppedCount()
                  << ", bad words " << hist.getBadWordCount() << '\n';
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
    return 0;
}
