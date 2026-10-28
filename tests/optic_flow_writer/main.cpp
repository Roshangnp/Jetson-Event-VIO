#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <fstream>
#include <iostream>
#include <string>
#include <unistd.h>
#include <vector>

#include "types.h"
#include "evt21Decoder.h"
#include "evt21Streamer.h"
#include "histograms.h"
#include "histogram_buffer.h"
#include "optic_flow.h"
#include "writeFlows.h"
#include "writeHistograms.h"

// ~2.4 MB and ~6.7 MB, too big for the stack
static histograms hist;
static optic_flow flow;

// log every Nth flow_field and the histogram pair it came from: one flow_field is ~1.15 MB, so
// logging every window (~390/s) would be ~450 MB/s
constexpr size_t LOG_EVERY_N = 20;

// the pipeline runs this long without logging before recording starts, so the startup burst and
// blackout (~250 ms) and the first GPU call (CUDA setup, ~70 ms) are over
constexpr auto WARMUP_TIME = std::chrono::seconds(3);

// camera buffers recorded once the warm-up is over
constexpr int RECORD_BUFFER_COUNT = 100;

namespace {

using steady = std::chrono::steady_clock;

// Runs a shell command and returns its output, or "" if it can't be run.
std::string commandOutput(const std::string& command)
{
    std::string output;
    if (FILE* pipe = ::popen(command.c_str(), "r")) {
        char chunk[256];
        while (std::fgets(chunk, sizeof chunk, pipe) != nullptr) {
            output += chunk;
        }
        ::pclose(pipe);
    }
    return output;
}

// nvpmodel ID of the power mode called name in /etc/nvpmodel.conf, or -1 if there is none
int powerModeId(const std::string& name)
{
    // lines look like "NVPM VERB: POWER_MODEL: ID=0 NAME=MAXN_SUPER"
    const std::string modes = commandOutput("/usr/sbin/nvpmodel -p --verbose 2>/dev/null");
    const std::string key = " NAME=" + name;
    for (size_t at = modes.find(key); at != std::string::npos; at = modes.find(key, at + 1)) {
        const size_t end = at + key.size();
        if (end != modes.size() && modes[end] != '\n' && modes[end] != ' ') {
            continue;   // a longer name that starts with name
        }
        const size_t id = modes.rfind("ID=", at);
        if (id != std::string::npos) {
            return std::atoi(modes.c_str() + id + 3);
        }
    }
    return -1;
}

// current nvpmodel power mode ID, or -1 if it can't be read. `nvpmodel -q` prints the name, then the ID.
int currentPowerModeId()
{
    const std::string query = commandOutput("/usr/sbin/nvpmodel -q 2>/dev/null");
    const size_t last_line = query.find_last_not_of("\n");
    if (last_line == std::string::npos) {
        return -1;
    }
    const size_t start = query.find_last_of('\n', last_line);
    return std::atoi(query.c_str() + (start == std::string::npos ? 0 : start + 1));
}

// For the run: switches the power mode to MAXN_SUPER (no caps on CPU / GPU clocks) and pins the CPU, GPU
// and memory clocks at that mode's maximum (jetson_clocks), so timings don't depend on the power mode or
// the governor. Restores the previous mode and clock settings when destroyed. MAXN_SUPER and the default
// 40W mode have the same online cores and power gating on this board, so the switch needs no reboot.
// Needs root: run the test with sudo, or allow both for this user with a sudoers rule
// (jetson ALL=(root) NOPASSWD: /usr/bin/jetson_clocks, /usr/sbin/nvpmodel). Without either the test
// runs on the current mode and the governor's clocks. Killing the test (Ctrl-C) skips the restore, and
// the power mode persists across reboots: `sudo nvpmodel -m <previous ID>` and a reboot undo it.
class max_performance {
public:
    static constexpr const char* POWER_MODE = "MAXN_SUPER";

    max_performance()
        // one file per run: jetson_clocks --store refuses to overwrite an existing file
        : settings_file_("/tmp/optic_flow_writer_clocks_" + std::to_string(::getpid()) + ".conf")
    {
        // the clock settings of the current mode, restored after switching back to it
        const std::string store = "sudo -n /usr/bin/jetson_clocks --store " + settings_file_ + " > /dev/null 2>&1";
        if (std::system(store.c_str()) != 0) {
            return;
        }
        stored_ = true;

        previous_mode_ = currentPowerModeId();
        const int target_mode = powerModeId(POWER_MODE);
        if (target_mode >= 0 && previous_mode_ >= 0 && target_mode != previous_mode_) {
            // < /dev/null: nvpmodel asks before a mode change that needs a reboot; this answers no
            const std::string set_mode = "sudo -n /usr/sbin/nvpmodel -m " + std::to_string(target_mode) + " < /dev/null > /dev/null 2>&1";
            switched_ = std::system(set_mode.c_str()) == 0 && currentPowerModeId() == target_mode;
        }
        in_mode_ = target_mode >= 0 && currentPowerModeId() == target_mode;
        pinned_ = std::system("sudo -n /usr/bin/jetson_clocks > /dev/null 2>&1") == 0;
    }

    ~max_performance() { restore(); }

    max_performance(const max_performance&) = delete;
    max_performance& operator=(const max_performance&) = delete;

    void restore() noexcept
    {
        if (!stored_) {
            return;
        }
        stored_ = false;
        pinned_ = false;
        try {
            // results are ignored: nothing useful to do if restoring fails. The mode goes back first, so
            // the stored clock settings are restored in the mode they were taken in.
            if (switched_) {
                switched_ = false;
                const std::string set_mode = "sudo -n /usr/sbin/nvpmodel -m " + std::to_string(previous_mode_) + " < /dev/null > /dev/null 2>&1";
                if (std::system(set_mode.c_str()) != 0) {
                }
            }
            const std::string restore = "sudo -n /usr/bin/jetson_clocks --restore " + settings_file_ + " > /dev/null 2>&1";
            if (std::system(restore.c_str()) != 0) {
            }
            const std::string remove = "sudo -n rm -f " + settings_file_;
            if (std::system(remove.c_str()) != 0) {
            }
        } catch (...) {
            // nothing useful to do while restoring
        }
    }

    // the clocks are pinned (in whatever power mode the run ended up in)
    bool pinned() const noexcept { return pinned_; }
    // the run is in POWER_MODE, switched to or already in it
    bool inMode() const noexcept { return in_mode_; }

private:
    std::string settings_file_;
    int previous_mode_ = -1;
    bool stored_ = false;
    bool switched_ = false;
    bool in_mode_ = false;
    bool pinned_ = false;
};

// current frequency of CPU 0 in MHz, or 0 if it can't be read
long cpuMHz()
{
    std::ifstream file("/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq");
    long khz = 0;
    file >> khz;
    return khz / 1000;
}

double microsecondsSince(steady::time_point begin)
{
    return std::chrono::duration<double, std::micro>(steady::now() - begin).count();
}

// one TIME_HIGH tick is 64 us
constexpr double TICK_MS = 0.064;

// timings and contents of one recorded camera buffer
struct buffer_stats {
    double get_us = 0;       // getBuffer(): waiting for the camera plus DQBUF
    double hist_us = 0;      // updateHistograms() + releaseBuffer()
    double flow_us = 0;      // every flow computed from the windows this buffer completed
    double log_us = 0;       // writer tryWrite() copies
    size_t words = 0;
    size_t windows = 0;      // histogram pairs this buffer completed
    size_t advance_ticks = 0;  // TIME_HIGH advanced since the previous buffer: the camera time it covers
};

// Follows the TIME_HIGH words across buffers. A jump of more than one tick means TIME_HIGH words
// are missing from the stream: inside a buffer that is data lost before the driver, between
// buffers it can also be a dropped buffer (compare with droppedBufferCount()).
struct time_high_tracker {
    bool has_last = false;
    size_t last = 0;
    size_t jumps_within = 0;
    size_t jumps_between = 0;
    size_t missing_ticks = 0;

    // returns how far TIME_HIGH advanced from the previous buffer's last TIME_HIGH to this one's
    size_t scan(evt21_buffer buffer, size_t words)
    {
        const bool had_last = has_last;
        const size_t previous = last;
        bool first_in_buffer = true;
        for (size_t i = 0; i < words; ++i) {
            if (get_evt21_word_type(buffer[i]) != evt21_type::TIME_HIGH) {
                continue;
            }
            const size_t time_high = get_time_high(buffer[i]);
            if (has_last) {
                const size_t step = get_time_high_duration(last, time_high);
                if (step > 1) {
                    (first_in_buffer ? jumps_between : jumps_within) += 1;
                    missing_ticks += step - 1;
                }
            }
            has_last = true;
            last = time_high;
            first_in_buffer = false;
        }
        return had_last ? get_time_high_duration(previous, last) : 0;
    }
};

double percentile(std::vector<double> values, double p)
{
    if (values.empty()) {
        return 0.0;
    }
    std::sort(values.begin(), values.end());
    return values[static_cast<size_t>(p / 100.0 * static_cast<double>(values.size() - 1))];
}

double mean(const std::vector<double>& values)
{
    double sum = 0.0;
    for (double v : values) {
        sum += v;
    }
    return values.empty() ? 0.0 : sum / static_cast<double>(values.size());
}

void printTimes(const char* name, const std::vector<double>& us)
{
    std::cout << "  " << name << ": mean " << mean(us) << ", p50 " << percentile(us, 50)
              << ", p95 " << percentile(us, 95) << ", max " << percentile(us, 100) << " us\n";
}

// per-buffer report for the given buffers
void printBuffers(const char* title, const std::vector<buffer_stats>& buffers)
{
    std::vector<double> get, hist_us, flow_us, log_us, busy, covered, words, windows;
    for (const buffer_stats& b : buffers) {
        get.push_back(b.get_us);
        hist_us.push_back(b.hist_us);
        flow_us.push_back(b.flow_us);
        log_us.push_back(b.log_us);
        busy.push_back(b.hist_us + b.flow_us + b.log_us);
        covered.push_back(static_cast<double>(b.advance_ticks) * TICK_MS * 1000.0);
        words.push_back(static_cast<double>(b.words));
        windows.push_back(static_cast<double>(b.windows));
    }
    std::cout << title << " (" << buffers.size() << " buffers)\n";
    if (buffers.empty()) {
        return;
    }
    std::cout << "  words per buffer: mean " << mean(words) << ", windows per buffer: mean " << mean(windows)
              << ", max " << percentile(windows, 100) << '\n';
    printTimes("camera time per buffer", covered);
    printTimes("waiting in getBuffer  ", get);
    printTimes("histograms + release  ", hist_us);
    printTimes("flow                  ", flow_us);
    printTimes("logging copies        ", log_us);
    printTimes("busy (all but waiting)", busy);
    size_t over = 0;
    for (size_t i = 0; i < buffers.size(); ++i) {
        over += busy[i] > covered[i] && covered[i] > 0;
    }
    std::cout << "  buffers whose processing took longer than the camera time they cover: " << over << '\n';
}

} // namespace

int main(int argc, char* argv[]){
    bool use_gpu = true;
    if (argc == 4 && std::strcmp(argv[3], "cpu") == 0) {
        use_gpu = false;
    } else if (argc != 3 && !(argc == 4 && std::strcmp(argv[3], "gpu") == 0)) {
        std::cerr << "usage: " << argv[0] << " <video-device> <sensor-device> [gpu|cpu]\n";
        return 1;
    }

    // restored when main returns, by exception or not
    max_performance performance;
    if (performance.pinned()) {
        std::cout << "power mode " << (performance.inMode() ? max_performance::POWER_MODE : "unchanged (could not switch to MAXN_SUPER)")
                  << ", clocks pinned at the mode's maximum (jetson_clocks), CPU at " << cpuMHz() << " MHz" << std::endl;
    } else {
        std::cout << "warning: could not switch power mode or pin clocks (nvpmodel / jetson_clocks need root: run with "
                     "sudo); running on the current mode and the governor's clocks, CPU now at "
                  << cpuMHz() << " MHz" << std::endl;
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
        const auto warmup_end = steady::now() + WARMUP_TIME;
        bool recording = false;
        int recorded_buffers = 0;
        size_t warmup_flow_count = 0;
        size_t flow_count = 0;
        double flow_seconds = 0.0;

        std::vector<buffer_stats> stats;
        stats.reserve(RECORD_BUFFER_COUNT);
        time_high_tracker time_high;
        uint64_t dropped_at_start = 0;
        steady::time_point record_begin;

        while (recorded_buffers < RECORD_BUFFER_COUNT) {
            if (!recording && steady::now() >= warmup_end) {
                recording = true;
                dropped_at_start = streamer.droppedBufferCount();
                time_high = time_high_tracker{};
                record_begin = steady::now();
                std::cout << "logging started" << std::endl;
            }
            if (recording) {
                // a timed-out getBuffer() counts too, like histogram_buffer_writer
                ++recorded_buffers;
            }

            buffer_stats b;
            auto begin = steady::now();
            evt21_buffer buffer = streamer.getBuffer();
            b.get_us = microsecondsSince(begin);
            if (buffer == nullptr) {
                // no camera buffer within the timeout, try again
                continue;
            }
            size_t word_count = streamer.wordCount();
            b.words = word_count;
            // not timed: only the instrumentation needs it
            b.advance_ticks = time_high.scan(buffer, word_count);

            begin = steady::now();
            hist.updateHistograms(buffer,word_count);
            streamer.releaseBuffer();
            b.hist_us = microsecondsSince(begin);

            // one camera buffer can complete several windows, so compute flow for every
            // pair in order until none are left. Flow also runs during the warm-up, so the
            // recorded timings are steady state.
            for (histograms::histogram_buffer_pair hist_pair = hist.dequeueNextHistogramPair();
                 hist_pair[0] != nullptr;
                 hist_pair = hist.dequeueNextHistogramPair()) {
                begin = steady::now();
                const flow_field& flow_field_ref = use_gpu ? flow.run_LK_method_GPU(hist_pair)
                                                           : flow.run_LK_method_CPU(hist_pair);
                const double flow_us = microsecondsSince(begin);
                ++b.windows;

                if (!recording) {
                    ++warmup_flow_count;
                } else {
                    flow_seconds += flow_us * 1e-6;
                    b.flow_us += flow_us;
                    if (flow_count % LOG_EVERY_N == 0) {
                        begin = steady::now();
                        flow_writer.tryWrite(flow_field_ref, flow.getTimeHigh());
                        // the pair this flow came from, older first
                        hist_writer.tryWrite(*hist_pair[1]);
                        hist_writer.tryWrite(*hist_pair[0]);
                        b.log_us += microsecondsSince(begin);
                    }
                    ++flow_count;
                }
                hist.queueHistogramPair();
            }
            if (recording) {
                stats.push_back(b);
            }
        }
        const double record_ms = std::chrono::duration<double, std::milli>(steady::now() - record_begin).count();
        const uint64_t dropped = streamer.droppedBufferCount() - dropped_at_start;

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

        // instrumentation
        size_t camera_ticks = 0;
        for (const buffer_stats& b : stats) {
            camera_ticks += b.advance_ticks;
        }
        const double camera_ms = static_cast<double>(camera_ticks) * TICK_MS;
        std::cout << "\nrecording: " << record_ms << " ms wall clock, " << camera_ms << " ms of camera time (TIME_HIGH), "
                  << flow_count << " windows processed = " << static_cast<double>(flow_count) * 40 * TICK_MS
                  << " ms of camera time\n";
        std::cout << "driver dropped buffers (V4L2 sequence gaps): " << dropped << '\n';
        std::cout << "TIME_HIGH jumps: " << time_high.jumps_between << " between buffers, " << time_high.jumps_within
                  << " inside buffers, " << time_high.missing_ticks << " ticks missing ("
                  << static_cast<double>(time_high.missing_ticks) * TICK_MS << " ms, "
                  << (camera_ticks ? 100.0 * static_cast<double>(time_high.missing_ticks) / static_cast<double>(camera_ticks) : 0.0)
                  << "% of camera time)\n";
        printBuffers("all buffers", stats);
        // busy buffers: they cover less camera time than about 1.2 windows (3 ms)
        std::vector<buffer_stats> busy;
        for (const buffer_stats& b : stats) {
            if (b.advance_ticks > 0 && static_cast<double>(b.advance_ticks) * TICK_MS < 3.0) {
                busy.push_back(b);
            }
        }
        printBuffers("high-event-rate buffers (< 3 ms of camera time each)", busy);
    } catch (const std::exception& error) {
        flow.stop();
        std::cerr << error.what() << '\n';
        return 1;
    }
    return 0;
}
