#pragma once

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "types.h"
#include "../addressEventCamera/histogram_buffer.h"

// Logs completed histograms to a binary file from a background thread.
//
// tryWrite() copies a histogram into a preallocated ring of slots and returns
// immediately, so the caller can queue the histogram pair straight away. The
// writer thread drains the ring to disk. Only the ring is shared between the
// threads, so the evt21Streamer -> histograms pipeline stays single-threaded.
//
// File layout: no file header, just back-to-back fixed-size histogram_records
// in native byte order (little-endian on Jetson).
class writeHistograms{
    public:
    static constexpr size_t SLOT_COUNT = 32;

    // One record on disk and one slot in the ring (204,808 bytes, no padding).
    struct histogram_record {
        // decoded TIME_HIGH the window began at (histogram_buffer::getTimeHighStart)
        uint64_t time_high_start;
        histogram_array hist;
    };

    writeHistograms() = default;
    ~writeHistograms();

    // non-copyable, owns a file, a thread and ~6.5 MB of slots
    writeHistograms(const writeHistograms&) = delete;
    writeHistograms& operator=(const writeHistograms&) = delete;

    // creates (or truncates) the file at path and allocates the ring once.
    // Calls stop() first, so it is safe to call twice. Throws std::system_error
    // if the file cannot be opened.
    void initialize(const std::string& path);

    // starts the writer thread. Throws std::logic_error if not initialized or
    // already started.
    void start();

    // writes every record already queued, joins the writer thread, syncs and
    // closes the file. Initialize again before restarting.
    void stop() noexcept;

    // copies buffer's histogram and time high start into a free slot for the writer
    // thread. Never blocks on the disk: returns false and counts a drop if every
    // slot is full. buffer must be READY or HELD (throws std::logic_error otherwise).
    // Rethrows the writer thread's error if a write to the file failed.
    bool tryWrite(histogram_buffer& buffer);

    // records written to the file so far
    size_t writtenCount() const noexcept { return written_count_.load(); }

    // records dropped because the ring was full
    size_t droppedCount() const noexcept { return dropped_count_; }

    private:
    // writer thread: waits for full slots and writes them to the file
    void writerLoop();

    int fd_ = -1;
    std::thread writer_;

    // the ring, allocated once in initialize()
    std::unique_ptr<histogram_record[]> slots_;

    // next slot tryWrite fills; only the pipeline thread touches it
    size_t head_ = 0;

    // everything below the mutex is guarded by it
    std::mutex mutex_;
    std::condition_variable slot_filled_;
    // next slot the writer thread writes
    size_t tail_ = 0;
    // slots filled and not yet written
    size_t filled_ = 0;
    // set by stop(); the writer thread drains the ring and exits
    bool stopping_ = false;
    // a write error from the writer thread, rethrown by tryWrite()
    std::exception_ptr writer_error_;

    // only the writer thread increments it; read from the pipeline thread
    std::atomic<size_t> written_count_{0};
    // only the pipeline thread touches it
    size_t dropped_count_ = 0;
};
