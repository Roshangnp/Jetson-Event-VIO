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

#include "flow_types.h"

// Logs flow fields to a binary file from a background thread. Works like writeHistograms.
//
// tryWrite() copies a flow_field into a preallocated ring of slots and returns
// immediately. The writer thread drains the ring to disk. Only the ring is shared
// between the threads, so the optic flow pipeline stays single-threaded.
//
// File layout: no file header, just back-to-back fixed-size flow_records in native
// byte order (little-endian on Jetson), including the compiler's padding (see the
// static_asserts below and the fileWriter README).
class writeFlows{
    public:
    // A flow_field is ~1.15 MB at a dense 11x11 grid (~6x a histogram record), so the ring is
    // smaller than writeHistograms' 32: 8 slots is ~9.2 MB.
    static constexpr size_t SLOT_COUNT = 8;

    // One record on disk and one slot in the ring.
    struct flow_record {
        // optic_flow::getTimeHigh() of the run that produced flow
        uint64_t time_high;
        flow_field flow;
    };

    writeFlows() = default;
    ~writeFlows();

    // non-copyable, owns a file, a thread and ~9.2 MB of slots
    writeFlows(const writeFlows&) = delete;
    writeFlows& operator=(const writeFlows&) = delete;

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

    // copies flow and time_high into a free slot for the writer thread. Never blocks
    // on the disk: returns false and counts a drop if every slot is full. Throws
    // std::logic_error if the writer isn't started. Rethrows the writer thread's
    // error if a write to the file failed.
    bool tryWrite(const flow_field& flow, const size_t& time_high);

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
    std::unique_ptr<flow_record[]> slots_;

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

// The on-disk layout is the in-memory layout, so a change to these types changes the file
// format. Update the fileWriter README's numpy dtype if any of these fail.
static_assert(sizeof(flow_patch) == 12, "flow_patch layout changed: update the flow file format");
static_assert(offsetof(writeFlows::flow_record, flow) == 8, "flow_record layout changed: update the flow file format");
static_assert(sizeof(writeFlows::flow_record) == 8 + 16 + flowConfig::GRID_POINT_NUM * 12,
              "flow_record size changed: update the flow file format");
