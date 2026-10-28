#include "writeFlows.h"

#include <cerrno>
#include <fcntl.h>
#include <stdexcept>
#include <system_error>
#include <unistd.h>

namespace {

// Throws std::system_error carrying the current errno.
[[noreturn]] void fail(const char* operation) {
    throw std::system_error(errno, std::generic_category(), operation);
}

// write() until every byte is written, retrying partial writes and EINTR.
void writeAll(int fd, const void* data, size_t bytes) {
    const char* next = static_cast<const char*>(data);
    while (bytes > 0) {
        const ssize_t result = ::write(fd, next, bytes);
        if (result == -1) {
            if (errno == EINTR) {
                continue;
            }
            fail("write flow file");
        }
        next += result;
        bytes -= static_cast<size_t>(result);
    }
}

} // namespace

writeFlows::~writeFlows() {
    stop();
}

void writeFlows::initialize(const std::string& path) {
    // Calling initialize() twice must not leak the previous file or thread.
    stop();

    fd_ = ::open(path.c_str(), O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0644);
    if (fd_ == -1) {
        fail("open flow file");
    }

    // Allocated once and kept across stop() / initialize().
    if (!slots_) {
        slots_ = std::make_unique<flow_record[]>(SLOT_COUNT);
    }

    head_ = 0;
    tail_ = 0;
    filled_ = 0;
    stopping_ = false;
    writer_error_ = nullptr;
    written_count_ = 0;
    dropped_count_ = 0;
}

void writeFlows::start() {
    if (fd_ == -1) {
        throw std::logic_error("initialize the flow file before starting it");
    }
    if (writer_.joinable()) {
        throw std::logic_error("flow writer is already started");
    }
    writer_ = std::thread(&writeFlows::writerLoop, this);
}

void writeFlows::stop() noexcept {
    if (writer_.joinable()) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stopping_ = true;
        }
        slot_filled_.notify_one();
        // The writer thread drains the ring before it exits.
        writer_.join();
    }

    if (fd_ != -1) {
        (void)::fdatasync(fd_);
        (void)::close(fd_);
        fd_ = -1;
    }
}

bool writeFlows::tryWrite(const flow_field& flow, const size_t& time_high) {
    if (!writer_.joinable()) {
        throw std::logic_error("flow writer is not started");
    }

    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (writer_error_) {
            std::rethrow_exception(writer_error_);
        }
        if (filled_ == SLOT_COUNT) {
            ++dropped_count_;
            return false;
        }
    }

    // Copy outside the lock so the writer thread is never stalled by it. Slot
    // head_ is not filled, so the writer thread will not read it until it is
    // published below.
    flow_record& slot = slots_[head_];
    slot.time_high = time_high;
    slot.flow = flow;

    {
        std::lock_guard<std::mutex> lock(mutex_);
        head_ = (head_ + 1) % SLOT_COUNT;
        ++filled_;
    }
    slot_filled_.notify_one();
    return true;
}

void writeFlows::writerLoop() {
    try {
        for (;;) {
            size_t index;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                slot_filled_.wait(lock, [this] { return filled_ > 0 || stopping_; });
                if (filled_ == 0) {
                    // Stopping and the ring is drained.
                    return;
                }
                index = tail_;
            }

            // Write outside the lock so tryWrite() never waits on the disk.
            writeAll(fd_, &slots_[index], sizeof(flow_record));

            {
                std::lock_guard<std::mutex> lock(mutex_);
                tail_ = (tail_ + 1) % SLOT_COUNT;
                --filled_;
            }
            ++written_count_;
        }
    } catch (...) {
        // Hand the error to the pipeline thread; tryWrite() rethrows it.
        std::lock_guard<std::mutex> lock(mutex_);
        writer_error_ = std::current_exception();
    }
}
