// License: Apache 2.0. See LICENSE file in root directory.
// Low-overhead, opt-in V4L2 stage tracing for RealSense diagnostics.

#pragma once

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <limits>

#include <fcntl.h>
#include <sched.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/syscall.h>
#include <time.h>
#include <unistd.h>

namespace librealsense {
namespace platform {
namespace v4l2_diagnostic {

enum class stage : uint16_t
{
    select_return = 1,
    metadata_begin,
    metadata_end,
    video_dqbuf_begin,
    video_dqbuf_end,
    metadata_dqbuf_begin,
    metadata_dqbuf_end,
    callback_begin,
    callback_end,
    requeue_begin,
    requeue_end,
    sensor_timestamp_begin,
    sensor_timestamp_end,
    sensor_allocate_begin,
    sensor_allocate_end,
    sensor_copy_begin,
    sensor_copy_end,
    sensor_continue_begin,
    sensor_continue_end,
    sensor_invoke_begin,
    sensor_invoke_end,
    syncer_match_begin,
    syncer_match_end,
    syncer_emit_begin,
    syncer_emit_end,
};

struct file_header
{
    char magic[8];
    uint32_t version;
    uint32_t header_size;
    uint32_t event_size;
    uint32_t reserved;
    uint64_t capacity;
    uint64_t event_count;
    uint64_t dropped_count;
};

struct event
{
    uint64_t timestamp_ns;
    uint32_t tid;
    uint16_t cpu;
    uint16_t stage_id;
    int32_t fd;
    int32_t result;
    uint32_t sequence;
    uint32_t reserved;
};

static_assert(sizeof(event) == 32, "unexpected V4L2 diagnostic event size");

struct alignas(64) lane_counter
{
    uint64_t event_count;
    uint64_t dropped_count;
};

static_assert(sizeof(lane_counter) == 64, "diagnostic counters must not share cache lines");

class mapped_trace
{
public:
    mapped_trace()
    {
        auto path = std::getenv("RS_V4L2_DIAGNOSTIC_TRACE_FILE");
        if (!path || !*path)
            return;

        // The ring is split into per-CPU lanes to keep capture workers from
        // contending on a shared counter.  Eight million events gives each
        // lane two million slots on the four-core Raspberry Pi 5, enough for
        // a ten-minute, two-camera 30 FPS diagnostic run.
        uint64_t capacity = 8'000'000;
        if (auto text = std::getenv("RS_V4L2_DIAGNOSTIC_TRACE_CAPACITY"))
        {
            char * end = nullptr;
            errno = 0;
            auto parsed = std::strtoull(text, &end, 10);
            if (!errno && end != text && *end == '\0' && parsed > 0)
                capacity = parsed;
        }
        if (capacity > (std::numeric_limits<size_t>::max() - header_bytes) / sizeof(event))
            return;

        auto configured_cpus = ::sysconf(_SC_NPROCESSORS_CONF);
        _lane_count = static_cast<size_t>(std::max<long>(1, std::min<long>(64, configured_cpus)));
        _lane_count = std::min(_lane_count, static_cast<size_t>(capacity));
        _lane_capacity = static_cast<size_t>(capacity) / _lane_count;
        capacity = _lane_capacity * _lane_count;
        void * counters = nullptr;
        if (::posix_memalign(&counters, alignof(lane_counter), _lane_count * sizeof(lane_counter)))
            return;
        _lanes = static_cast<lane_counter *>(counters);
        std::memset(_lanes, 0, _lane_count * sizeof(lane_counter));

        _fd = ::open(path, O_RDWR | O_CREAT | O_TRUNC | O_CLOEXEC, 0644);
        if (_fd < 0)
        {
            free_lanes();
            return;
        }
        _mapping_size = header_bytes + static_cast<size_t>(capacity) * sizeof(event);
        // Keep the hot-path ring anonymous. A shared file mapping can trigger
        // dirty-page writeback in the middle of the measured steady phase and
        // thereby create the very latency outliers this diagnostic is meant to
        // explain. The destructor persists only the populated prefix.
        auto mapping = ::mmap(
            nullptr,
            _mapping_size,
            PROT_READ | PROT_WRITE,
            MAP_PRIVATE | MAP_ANONYMOUS,
            -1,
            0 );
        if (mapping == MAP_FAILED)
        {
            close_fd();
            free_lanes();
            return;
        }
        // Fault in every ring page before camera warmup so the trace itself
        // cannot introduce first-touch page faults during steady execution.
        std::memset(mapping, 0, _mapping_size);
        _header = static_cast<file_header *>(mapping);
        _events = reinterpret_cast<event *>(static_cast<unsigned char *>(mapping) + header_bytes);
        std::memcpy(_header->magic, "RSV4L2D", 7);
        _header->version = 1;
        _header->header_size = header_bytes;
        _header->event_size = sizeof(event);
        _header->capacity = capacity;
    }

    ~mapped_trace()
    {
        if (_header)
        {
            uint64_t stored_count = 0;
            uint64_t dropped_count = 0;
            for (size_t lane = 0; lane < _lane_count; ++lane)
            {
                stored_count += std::min<uint64_t>(_lanes[lane].event_count, _lane_capacity);
                dropped_count += _lanes[lane].dropped_count;
            }
            _header->event_count = stored_count;
            _header->dropped_count = dropped_count;
            auto populated_size = header_bytes + static_cast<size_t>(stored_count) * sizeof(event);
            if (_fd >= 0 && ::ftruncate(_fd, static_cast<off_t>(populated_size)) == 0)
            {
                write_all(_header, header_bytes, 0);
                off_t offset = header_bytes;
                for (size_t lane = 0; lane < _lane_count; ++lane)
                {
                    auto lane_events = std::min<uint64_t>(
                        _lanes[lane].event_count, _lane_capacity );
                    auto lane_bytes = static_cast<size_t>(lane_events) * sizeof(event);
                    write_all(_events + lane * _lane_capacity, lane_bytes, offset);
                    offset += static_cast<off_t>(lane_bytes);
                }
            }
            ::munmap(_header, _mapping_size);
        }
        close_fd();
        free_lanes();
    }

    mapped_trace(mapped_trace const &) = delete;
    mapped_trace & operator=(mapped_trace const &) = delete;

    void record(stage stage_id, int fd, int result = 0, uint32_t sequence = 0)
    {
        if (!_header)
            return;
        auto tid = static_cast<uint32_t>(::syscall(SYS_gettid));
        auto cpu = ::sched_getcpu();
        auto lane = cpu < 0 ? static_cast<size_t>(tid) % _lane_count
                            : static_cast<size_t>(cpu) % _lane_count;
        auto lane_index = __atomic_fetch_add(
            &_lanes[lane].event_count, uint64_t{ 1 }, __ATOMIC_RELAXED );
        if (lane_index >= _lane_capacity)
        {
            __atomic_fetch_add(
                &_lanes[lane].dropped_count, uint64_t{ 1 }, __ATOMIC_RELAXED );
            return;
        }
        auto index = lane * _lane_capacity + static_cast<size_t>(lane_index);

        timespec now{};
        ::clock_gettime(CLOCK_BOOTTIME, &now);
        event value{};
        value.timestamp_ns = static_cast<uint64_t>(now.tv_sec) * 1'000'000'000ULL
                           + static_cast<uint64_t>(now.tv_nsec);
        value.tid = tid;
        value.cpu = cpu < 0 ? UINT16_MAX : static_cast<uint16_t>(cpu);
        value.stage_id = static_cast<uint16_t>(stage_id);
        value.fd = fd;
        value.result = result;
        value.sequence = sequence;
        _events[index] = value;
        __atomic_thread_fence(__ATOMIC_RELEASE);
    }

private:
    static constexpr uint32_t header_bytes = 4096;

    void write_all(void const * data, size_t size, off_t offset)
    {
        auto bytes = static_cast<unsigned char const *>(data);
        while (size)
        {
            auto written = ::pwrite(_fd, bytes, size, offset);
            if (written < 0 && errno == EINTR)
                continue;
            if (written <= 0)
                return;
            bytes += written;
            size -= static_cast<size_t>(written);
            offset += written;
        }
    }

    void close_fd()
    {
        if (_fd >= 0)
        {
            ::close(_fd);
            _fd = -1;
        }
    }

    void free_lanes()
    {
        if (_lanes)
        {
            ::free(_lanes);
            _lanes = nullptr;
        }
    }

    int _fd = -1;
    size_t _mapping_size = 0;
    file_header * _header = nullptr;
    event * _events = nullptr;
    lane_counter * _lanes = nullptr;
    size_t _lane_count = 0;
    size_t _lane_capacity = 0;
};

inline mapped_trace & trace()
{
    static mapped_trace instance;
    return instance;
}

inline void record(stage stage_id, int fd, int result = 0, uint32_t sequence = 0)
{
    trace().record(stage_id, fd, result, sequence);
}

}  // namespace v4l2_diagnostic
}  // namespace platform
}  // namespace librealsense
