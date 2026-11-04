#pragma once
// Persistent mapped streaming buffer, triple-buffered with fences (plan 9.2).
//
// One GL buffer holds kSegments equal segments. Each frame the CPU writes into the next segment through a pointer
// that stays mapped for the buffer's whole life (glBufferStorage with MAP_PERSISTENT | MAP_COHERENT, so no flush calls
// are needed). When a frame's commands are issued, end() puts a fence behind them; before a segment is written again
// begin() waits on that fence, so the CPU never overwrites data the GPU is still reading.
//
// Use: begin(bytes) -> alloc(...) as many times as needed (aligned sub-ranges) -> bind the ranges for drawing -> end().
#include <cstddef>
#include <cstdint>

namespace cosmos {
namespace render {

class StreamArena {
public:
    static constexpr int kSegments = 3;

    StreamArena() = default;
    ~StreamArena() { destroy(); }
    StreamArena(const StreamArena&) = delete;
    StreamArena& operator=(const StreamArena&) = delete;

    bool init(std::size_t initialSegmentBytes);
    void destroy();

    // Starts a frame that will allocate at most `bytesNeeded` (including alignment padding). Advances to the next
    // segment, waits for its fence, and grows the buffer first if the segment is too small.
    bool begin(std::size_t bytesNeeded);

    // A sub-range of the current segment aligned to `alignment` (a power of two). Returns the CPU pointer and sets
    // *bufferOffset to the offset inside the GL buffer for glBindBufferRange. nullptr when the segment is full.
    void* alloc(std::size_t bytes, std::size_t alignment, std::size_t* bufferOffset);

    // Puts a fence behind everything the GPU was told to draw this frame.
    void end();

    unsigned int buffer() const { return buffer_; }
    std::size_t segmentBytes() const { return segmentBytes_; }
    std::size_t usedBytes() const { return used_; }
    std::uint32_t stalls() const { return stalls_; }       // times begin() had to block on a fence
    std::uint32_t growths() const { return growths_; }

private:
    bool create(std::size_t segmentBytes);
    void waitFence(int segment);

    unsigned int buffer_ = 0;
    std::uint8_t* mapped_ = nullptr;
    std::size_t segmentBytes_ = 0;
    int current_ = kSegments - 1;     // begin() moves to segment 0 first
    std::size_t used_ = 0;
    void* fences_[kSegments] = {};   // GLsync
    std::uint32_t stalls_ = 0;
    std::uint32_t growths_ = 0;
};

} // namespace render
} // namespace cosmos
