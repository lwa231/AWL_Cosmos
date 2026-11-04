#include "render/gl_stream.h"
#include <glad/glad.h>
#include <iostream>

namespace cosmos {
namespace render {

namespace {
constexpr GLbitfield kMapFlags = GL_MAP_WRITE_BIT | GL_MAP_PERSISTENT_BIT | GL_MAP_COHERENT_BIT;
constexpr std::size_t kSegmentAlign = 256;   // >= every offset alignment GL requires for UBO/SSBO ranges
std::size_t roundUp(std::size_t v, std::size_t a) { return (v + a - 1) / a * a; }
} // namespace

bool StreamArena::create(std::size_t segmentBytes)
{
    segmentBytes = roundUp(segmentBytes, kSegmentAlign);
    glCreateBuffers(1, &buffer_);
    glNamedBufferStorage(buffer_, static_cast<GLsizeiptr>(segmentBytes * kSegments), nullptr, kMapFlags);
    mapped_ = static_cast<std::uint8_t*>(glMapNamedBufferRange(buffer_, 0, static_cast<GLsizeiptr>(segmentBytes * kSegments), kMapFlags));
    if (!mapped_) {
        std::cerr << "StreamArena: persistent mapping of " << segmentBytes * kSegments << " bytes failed\n";
        glDeleteBuffers(1, &buffer_);
        buffer_ = 0;
        return false;
    }
    segmentBytes_ = segmentBytes;
    return true;
}

bool StreamArena::init(std::size_t initialSegmentBytes)
{
    destroy();
    current_ = kSegments - 1;
    used_ = 0;
    return create(initialSegmentBytes);
}

void StreamArena::destroy()
{
    for (int i = 0; i < kSegments; ++i) {
        if (fences_[i]) glDeleteSync(static_cast<GLsync>(fences_[i]));
        fences_[i] = nullptr;
    }
    if (buffer_) {
        glUnmapNamedBuffer(buffer_);
        glDeleteBuffers(1, &buffer_);
    }
    buffer_ = 0;
    mapped_ = nullptr;
    segmentBytes_ = 0;
    used_ = 0;
}

void StreamArena::waitFence(int segment)
{
    GLsync f = static_cast<GLsync>(fences_[segment]);
    if (!f) return;
    // First a non-blocking poll: only count a stall when the GPU really is still reading this segment.
    GLenum r = glClientWaitSync(f, GL_SYNC_FLUSH_COMMANDS_BIT, 0);
    if (r == GL_TIMEOUT_EXPIRED) {
        ++stalls_;
        while (r == GL_TIMEOUT_EXPIRED) r = glClientWaitSync(f, GL_SYNC_FLUSH_COMMANDS_BIT, 1000000000ull);
    }
    if (r == GL_WAIT_FAILED) std::cerr << "StreamArena: glClientWaitSync failed\n";
    glDeleteSync(f);
    fences_[segment] = nullptr;
}

bool StreamArena::begin(std::size_t bytesNeeded)
{
    if (!buffer_) return false;
    if (bytesNeeded > segmentBytes_) {
        // Grow: wait until the GPU has finished with every segment, then reallocate bigger (rare: doubles each time).
        for (int i = 0; i < kSegments; ++i) waitFence(i);
        std::size_t bigger = segmentBytes_ * 2;
        while (bigger < bytesNeeded) bigger *= 2;
        glUnmapNamedBuffer(buffer_);
        glDeleteBuffers(1, &buffer_);
        buffer_ = 0;
        mapped_ = nullptr;
        if (!create(bigger)) return false;
        ++growths_;
    }
    current_ = (current_ + 1) % kSegments;
    waitFence(current_);
    used_ = 0;
    return true;
}

void* StreamArena::alloc(std::size_t bytes, std::size_t alignment, std::size_t* bufferOffset)
{
    const std::size_t start = roundUp(used_, alignment);
    if (start + bytes > segmentBytes_) return nullptr;
    used_ = start + bytes;
    if (bufferOffset) *bufferOffset = static_cast<std::size_t>(current_) * segmentBytes_ + start;
    return mapped_ + static_cast<std::size_t>(current_) * segmentBytes_ + start;
}

void StreamArena::end()
{
    if (fences_[current_]) glDeleteSync(static_cast<GLsync>(fences_[current_]));
    fences_[current_] = glFenceSync(GL_SYNC_GPU_COMMANDS_COMPLETE, 0);
}

} // namespace render
} // namespace cosmos
