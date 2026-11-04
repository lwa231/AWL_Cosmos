#include "render/gl_explosions.h"
#include "render/gl_shader.h"
#include <glad/glad.h>
#include <iostream>
#include <vector>

namespace cosmos {
namespace render {

namespace {
constexpr GLuint kBindFragments = 8;
constexpr GLuint kBindTable = 9;
constexpr GLuint kBindStats = 10;
constexpr GLuint kGroupSize = 64;   // local_size_x of fragments.comp
}

bool ExplosionRenderer::init(const std::string& shaderDir)
{
    destroy();
    compute_ = buildComputeProgram(shaderDir, "fragments.comp");
    draw_ = buildProgram(shaderDir, "spark.vert", "spark.frag");
    if (!compute_ || !draw_) {
        destroy();
        return false;
    }
    // The pool starts zeroed: serial 0 means dead. Spawns are written with glNamedBufferSubData; the compute shader updates it in place.
    const std::vector<std::uint8_t> zeros(sizeof(app::FragmentGpu) * app::kExplosionMaxFragments, 0);
    glCreateBuffers(1, &pool_);
    glNamedBufferStorage(pool_, static_cast<GLsizeiptr>(zeros.size()), zeros.data(), GL_DYNAMIC_STORAGE_BIT);
    glCreateBuffers(1, &table_);
    glNamedBufferStorage(table_, static_cast<GLsizeiptr>(sizeof(app::ExplosionGpu) * app::kExplosionMaxLive), nullptr, GL_DYNAMIC_STORAGE_BIT);
    glCreateBuffers(1, &stats_);
    glNamedBufferStorage(stats_, static_cast<GLsizeiptr>(sizeof(app::ExplosionStatsGpu)), nullptr, GL_DYNAMIC_STORAGE_BIT);
    return true;
}

void ExplosionRenderer::destroy()
{
    if (pool_) glDeleteBuffers(1, &pool_);
    if (table_) glDeleteBuffers(1, &table_);
    if (stats_) glDeleteBuffers(1, &stats_);
    if (compute_) glDeleteProgram(compute_);
    if (draw_) glDeleteProgram(draw_);
    pool_ = table_ = stats_ = compute_ = draw_ = 0;
}

ExplosionRenderer::Result ExplosionRenderer::drawFrame(const app::ExplosionFrame& frame)
{
    Result res;
    if (!compute_ || !frame.active() || frame.fragmentCount == 0) return res;
    if (frame.table.size() != app::kExplosionMaxLive) {
        std::cerr << "ExplosionRenderer: the explosion table must have " << app::kExplosionMaxLive << " entries\n";
        return res;
    }

    for (const app::FragmentBatch& b : frame.spawns) {
        if (b.fragments.empty() || b.firstSlot + b.fragments.size() > app::kExplosionMaxFragments) continue;
        glNamedBufferSubData(pool_, static_cast<GLintptr>(b.firstSlot * sizeof(app::FragmentGpu)),
                             static_cast<GLsizeiptr>(b.fragments.size() * sizeof(app::FragmentGpu)), b.fragments.data());
    }
    glNamedBufferSubData(table_, 0, static_cast<GLsizeiptr>(frame.table.size() * sizeof(app::ExplosionGpu)), frame.table.data());

    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, kBindFragments, pool_);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, kBindTable, table_);
    const GLuint zero = 0;   // the compute pass accumulates this frame's cloud statistics from scratch
    glClearNamedBufferData(stats_, GL_R32UI, GL_RED_INTEGER, GL_UNSIGNED_INT, &zero);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, kBindStats, stats_);

    glUseProgram(compute_);
    glUniform1ui(0, frame.fragmentCount);
    glDispatchCompute((frame.fragmentCount + kGroupSize - 1) / kGroupSize, 1, 1);
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

    glUseProgram(draw_);   // premultiplied colour with alpha 0: the (ONE, ONE_MINUS_SRC_ALPHA) blend of the frame adds it
    glDrawArraysInstanced(GL_TRIANGLE_STRIP, 0, 4, static_cast<GLsizei>(frame.fragmentCount * (1 + app::kExplosionSparksPerFragment)));
    res.fragments = frame.fragmentCount;
    res.drawCalls = 1;
    return res;
}

} // namespace render
} // namespace cosmos
