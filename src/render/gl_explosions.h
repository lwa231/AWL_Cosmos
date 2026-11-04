#pragma once
// GPU side of the explosions (plan 9.7, 6.5): the fragment pool, the per-frame explosion table, the compute pass that
// integrates the fragments and the additive draw of fragments + sparks. All the data comes from app::ExplosionFrame
// (app/explosion_frame.h); nothing here feeds back into the simulation.
//
// Bindings: fragment pool = SSBO 8, explosion table = SSBO 9 (the frame uniform block at binding 0 must already be bound).
#include <cstdint>
#include <string>
#include "app/explosion_frame.h"

namespace cosmos {
namespace render {

class ExplosionRenderer {
public:
    ExplosionRenderer() = default;
    ~ExplosionRenderer() { destroy(); }
    ExplosionRenderer(const ExplosionRenderer&) = delete;
    ExplosionRenderer& operator=(const ExplosionRenderer&) = delete;

    bool init(const std::string& shaderDir);   // needs a current GL 4.5 context
    void destroy();

    struct Result {
        std::uint32_t fragments = 0;   // pool slots processed
        std::uint32_t drawCalls = 0;
    };
    // Uploads the spawns and the table, runs the compute pass over the pool, then draws additively. Does nothing when the
    // frame has no live explosion. Leaves the blend state as it found it.
    Result drawFrame(const app::ExplosionFrame& frame);

private:
    unsigned int compute_ = 0, draw_ = 0;
    unsigned int pool_ = 0, table_ = 0;
};

} // namespace render
} // namespace cosmos
