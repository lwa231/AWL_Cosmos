#pragma once
// Plain data the app hands to the renderer for explosions (plan 9.7, 6.5). Header-only, GL-free. The layouts below are the
// std430 layouts of shaders/fragments.comp and shaders/spark.vert; the sizes are asserted so the two cannot drift apart.
//
// How the GPU part works (the CPU half is app/explosions.h):
//  - Fragments live in one pool of kExplosionMaxFragments slots on the GPU. A new explosion overwrites the oldest slots
//    (recycling oldest-first), so the budget is fixed: 20k fragments and 8 sparks per fragment = 160k sparks.
//  - Each explosion has an ANCHOR in double precision on the CPU (the contact point, moving with the body it happened on).
//    Fragment positions and velocities are floats RELATIVE to the anchor, in the frame that translates with that body, so
//    they stay small and precise at 1e11 m from the origin. Every frame the CPU uploads, per live explosion, the anchor's
//    camera-relative offset, the sim and real time step of the frame and the few nearby bodies (mu, radius, atmosphere,
//    position relative to the anchor).
//  - shaders/fragments.comp integrates every fragment with a fixed substep (gravity of those bodies + exact drag with the
//    fragment's ballistic coefficient); shaders/spark.vert draws the fragments and their sparks additively.
//  - Fragments are visual only: outside the determinism guarantee (plan 4.6).
#include <cstdint>
#include <vector>

namespace cosmos {
namespace app {

inline constexpr std::uint32_t kExplosionMaxFragments = 20000;   // pool size (plan 9.7)
inline constexpr std::uint32_t kExplosionSparksPerFragment = 8;  // 20000 x 8 = 160k sparks
inline constexpr std::uint32_t kExplosionMaxLive = 64;           // explosions alive at once (table slots)
inline constexpr std::uint32_t kExplosionMaxBodies = 4;          // bodies per explosion whose gravity / air acts on its fragments

// One fragment: 64 bytes. Position and velocity are relative to its explosion's anchor, in the frame of the body the
// explosion is anchored to. A fragment is alive while its explosion slot still carries its serial.
struct FragmentGpu {
    float px = 0.0f, py = 0.0f, vx = 0.0f, vy = 0.0f;
    float simAge = 0.0f;       // sim seconds since the explosion
    float realAge = 0.0f;      // real seconds since the explosion
    float life = 1.0f;         // sim lifetime, s
    float minReal = 2.0f;      // minimum real-time visibility, s (plan 9.7)
    float ballistic = 0.01f;   // Cd A / (2 m) in m^2/kg: drag rate k = rho * ballistic (plan 5.6)
    float size = 1.0f;         // visual size multiplier
    std::uint32_t slot = 0;    // explosion table slot
    std::uint32_t serial = 0;  // serial of the explosion that owns it (0: dead)
    std::uint32_t landed = 0;  // 0 flying; k + 1: resting on the surface of table body k
    std::uint32_t seed = 0;    // per-fragment random bits (spark phases)
    float pad0 = 0.0f, pad1 = 0.0f;
};
static_assert(sizeof(FragmentGpu) == 64, "FragmentGpu must match the std430 Fragment of fragments.comp");

// One nearby body, relative to the explosion's anchor: 48 bytes.
struct ExplosionBodyGpu {
    float x = 0.0f, y = 0.0f, mu = 0.0f, radius = 0.0f;          // centre relative to the anchor [m], GM, radius
    float indirectX = 0.0f, indirectY = 0.0f;                    // acceleration of the anchor frame due to this body (tidal term; 0 for the frame's own body)
    float vx = 0.0f, vy = 0.0f;                                  // velocity relative to the anchor frame's body [m/s]
    float rho0 = 0.0f, scaleHeight = 0.0f, spin = 0.0f;          // atmosphere (0: none), spin rate [rad/s]
    float altitude = 0.0f;                                       // altitude of the anchor above this body's surface [m] (double precision on the CPU)
};
static_assert(sizeof(ExplosionBodyGpu) == 48, "ExplosionBodyGpu must match the std430 ExplosionBody");

// One live explosion: 224 bytes.
struct ExplosionGpu {
    float anchorX = 0.0f, anchorY = 0.0f;   // anchor relative to the camera [m]
    float dtSim = 0.0f, dtReal = 0.0f;      // this frame's sim and real seconds (0 on the frame of the spawn)
    std::uint32_t serial = 0;               // 0: slot unused
    std::uint32_t bodyCount = 0;
    std::uint32_t pad0 = 0, pad1 = 0;
    ExplosionBodyGpu bodies[4];
};
static_assert(sizeof(ExplosionGpu) == 32 + 4 * 48, "ExplosionGpu must match the std430 Explosion");

// New fragments to write into pool slots [firstSlot, firstSlot + fragments.size()). A spawn that wraps the pool is two batches.
struct FragmentBatch {
    std::uint32_t firstSlot = 0;
    std::vector<FragmentGpu> fragments;
};

struct ExplosionFrame {
    // Empty when nothing is alive (the renderer then does no work). Otherwise exactly kExplosionMaxLive entries, the unused
    // ones with serial 0.
    std::vector<ExplosionGpu> table;
    std::vector<FragmentBatch> spawns;   // written to the pool before this frame's update
    std::uint32_t fragmentCount = 0;     // pool slots in use so far (dispatch and draw range)

    bool active() const { return !table.empty(); }
    void clear()
    {
        table.clear();
        spawns.clear();
        fragmentCount = 0;
    }
};

} // namespace app
} // namespace cosmos
