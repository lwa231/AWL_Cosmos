#pragma once
// Explosion registry and per-frame upload data (plan 9.7, 6.5, 8 item 3). GL-free, read-only on the simulation: nothing here
// feeds back into src/core, and the random numbers come from a local counter-based stream (core/rng.h), never the physics
// streams of the shuttles.
//
// Flow per frame in the app:
//     explosions.handleEvents(driver.dueEvents(), driver.state(), driver.sim());   // spawn for ShuttleDestroyed / Collision
//     explosions.update(driver.state(), fi.simDelta, realDt, toCamera, frame.explosions);
//     renderer->drawExplosions();                                                    // draw order step 8, after the shuttles
//
// Anchor. An explosion is anchored at the contact point in DOUBLE precision, as an offset from the body it happened on (the
// ground it hit, or the survivor of a body collision), so the cloud travels with that body: anchor(t) = body(t) + offset.
// If that body disappears (merged away) the anchor carries on with the body's last velocity. Fragment state is float,
// relative to the anchor, in the frame translating with the body (gravity of the body and of up to three other nearby
// bodies, the others entering as tidal differences; drag with the body's rotating air). See explosion_frame.h.
//
// Lifetimes run in SIM time with a minimum REAL-time visibility (default 2 s): a fragment fades along
// u = min(simAge / life, realAge / minReal) and is gone at u >= 1, so at 1e5x the cloud still lives for 2 real seconds, and at
// 1x it lasts its sim lifetime. Both ages stand still while the clock is stopped. A fragment that reaches the surface of a
// listed body stays there (rotating with it) until its life ends.
//
// Budget: kExplosionMaxFragments slots recycled oldest-first (a new explosion overwrites the oldest slots; an explosion that
// loses all its slots disappears), kExplosionMaxLive explosions at once (the oldest is evicted beyond that).
#include <cstdint>
#include <deque>
#include <functional>
#include <unordered_map>
#include <vector>
#include "app/explosion_frame.h"
#include "app/sim_driver.h"
#include "core/events.h"
#include "core/simulation.h"
#include "core/vec3.h"

namespace cosmos {
namespace app {

struct ExplosionConfig {
    std::uint32_t maxFragments = kExplosionMaxFragments;
    double minRealVisible = 2.0;     // s of real time every fragment stays visible at least
    double maxRealDt = 0.25;         // s: a longer real frame (a stall) counts as this much
    std::uint64_t seed = 0xE5710510ull;   // salt of the visual random stream
};

// What to spawn, in absolute double-precision world terms.
struct ExplosionSpec {
    Vec3 contact;                    // contact point (the anchor)
    std::uint64_t refBody = 0;       // body the anchor travels with (0: a free anchor with refVel)
    Vec3 refPos, refVel;             // that body at the event time
    double refSpin = 0.0;            // its spin rate, rad/s: the ground moves under the cloud
    Vec3 normal;                     // unit vector away from the body centre at the contact
    Vec3 impactVelocity;             // impactor velocity relative to the body
    double impactorRadius = 2.0;     // m: scales the cloud (a 2 m shuttle ... a planet)
    std::uint64_t sourceId = 0;      // shuttle or body id, keys the random stream together with `time`
    double time = 0.0;               // sim time of the event
};

// Spawn parameters from the size and speed of the impact ("scaled", plan 8 item 3). Pure; exposed for tests.
struct ExplosionParams {
    std::uint32_t fragments = 0;
    double speedMin = 0.0, speedMax = 0.0;       // ejecta speed range, m/s
    double lifeMin = 0.0, lifeMax = 0.0;         // sim lifetime range, s
    double ballisticMin = 0.0, ballisticMax = 0.0;   // Cd A / (2 m), m^2/kg
    double halfAngle = 0.0;                      // ejecta cone half angle around the surface normal, rad
    double spawnRadius = 0.0;                    // fragments start within this distance of the contact point, m
};
ExplosionParams explosionParams(double impactorRadius, double impactSpeed);

class ExplosionSystem {
public:
    // Writes the camera-relative float position of a world point (the app's camera does the double subtraction).
    using ToCamera = std::function<void(const Vec3& world, float& x, float& y)>;

    explicit ExplosionSystem(const ExplosionConfig& config = {});

    // Forget everything (a rebuilt simulation: the events and bodies the explosions refer to are gone).
    void clear();

    // Spawn for the due events of this frame: ShuttleDestroyed (at the contact point of the dead shuttle, read from the
    // simulation store) and Collision (at the event's contact point, on the survivor). Other events are ignored.
    void handleEvents(const std::vector<Event>& due, const DisplayState& state, const Simulation& sim);

    // Direct spawn. Returns the explosion's serial (never 0), or 0 when it cannot be made (no fragments).
    std::uint32_t spawn(const ExplosionSpec& spec);

    // One frame: advance the ages by the sim and real seconds of the frame (both 0 while the clock is stopped), drop the
    // finished explosions, follow the bodies at the display time and fill `out`. `out` is cleared first.
    void update(const DisplayState& state, double simDt, double realDt, const ToCamera& toCamera, ExplosionFrame& out);

    // ---- inspection (tests, stats) ----
    struct Range {
        std::uint32_t first = 0, count = 0;
    };
    struct Explosion {
        std::uint32_t serial = 0;
        std::uint32_t slot = 0;          // table slot
        std::uint64_t refBody = 0;
        Vec3 offset;                     // anchor - body position
        Vec3 anchor;                     // world position as of the last update
        Vec3 lastRefVel;
        double simAge = 0.0, realAge = 0.0;
        double lifeMax = 0.0, minRealMax = 0.0;
        std::vector<Range> ranges;       // pool slots it still owns
        std::uint32_t live = 0;          // fragments it still owns
        bool fresh = true;               // spawned since the last update
    };
    const std::deque<Explosion>& explosions() const { return explosions_; }
    std::size_t liveExplosions() const { return explosions_.size(); }
    std::uint32_t liveFragments() const;
    std::uint32_t fragmentHighWater() const { return highWater_; }
    const ExplosionConfig& config() const { return config_; }

private:
    struct Seen {   // last display state of a body, for events whose bodies have since vanished
        Vec3 r, v;
        double mu = 0.0, radius = 0.0, spin = 0.0;
        double t = 0.0;   // sim time of this sample
    };
    bool bodyAt(std::uint64_t id, double t, const DisplayState& st, const Simulation& sim, Vec3& r, Vec3& v) const;
    // Takes n pool slots (at most two ranges, the second after a wrap) and removes them from the older explosions.
    std::vector<Range> allocate(std::uint32_t n);
    std::uint32_t freeSlot();

    ExplosionConfig config_;
    std::deque<Explosion> explosions_;   // oldest first
    std::vector<FragmentBatch> pending_; // spawns not yet handed to the renderer
    std::unordered_map<std::uint64_t, Seen> seen_;
    std::uint32_t head_ = 0;             // next pool slot to write
    std::uint32_t highWater_ = 0;
    std::uint32_t serialCounter_ = 0;
};

} // namespace app
} // namespace cosmos
