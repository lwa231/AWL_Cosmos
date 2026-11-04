#pragma once
// Warp step-down for approaching impacts (plan section 8 items 4-7). GL-free, read-only: it reads the display state of the
// SimDriver and the ladder of its Warp; the physics is never touched (the slow-down exists only so the user can watch).
//
// Once per frame (after SimDriver::advance) the app calls SlowDown::update():
//   1. Predict the impacts that are due soon, from the DISPLAY state (so times are relative to what the user sees):
//        - the watched shuttles (the followed or selected one), plus EVERY shuttle when options.anyShuttle is on,
//          against their hierarchy parent, with R_trig = surface + atmosphere top (impact.h predictShuttleImpact);
//        - the body pairs that orbit each other in the closest-pair tree (a body with its parent, or with the leaf bodies
//          of a barycentre parent): the two-body conic of plan 5.10 is only meaningful for such pairs, an unrelated pair
//          (two planets of one star) would give nonsense.
//   2. If the soonest impact is inside the look-ahead, step the warp down ONE ladder level (Warp::slowDownOneLevel, which
//      also remembers the warp for the Resume button) and let the next frame re-predict.
//
// Look-ahead. The plan gives 3 s x warp of sim time (impact.h lookAheadTime). The trigger compares the time to impact with
// the look-ahead of the level it would step DOWN TO: warp W steps to W' when t_hit <= 3 s x W'. Then every level, 1x
// included, is entered with about 3 real seconds of that level left, so the user is at 1x about 3 real seconds before the
// impact (plan 8 item 5) and the whole descent from 1e5x takes ~14 s. (Comparing with the look-ahead of the level being left
// would hold each level for 27 s and reach 1x with 30 s to go.) The pre-slow-down warp is kept by Warp for the Resume warp
// button; nothing here changes it.
//
// Not predicted: a shuttle already inside the trigger radius (kind Inside). It entered the air at the warp the user chose or
// resumed; the Resume button must stay usable until the event.
//
// Headless and ML runs never slow down: leave options.enabled false there (the SimDriver itself never touches the warp).
#include <cstdint>
#include <limits>
#include <vector>
#include "app/sim_driver.h"

namespace cosmos {
namespace app {

struct SlowDownOptions {
    bool enabled = true;       // false: update() does nothing (headless / ML)
    bool anyShuttle = false;   // "slow down for any shuttle impact" switch (plan 8 item 4)
};

struct SlowDownReport {
    enum class Cause : std::uint8_t { None, Shuttle, BodyPair };
    Cause cause = Cause::None;
    double timeToImpact = std::numeric_limits<double>::infinity();   // sim s from the display time of the soonest predicted impact
    std::uint64_t subject = 0;   // shuttle id, or body a of the pair
    std::uint64_t other = 0;     // the body hit, or body b of the pair
    bool steppedDown = false;    // the warp was lowered by this update
    double warpBefore = 1.0;
    double warpAfter = 1.0;
};

// The warp one ladder level below `warp` (the same step Warp::slowDownOneLevel takes); `warp` itself at 1x and below.
double nextLowerWarp(double warp);

// Soonest predicted impact among the watched shuttles (all shuttles with anyShuttle) and the body pairs, from the display
// state. Only impacts with t <= horizon count; the result has cause None when there is none.
SlowDownReport predictSoonestImpact(const DisplayState& state, const std::vector<std::uint64_t>& watchedShuttles, bool anyShuttle,
                                    double horizon);

class SlowDown {
public:
    SlowDownOptions options;

    // One frame. `watched` = ids of the followed / selected shuttles. Returns what was found (and whether it stepped down).
    // Does nothing while the driver is paused, with warp <= 1, or when disabled.
    SlowDownReport update(SimDriver& driver, const std::vector<std::uint64_t>& watched);
};

} // namespace app
} // namespace cosmos
