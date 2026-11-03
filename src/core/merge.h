#pragma once
#include <optional>
#include "core/bodies.h"
#include "core/diagnostics.h"
#include "core/events.h"
#include "core/vec3.h"

namespace cosmos {

// What one merge did (plan section 5.4). Kept by the caller as the merge log.
struct MergeResult {
    BodyId survivor = 0;
    BodyId absorbed = 0;
    double t = 0.0;
    double relSpeed = 0.0;        // |v_a - v_b| just before the merge
    Vec3 point;                   // contact point used for the Collision event

    BodyKind oldKind = BodyKind::Planet; // survivor's kind before / after (equal when not reclassified)
    BodyKind newKind = BodyKind::Planet;

    double omegaRaw = 0.0;        // S_G / I_G before the breakup clamp (rad/s)
    double omegaBreak = 0.0;      // sqrt(mu / R^3) of the merged body
    bool spinClamped = false;     // |omegaRaw| > omegaBreak, spin set to +-omegaBreak

    double dEnergyG = 0.0;        // E.G(after) - E.G(before), full system, section 5.3 potential when overlapping
    // L.G that the merged body does not carry: the z clamp loss when spinClamped, plus any x/y orbital angular
    // momentum of the pair about the new centre (spin is a scalar about z, so that part cannot be stored).
    // L.G(after) + lostAngularMomentumG = L.G(before), up to rounding.
    Vec3 lostAngularMomentumG;
};

// Merge two bodies perfectly inelastically into the heavier one (plan 5.4, 12.2), at sim time t.
// The survivor is the larger mu (the lower ID on a tie); it keeps its ID, name, colour, atmosphere, kindLocked flag
// and inertia factor. The absorbed body is removed from the store (its ID is never reused).
// Pushes Collision, Merge and, if the kind changed, KindChanged onto `events` when it is not null.
// Returns nullopt (nothing changed) if an ID is unknown or both IDs are the same.
std::optional<MergeResult> mergeBodies(Bodies& bodies, BodyId idA, BodyId idB, double t, EventQueue* events = nullptr);

// Add one merge's dE and lost L onto the running ledger (see withMergesAddedBack in diagnostics.h).
void record(MergeLedger& ledger, const MergeResult& m);

} // namespace cosmos
