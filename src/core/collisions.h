#pragma once
#include <optional>
#include <vector>
#include "core/bodies.h"
#include "core/vec3.h"

namespace cosmos {

// Swept collision test (plan section 5.11). Relative motion over the step is linear, p(t) = p0 + w t, and the
// bodies touch when |p| = R (R = R_a + R_b). Returns the first contact time t* in [0, dt], or nullopt when the
// path never reaches R within the step. Already overlapping (|p0| <= R) gives exactly 0.
// The root is evaluated as C / (-B + sqrt(B^2 - A C)), algebraically equal to the plan's (-B - sqrt(..)) / A but
// free of cancellation when the step is long and the miss distance small.
std::optional<double> sweptContactTime(const Vec3& p0, const Vec3& w, double R, double dt);

// Same test from the relative positions at the start and end of the step: w = (p1 - p0) / dt (chord velocity).
// For dt <= 0 only the overlap case is reported.
std::optional<double> sweptContactTimeBetween(const Vec3& p0, const Vec3& p1, double R, double dt);

// One colliding body pair. ia < ib are indices into the store at the time of the call; a and b are their IDs.
struct BodyContact {
    std::size_t ia = 0;
    std::size_t ib = 0;
    BodyId a = 0;
    BodyId b = 0;
    double t = 0.0; // contact time within the step, seconds from the start
};

// All colliding pairs among the bodies over a step of length dt, using the start positions and the bodies' current
// velocities as the relative velocity. Sorted by (t, a, b) with a < b by store index, so the order is
// deterministic and independent of floating-point ties in the loop order.
std::vector<BodyContact> findBodyContacts(const Bodies& bodies, double dt);

// Same, but the relative motion of each pair is the chord between the start state (bodies.state) and `end`,
// the body positions at the end of the step. `end` must have the same size as the store.
std::vector<BodyContact> findBodyContacts(const Bodies& bodies, const BodyState& end, double dt);

} // namespace cosmos
