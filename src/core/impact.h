#pragma once
#include <cstdint>
#include <optional>
#include <vector>
#include "core/bodies.h"
#include "core/events.h"
#include "core/vec3.h"

namespace cosmos {

// Impact prediction and the swept shuttle-vs-body test (plan sections 5.10, 5.11, 8 items 1-4, 4.7).
// Everything here is a pure function of its arguments: no state, no global data, fixed operation order (plan 4.6).
// Predictions use only x and y (2D, like kepler.h). SI units; mu = GM.

// ---------------------------------------------------------------------------------------------------------------
// Trigger radius (plan 5.5, 5.10)

// Atmosphere cutoff density (kg/m^3): the top of the atmosphere is where rho falls to this value.
inline constexpr double kAtmosphereCutoffDensity = 1e-6;

// top = H ln(rho0 / 1e-6). 0 without an atmosphere, and 0 when rho0 <= 1e-6 (the air never gets that thin to matter).
double triggerAtmosphereTop(const std::optional<Atmosphere>& atmosphere);

// R_trig = R + atmosphere top.
double triggerRadius(double radius, const std::optional<Atmosphere>& atmosphere);

// ---------------------------------------------------------------------------------------------------------------
// 1. Impact against a parent from the osculating two-body conic (plan 5.10)

enum class ImpactKind : std::uint8_t {
    None,        // the path never reaches R_trig: periapsis q >= R_trig, or an unbound orbit that is already outbound
    Impact,      // reaches R_trig inbound after t seconds
    Inside,      // r <= R_trig right now (t = 0); the orbit may still leave R_trig later, so this is not a hit forecast
    InsideBound, // bound and the apoapsis p/(1-e) <= R_trig: it never leaves R_trig; there is no crossing, drag handles it
    Invalid      // non-finite input, mu <= 0, R_trig <= 0 or r = 0
};

struct ImpactPrediction {
    ImpactKind kind = ImpactKind::None;
    double t = 0.0;      // Impact: seconds from now until r = R_trig inbound. Inside / InsideBound: 0. None / Invalid: +infinity
    double nuHit = 0.0;  // Impact: true anomaly of the crossing, in the (-pi, 0) inbound branch (the nu of stateToElements)
    double speedHit = 0.0; // Impact: speed relative to the parent at the crossing (vis-viva)
    Vec3 point;          // Impact: crossing point relative to the parent (|point| = R_trig, z = 0)

    bool hits() const { return kind == ImpactKind::Impact; }
};

// Will the body at relative position r with relative velocity v (both relative to the parent, mu = mu_parent, plus the
// body's own mu if it is not massless) reach the sphere of radius rTrig?
//
// Classification uses the osculating elements of stateToElements, as in plan 5.10:
//   bound and apoapsis a(1+e) <= rTrig  -> InsideBound;   r <= rTrig -> Inside;   q >= rTrig -> None;
//   unbound and r.v >= 0 (not inbound)  -> None;           otherwise Impact.
// For an Impact the crossing is the inbound branch nu_hit < 0 of cos(nu_hit) = (p / rTrig - 1) / e. The time is the
// plan's Kepler-equation difference t = (M_hit - M_now) / n (wrapped to [0, period) for an ellipse), evaluated
// from (e cos E, e sin E) = (1 - r/a, r.v / sqrt(mu a)) and its hyperbolic twin instead of from nu and tan(E/2): the
// same numbers, but without the cancellation in 1 - e, so near-radial orbits (p -> 0, including h = 0 exactly) and
// near-parabolic ones stay accurate. M = E - e sin E is summed as (E - sin E) + (1 - e) sin E with a series for small E.
// An (almost) exactly parabolic orbit (|energy| r / mu < 1e-15) uses Barker's equation.
// Retrograde orbits (h < 0) need no special case: the signed r.v and the energy do not depend on the sense of rotation.
ImpactPrediction predictParentImpact(const Vec3& r, const Vec3& v, double mu, double rTrig);

// Convenience: shuttle (massless, so only the parent's mu attracts it) against its parent body, with R_trig =
// R_parent + atmosphere top of the parent. rShuttle / vShuttle / rParent / vParent are absolute.
ImpactPrediction predictShuttleImpact(const Vec3& rShuttle, const Vec3& vShuttle, const Vec3& rParent, const Vec3& vParent,
                                      double muParent, double parentRadius, const std::optional<Atmosphere>& parentAtmosphere);

// ---------------------------------------------------------------------------------------------------------------
// 3. Body-body pair prediction: the same conic for the pair, mu = mu_a + mu_b, R_trig = R_a + R_b (plan 5.10)

// point in the result is relative to body a.
ImpactPrediction predictPairImpact(const Vec3& pa, const Vec3& va, double muA, double radiusA,
                                   const Vec3& pb, const Vec3& vb, double muB, double radiusB);

// Same, for the bodies with store indices i and j.
ImpactPrediction predictPairImpact(const Bodies& bodies, std::size_t i, std::size_t j);

struct PairImpact {
    std::size_t ia = 0, ib = 0; // ia < ib, store indices at the time of the call
    BodyId a = 0, b = 0;
    ImpactPrediction prediction;
};

// All pairs predicted to collide within `horizon` seconds (kind Impact, 0 < t <= horizon), sorted by (t, a, b).
// out is cleared first; reuse it between calls to avoid allocations.
void predictPairImpacts(const Bodies& bodies, double horizon, std::vector<PairImpact>& out);

// ---------------------------------------------------------------------------------------------------------------
// 2. Rough closing estimate (plan 5.10)

// Smallest positive root of  gap = vc t + 0.5 ac t^2  (gap = separation minus the contact distance, vc = closing speed,
// ac = closing acceleration, both positive when closing). gap <= 0 gives 0. nullopt when the quadratic never reaches
// the gap (not closing, or closing but decelerating to a stop first). Cancellation-free root formula.
std::optional<double> closingTime(double gap, double closingSpeed, double closingAccel);

// From vectors: dp = p_b - p_a, dv = v_b - v_a, da = a_b - a_a; contactDistance = R_a + R_b. The closing speed is
// -dp.dv / |dp| and the closing acceleration -dp.da / |dp| (components along the line of centres, a straight-line
// estimate: the sideways velocity is not turned into extra separation). Exact for motion along the line of centres
// with constant acceleration.
std::optional<double> estimateClosing(const Vec3& dp, const Vec3& dv, const Vec3& da, double contactDistance);

// ---------------------------------------------------------------------------------------------------------------
// 4. Look-ahead (plan 8.4): 3 s of real time at the current warp

inline constexpr double kLookAheadSeconds = 3.0;

// Sim-time look-ahead for a warp factor (0 for warp <= 0).
double lookAheadTime(double warp);

// The predicted time if it falls within the look-ahead of `warp`, otherwise nullopt. t must be finite and >= 0.
std::optional<double> withinLookAhead(double t, double warp);

// "Impact within the look-ahead?": the predicted time of an Impact if t <= lookAheadTime(warp). Inside / InsideBound /
// None / Invalid give nullopt (the entry has already happened, or there is none), unless countInside is set, which
// reports Inside as t = 0 (for a caller that wants to keep slowing down while a shuttle is in the air).
std::optional<double> impactWithinLookAhead(const ImpactPrediction& p, double warp, bool countInside = false);

// ---------------------------------------------------------------------------------------------------------------
// 5. Swept shuttle-vs-body test over one accepted shuttle step (plan 5.11, 8 items 1 and 3)

struct SweptShuttleBody {
    std::optional<double> surface;    // first contact with R_body + R_shuttle, seconds from the step start (0 if overlapping)
    std::optional<double> atmosphere; // first crossing into R_body + atmosphere top, seconds from the step start
    bool startedInAtmosphere = false; // the step began inside the trigger radius (the entry was reported earlier: no event now)
    Vec3 surfacePoint;                // shuttle position at the surface contact (chord interpolation); valid with `surface`
};

// Relative motion over the step is the chord from p0 = s0 - b0 to p1 = s1 - b1 (shuttle start/end minus the body's
// positions at the same two instants; take the body's from IAS15 dense output), tested with sweptContactTimeBetween.
// The atmosphere trigger radius is max(R_body + atmosphereTop, R_body + R_shuttle); no atmosphere (top <= 0) gives no
// atmosphere crossing. A step that starts inside the trigger radius reports no crossing (startedInAtmosphere), so an
// AtmosphereEntry is raised once, not at every step. When `surface` is set and the step started outside, `atmosphere`
// is set too and is <= surface (the bigger sphere is reached first).
SweptShuttleBody sweptShuttleVsBody(const Vec3& s0, const Vec3& s1, const Vec3& b0, const Vec3& b1, double dt,
                                    double bodyRadius, double shuttleRadius, double atmosphereTop);

struct ShuttleBodyHit {
    std::size_t index = 0; // body index in the store
    BodyId body = 0;
    SweptShuttleBody contact;
};

// The shuttle against every body: bodies.state is the start of the step, `bodiesEnd` the body positions at the end
// of the same interval (same size as the store). Bodies with no crossing and no contact are left out. out is cleared
// first, then sorted by (earliest event time, body id), so the order is deterministic.
void findShuttleContacts(const Vec3& s0, const Vec3& s1, double shuttleRadius, const Bodies& bodies,
                         const BodyState& bodiesEnd, double dt, std::vector<ShuttleBodyHit>& out);

// Events (plan 4.7) for a single body at the step starting at tStart (sim time): AtmosphereEntry at
// tStart + atmosphere and ShuttleDestroyed(by = body) at tStart + surface. Appended in that order; the queue sorts them.
void appendShuttleStepEvents(std::vector<Event>& out, double tStart, BodyId shuttle, BodyId body, const SweptShuttleBody& c);

// Events from a findShuttleContacts result. The shuttle is destroyed once, by the body with the earliest surface
// contact (ties: smaller body id); later atmosphere entries of other bodies (time > that contact) are dropped, as
// the shuttle no longer exists. Returns true if the shuttle was destroyed.
bool appendShuttleStepEvents(std::vector<Event>& out, double tStart, BodyId shuttle, const std::vector<ShuttleBodyHit>& hits);

} // namespace cosmos
