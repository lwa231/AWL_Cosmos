#pragma once
#include "core/impact.h"
#include "core/vec3.h"

// 3D impact prediction (V1.1 step 5, docs/V1_1_3D_CORE.md). DECLARATIONS ONLY: the developer writes the definitions
// (src/core/impact3d.cpp). The spec is tests/test_3d_core_impact.cpp. Frame and units as kepler3d.h.
//
// What already works in 3D and needs NO new function (the spec tests exercise these out of plane instead):
//  * hierarchy.h: buildHierarchy, hillRadius(Hierarchy, node), shuttleParent, buildHillTable. They only use norm/norm2
//    of 3D differences, so the 3D Hill radius d * cbrt(mu / (3 mu_sibling)) is already the right one
//    (tests/test_3d_core_hierarchy.cpp).
//  * collisions.h: sweptContactTime, sweptContactTimeBetween, findBodyContacts; impact.h: sweptShuttleVsBody,
//    findShuttleContacts, estimateClosing, closingTime. All use dot/norm2 of full Vec3s (tests/test_3d_core_swept.cpp).
//  * radii.h: hillRadius(a, e, muM, muCentral), soiRadius: scalar, frame-free.
// What is 2D only: predictParentImpact (and so predictShuttleImpact / predictPairImpact) reads only x and y, and its
// crossing point has z = 0. The 3D versions below replace it.

namespace cosmos {

// predictParentImpact with the full 3D r and v (relative to the parent; mu = mu_parent + mu_body; rTrig > 0).
// Same classification, same ImpactKind meanings, same accuracy rules (near-radial, near-parabolic, Barker) as the 2D
// function, with |r|, |v|^2 and r.v taken over all three components:
//   bound and apoapsis a(1+e) <= rTrig -> InsideBound;  |r| <= rTrig -> Inside;  q >= rTrig -> None;
//   unbound and r.v >= 0 -> None;  otherwise Impact.
// Impact fields:
//   t        seconds until |r| = rTrig on the inbound branch (rotation invariant: depends only on |r|, |v|, r.v, mu)
//   nuHit    true anomaly of the crossing on the inbound branch, -pi <= nuHit <= 0 as in 2D (-pi for a radial orbit),
//            measured in the orbit plane in the direction of motion
//   speedHit vis-viva speed at rTrig
//   point    the 3D crossing point relative to the parent, |point| = rTrig, lying in the orbit plane (point . h = 0).
//            Get it in the plane spanned by r and v: point = rTrig (cos(du) r_hat + sin(du) t_hat), where t_hat is the
//            unit vector in the plane perpendicular to r_hat on the side of motion (h x r_hat / |h|) and du = nuHit - nu
//            the angle still to travel (nu of the current state). For a radial orbit (h = 0) point = rTrig * r_hat.
// Invalid: non-finite input, mu <= 0, rTrig <= 0 or r = 0 (as 2D).
// Reduction: for r.z = v.z = 0 every field equals predictParentImpact(r, v, mu, rTrig) to about 1e-12 relative
// (kind exactly equal; point.z == 0).
// Rotation invariance: for any rotation Q, predictParentImpact3D(Q r, Q v) has the same kind, t, nuHit and speedHit
// as predictParentImpact3D(r, v), and point' = Q point (to about 1e-12 relative).
ImpactPrediction predictParentImpact3D(const Vec3& r, const Vec3& v, double mu, double rTrig);

// Massless shuttle against its parent, 3D: predictParentImpact3D(rShuttle - rParent, vShuttle - vParent, muParent,
// triggerRadius(parentRadius, parentAtmosphere)). Positions and velocities absolute.
ImpactPrediction predictShuttleImpact3D(const Vec3& rShuttle, const Vec3& vShuttle, const Vec3& rParent,
                                        const Vec3& vParent, double muParent, double parentRadius,
                                        const std::optional<Atmosphere>& parentAtmosphere);

// Body-body pair, 3D: predictParentImpact3D(pb - pa, vb - va, muA + muB, radiusA + radiusB). point relative to a.
ImpactPrediction predictPairImpact3D(const Vec3& pa, const Vec3& va, double muA, double radiusA,
                                     const Vec3& pb, const Vec3& vb, double muB, double radiusB);

} // namespace cosmos
