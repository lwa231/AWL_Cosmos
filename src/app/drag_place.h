#pragma once
// Drag-to-place (V1.1 plan step 4). GL-free: the main loop feeds it the bodies' current states and the cursor's world
// point; it answers which body the dragged object would orbit (the smallest sphere of influence / Hill sphere that
// contains the point, the same rule shuttles use, core/hierarchy.h) and the velocity of a circular orbit there, or the
// velocity a Shift-drag arrow describes.
//
// Coordinates are world metres. 2D for now: the circular velocity lies in the x-y plane (prograde = counter-clockwise
// seen from +z). The 3D placement UI (step 5) passes points with z != 0; the parent choice already works in 3D, the
// circular direction then uses the plane normal `up` (default +z).
#include <cstdint>
#include <limits>
#include <vector>
#include "core/vec3.h"

namespace cosmos {
namespace app {

struct PlaceBody {
    std::uint64_t id = 0;
    Vec3 r;
    Vec3 v;
    double mu = 0.0;
    double radius = 0.0;
};

// One body's spheres for the drag overlay. Infinite radii (the dominant body) are reported as +inf; draw nothing then.
struct PlaceRings {
    std::uint64_t id = 0;
    Vec3 centre;
    double soi = std::numeric_limits<double>::infinity();    // Laplace sphere of influence, d (mu / mu_parent)^(2/5)
    double hill = std::numeric_limits<double>::infinity();   // Hill radius from the hierarchy
};

struct PlaceSnap {
    std::uint64_t parentId = 0;   // the body to orbit, 0 when there are no bodies
    Vec3 parentR;                 // where the orbited mass is (the body, or the barycentre node it heads)
    Vec3 parentV;
    double parentMu = 0.0;        // mu of what is orbited (the node's total mu for a barycentre node)
    double distance = 0.0;        // |point - parentR|
    double circularSpeed = 0.0;   // sqrt((parentMu + mu) / distance), relative to the parent
    Vec3 velocity;                // world velocity of the circular orbit (parentV + circular)
    bool insideParent = false;    // the point is inside the parent's radius (cannot be placed there)
};

// Rings of every body except `exclude` (the dragged one, which must not attract itself).
std::vector<PlaceRings> placementRings(const std::vector<PlaceBody>& bodies, std::uint64_t exclude = 0);

// Parent and circular velocity for an object of gravitational parameter `mu` (0 for a shuttle) dropped at `point`.
// `exclude` is the dragged body's own id (0 for a new object). `retrograde` flips the direction.
PlaceSnap snapPlacement(const std::vector<PlaceBody>& bodies, const Vec3& point, double mu = 0.0, std::uint64_t exclude = 0,
                        bool retrograde = false, const Vec3& up = Vec3{0.0, 0.0, 1.0});

// Shift-drag: a velocity arrow drawn from the object's position. `arrowWorld` is the arrow's tip minus its base in world
// metres; `metresPerSpeed` is how many metres of arrow mean 1 m/s relative to the parent (the UI chooses it so a
// circular-speed arrow has a comfortable length, see arrowScaleFor). Result: parent velocity + arrow / metresPerSpeed.
Vec3 velocityFromArrow(const PlaceSnap& snap, const Vec3& arrowWorld, double metresPerSpeed);
// Arrow scale such that the circular speed is drawn as `lengthWorld` metres (e.g. 80 px * metres per pixel). Never 0.
double arrowScaleFor(const PlaceSnap& snap, double lengthWorld);

// Time for the predicted path: `orbits` periods of the circular orbit (a fallback of one day when there is no parent).
double previewHorizon(const PlaceSnap& snap, double orbits = 3.0);

} // namespace app
} // namespace cosmos
