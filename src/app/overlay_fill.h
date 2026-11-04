#pragma once
// Fills the M7 body-based overlay arrays of a RenderFrame (plan 9.5): gravity-well sources, atmosphere bubbles and the
// Hill / SOI / Roche circles. GL-free; the renderer (src/render) only sees the filled RenderFrame arrays.
//
// All three append functions read the camera-relative position of body i from frame.bodies[i], so call them AFTER the
// frame's bodies were pushed in the same order as the DisplayState bodies (what main.cpp's fillFrame does). They only
// add to the arrays: the caller clears them (RenderFrame::clearObjects) and decides, per toggle, which to call.
#include "app/render_frame.h"
#include "app/sim_driver.h"
#include "core/vec3.h"
#include <vector>

namespace cosmos {
namespace app {

// The three radii of plan 5.8 for one body, relative to its parent in the closest-pair hierarchy (core/hierarchy.h).
// Zero means "none": no parent, an unbound orbit (e >= 1), or (Roche) a parent that is a barycentre node or has no radius.
struct BodyRadii {
    double hill = 0.0;      // a (1 - e) (m / 3M)^(1/3), a circle around the body
    double soi = 0.0;       // a (m / M)^(2/5), a circle around the body
    double roche = 0.0;     // 2.44 R_p (rho_p / rho_m)^(1/3), a circle around the PARENT (the limit for this body as satellite)
    int parent = -1;        // hierarchy node orbited (a body index when < the body count, else a barycentre node), -1: none
    int rocheCentre = -1;   // body index the Roche circle is centred on, -1: none
};

// Per-body radii from SI state. a and e come from the osculating elements (core/kepler.h) of the body about its parent
// node with mu = mu_parent + mu_body; M and m in the formulas are the parent node's and the body's mu.
// Cost: the hierarchy build is O(n^3) in the body count (fine for ~100 bodies per frame; throttle it for thousands).
std::vector<BodyRadii> computeBodyRadii(const std::vector<Vec3>& r, const std::vector<Vec3>& v,
                                        const std::vector<double>& mu, const std::vector<double>& radius);
std::vector<BodyRadii> computeBodyRadii(const std::vector<DisplayBody>& bodies);

struct CircleSet {
    bool hill = true;
    bool soi = true;
    bool roche = true;
};

// Appends one RenderCircle per non-zero radius of the chosen kinds (Hill and SOI centred on the body, Roche on its parent).
void appendCircles(RenderFrame& frame, const std::vector<BodyRadii>& radii, const CircleSet& which = CircleSet());

// One well per body with mu > 0 (plan 9.5: radius R_j is the physical radius).
void appendWells(RenderFrame& frame, const std::vector<DisplayBody>& bodies);

// One bubble per body with an atmosphere whose top (plan 5.5, atmosphereTop) is above the surface. Tinted by the body colour.
void appendAtmospheres(RenderFrame& frame, const std::vector<DisplayBody>& bodies);

} // namespace app
} // namespace cosmos
