#include "app/overlay_fill.h"
#include "core/atmosphere.h"
#include "core/hierarchy.h"
#include "core/kepler.h"
#include "core/radii.h"
#include <algorithm>
#include <cmath>

namespace cosmos {
namespace app {

std::vector<BodyRadii> computeBodyRadii(const std::vector<Vec3>& r, const std::vector<Vec3>& v,
                                        const std::vector<double>& mu, const std::vector<double>& radius)
{
    const std::size_t n = mu.size();
    std::vector<BodyRadii> out(n);
    if (n < 2 || r.size() != n || v.size() != n || radius.size() != n) return out;

    const Hierarchy h = buildHierarchy(r, v, mu);
    for (std::size_t i = 0; i < n; ++i) {
        BodyRadii& o = out[i];
        const int p = h.bodyParent[i];
        o.parent = p;
        if (p < 0 || !(mu[i] > 0.0)) continue;
        const HierarchyNode& pn = h.nodes[static_cast<std::size_t>(p)];
        if (!(pn.mu > 0.0)) continue;

        const Elements el = stateToElements(r[i] - pn.r, v[i] - pn.v, pn.mu + mu[i]);
        if (std::isfinite(el.a) && std::isfinite(el.e) && el.a > 0.0 && el.e < 1.0) {   // a bound ellipse
            o.hill = cosmos::hillRadius(el.a, el.e, mu[i], pn.mu);
            o.soi = soiRadius(el.a, mu[i], pn.mu);
        }

        // Roche needs a physical primary: a body (index < n), not a barycentre node
        if (static_cast<std::size_t>(p) < n && radius[static_cast<std::size_t>(p)] > 0.0 && radius[i] > 0.0 &&
            mu[static_cast<std::size_t>(p)] > 0.0) {
            const std::size_t pi = static_cast<std::size_t>(p);
            o.roche = rocheLimit(radius[pi], meanDensity(mu[pi], radius[pi]), meanDensity(mu[i], radius[i]));
            o.rocheCentre = p;
        }
    }
    return out;
}

std::vector<BodyRadii> computeBodyRadii(const std::vector<DisplayBody>& bodies)
{
    std::vector<Vec3> r, v;
    std::vector<double> mu, radius;
    for (const DisplayBody& b : bodies) {
        r.push_back(b.r);
        v.push_back(b.v);
        mu.push_back(b.mu);
        radius.push_back(b.radius);
    }
    return computeBodyRadii(r, v, mu, radius);
}

void appendCircles(RenderFrame& frame, const std::vector<BodyRadii>& radii, const CircleSet& which)
{
    const std::size_t n = std::min(radii.size(), frame.bodies.size());
    auto add = [&frame](const RenderBody& centre, double radius, CircleKind kind) {
        if (!(radius > 0.0) || !std::isfinite(radius) || radius > 1e30) return;
        RenderCircle c;
        c.x = centre.x;
        c.y = centre.y;
        c.radius = static_cast<float>(radius);
        c.kind = kind;
        frame.circles.push_back(c);
    };
    for (std::size_t i = 0; i < n; ++i) {
        const BodyRadii& b = radii[i];
        if (which.hill) add(frame.bodies[i], b.hill, CircleKind::Hill);
        if (which.soi) add(frame.bodies[i], b.soi, CircleKind::Soi);
        if (which.roche && b.rocheCentre >= 0 && static_cast<std::size_t>(b.rocheCentre) < frame.bodies.size())
            add(frame.bodies[static_cast<std::size_t>(b.rocheCentre)], b.roche, CircleKind::Roche);
    }
}

void appendWells(RenderFrame& frame, const std::vector<DisplayBody>& bodies)
{
    const std::size_t n = std::min(bodies.size(), frame.bodies.size());
    for (std::size_t i = 0; i < n; ++i) {
        if (!(bodies[i].mu > 0.0)) continue;
        RenderWell w;
        w.x = frame.bodies[i].x;
        w.y = frame.bodies[i].y;
        w.radius = static_cast<float>(bodies[i].radius);
        w.mu = static_cast<float>(bodies[i].mu);
        frame.wells.push_back(w);
    }
}

void appendAtmospheres(RenderFrame& frame, const std::vector<DisplayBody>& bodies)
{
    const std::size_t n = std::min(bodies.size(), frame.bodies.size());
    for (std::size_t i = 0; i < n; ++i) {
        if (!bodies[i].atmosphere) continue;
        const double top = atmosphereTop(*bodies[i].atmosphere);
        if (!(top > 0.0) || !(bodies[i].atmosphere->scaleHeight > 0.0)) continue;
        RenderAtmosphere a;
        a.x = frame.bodies[i].x;
        a.y = frame.bodies[i].y;
        a.radius = static_cast<float>(bodies[i].radius);
        a.top = static_cast<float>(top);
        a.scaleHeight = static_cast<float>(bodies[i].atmosphere->scaleHeight);
        // Tint: the body colour pulled toward sky blue (a nuance the plan leaves open: "tinted per body")
        const std::array<float, 3>& c = bodies[i].colour;
        a.tint = {0.6f * c[0] + 0.4f * 0.45f, 0.6f * c[1] + 0.4f * 0.70f, 0.6f * c[2] + 0.4f * 1.0f};
        frame.atmospheres.push_back(a);
    }
}

const std::vector<BodyRadii>& RadiiCache::get(const std::vector<DisplayBody>& bodies, std::uint64_t generation, double realNow)
{
    bool same = generation == generation_ && bodies.size() == ids_.size();
    for (std::size_t i = 0; same && i < bodies.size(); ++i) same = bodies[i].id == ids_[i];
    // A clock that went backwards (a new run) also forces a recomputation.
    if (!same || realNow - computedAt_ >= minRealGap || realNow < computedAt_) {
        radii_ = computeBodyRadii(bodies);
        ids_.resize(bodies.size());
        for (std::size_t i = 0; i < bodies.size(); ++i) ids_[i] = bodies[i].id;
        generation_ = generation;
        computedAt_ = realNow;
        ++computations_;
    }
    return radii_;
}

void RadiiCache::reset()
{
    radii_.clear();
    ids_.clear();
    generation_ = ~std::uint64_t(0);
    computedAt_ = -1.0e300;
}

} // namespace app
} // namespace cosmos
