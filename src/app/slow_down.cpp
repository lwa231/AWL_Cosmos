#include "app/slow_down.h"
#include <cmath>
#include "core/hierarchy.h"
#include "core/impact.h"

namespace cosmos {
namespace app {

namespace {

// The leaf body a hierarchy node stands for: the leaf itself, or the heavier child chain down to a leaf (childA is heavier).
int leafOf(const Hierarchy& h, int node, int bodyCount)
{
    while (node >= bodyCount) node = h.nodes[static_cast<std::size_t>(node)].childA;
    return node;
}

void consider(SlowDownReport& best, const ImpactPrediction& p, double horizon, SlowDownReport::Cause cause, std::uint64_t subject,
              std::uint64_t other)
{
    if (!p.hits() || !(p.t <= horizon) || !(p.t < best.timeToImpact)) return;
    best.cause = cause;
    best.timeToImpact = p.t;
    best.subject = subject;
    best.other = other;
}

} // namespace

double nextLowerWarp(double warp)
{
    Warp w(warp);
    w.slowDownOneLevel();
    return w.value();
}

SlowDownReport predictSoonestImpact(const DisplayState& state, const std::vector<std::uint64_t>& watchedShuttles, bool anyShuttle,
                                    double horizon)
{
    SlowDownReport best;
    if (!(horizon > 0.0)) return best;

    // Bodies that exist (an absorbed one is about to vanish into its survivor)
    std::vector<const DisplayBody*> bodies;
    for (const DisplayBody& b : state.bodies)
        if (!b.absorbed) bodies.push_back(&b);
    const int n = static_cast<int>(bodies.size());
    if (n == 0) return best;

    std::vector<Vec3> r(bodies.size()), v(bodies.size());
    std::vector<double> mu(bodies.size());
    for (std::size_t i = 0; i < bodies.size(); ++i) {
        r[i] = bodies[i]->r;
        v[i] = bodies[i]->v;
        mu[i] = bodies[i]->mu;
    }
    const Hierarchy h = buildHierarchy(r, v, mu);

    // ---- shuttles against their parent body ----
    auto watched = [&](std::uint64_t id) {
        for (std::uint64_t w : watchedShuttles)
            if (w == id) return true;
        return false;
    };
    for (const DisplayShuttle& s : state.shuttles) {
        if (!anyShuttle && !watched(s.id)) continue;
        const int node = shuttleParent(h, s.r);
        if (node < 0) continue;
        const int pi = leafOf(h, node, n);
        if (pi < 0 || pi >= n) continue;
        const DisplayBody& par = *bodies[static_cast<std::size_t>(pi)];
        const ImpactPrediction p = predictShuttleImpact(s.r, s.v, par.r, par.v, par.mu, par.radius, par.atmosphere);
        consider(best, p, horizon, SlowDownReport::Cause::Shuttle, s.id, par.id);
    }

    // ---- body pairs that orbit each other ----
    auto pair = [&](int i, int j) {
        if (i == j || i < 0 || j < 0 || i >= n || j >= n) return;
        const DisplayBody& a = *bodies[static_cast<std::size_t>(i)];
        const DisplayBody& b = *bodies[static_cast<std::size_t>(j)];
        const ImpactPrediction p = predictPairImpact(a.r, a.v, a.mu, a.radius, b.r, b.v, b.mu, b.radius);
        consider(best, p, horizon, SlowDownReport::Cause::BodyPair, a.id, b.id);
    };
    for (int i = 0; i < n; ++i) {
        const int p = h.bodyParent[static_cast<std::size_t>(i)];
        if (p < 0) continue;
        if (p < n) {
            pair(i, p);
        } else {   // a barycentre parent: the body against the leaf bodies of that node
            const HierarchyNode& node = h.nodes[static_cast<std::size_t>(p)];
            pair(i, leafOf(h, node.childA, n));
            pair(i, leafOf(h, node.childB, n));
        }
    }
    return best;
}

SlowDownReport SlowDown::update(SimDriver& driver, const std::vector<std::uint64_t>& watched)
{
    SlowDownReport rep;
    Warp& warp = driver.warp();
    rep.warpBefore = rep.warpAfter = warp.value();
    if (!options.enabled || driver.paused() || !(warp.value() > 1.0)) return rep;

    const double next = nextLowerWarp(warp.value());
    rep = predictSoonestImpact(driver.state(), watched, options.anyShuttle, lookAheadTime(warp.value()));
    rep.warpBefore = rep.warpAfter = warp.value();
    if (rep.cause == SlowDownReport::Cause::None) return rep;

    // Step down one level when the impact is inside the look-ahead of the level we step to (see the file comment)
    if (rep.timeToImpact <= lookAheadTime(next) && warp.slowDownOneLevel()) {
        rep.steppedDown = true;
        rep.warpAfter = warp.value();
    }
    return rep;
}

} // namespace app
} // namespace cosmos
