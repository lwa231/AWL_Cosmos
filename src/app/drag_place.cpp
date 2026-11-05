#include "app/drag_place.h"
#include <cmath>
#include "core/hierarchy.h"

namespace cosmos {
namespace app {

namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();

struct Built {
    std::vector<const PlaceBody*> src;   // hierarchy leaf index -> body
    Hierarchy h;
};

Built build(const std::vector<PlaceBody>& bodies, std::uint64_t exclude)
{
    Built b;
    std::vector<Vec3> r, v;
    std::vector<double> mu;
    for (const PlaceBody& p : bodies) {
        if (p.id == exclude && exclude != 0) continue;
        b.src.push_back(&p);
        r.push_back(p.r);
        v.push_back(p.v);
        mu.push_back(p.mu);
    }
    if (!b.src.empty()) b.h = buildHierarchy(r, v, mu);
    return b;
}

int siblingOf(const Hierarchy& h, int node)
{
    const int p = h.nodes[static_cast<std::size_t>(node)].parent;
    if (p < 0) return -1;
    const HierarchyNode& pn = h.nodes[static_cast<std::size_t>(p)];
    return pn.childA == node ? pn.childB : pn.childA;
}

// Laplace SOI with the same inheritance rule as hillRadius: the lighter child of a pair gets d (mu / mu_sibling)^(2/5),
// the heavier inherits its parent's, the root is infinite.
double soiRadius(const Hierarchy& h, int node)
{
    for (;;) {
        const int sib = siblingOf(h, node);
        if (sib < 0) return kInf;
        const HierarchyNode& a = h.nodes[static_cast<std::size_t>(node)];
        const HierarchyNode& s = h.nodes[static_cast<std::size_t>(sib)];
        const int parent = a.parent;
        const HierarchyNode& pn = h.nodes[static_cast<std::size_t>(parent)];
        if (pn.childA != node && s.mu > 0.0) return norm(a.r - s.r) * std::pow(a.mu / s.mu, 0.4);
        node = parent;   // heavier child (childA): inherits
    }
}

int heaviestLeaf(const Hierarchy& h, int node)
{
    while (h.nodes[static_cast<std::size_t>(node)].childA >= 0) node = h.nodes[static_cast<std::size_t>(node)].childA;
    return node;
}

} // namespace

std::vector<PlaceRings> placementRings(const std::vector<PlaceBody>& bodies, std::uint64_t exclude)
{
    std::vector<PlaceRings> out;
    const Built b = build(bodies, exclude);
    out.reserve(b.src.size());
    for (std::size_t i = 0; i < b.src.size(); ++i) {
        PlaceRings ring;
        ring.id = b.src[i]->id;
        ring.centre = b.src[i]->r;
        ring.soi = soiRadius(b.h, static_cast<int>(i));
        ring.hill = hillRadius(b.h, static_cast<int>(i));
        out.push_back(ring);
    }
    return out;
}

PlaceSnap snapPlacement(const std::vector<PlaceBody>& bodies, const Vec3& point, double mu, std::uint64_t exclude, bool retrograde,
                        const Vec3& up)
{
    PlaceSnap s;
    const Built b = build(bodies, exclude);
    if (b.src.empty()) return s;
    HillTable table;
    buildHillTable(b.h, table);
    const int node = shuttleParent(b.h, table, point);
    if (node < 0) return s;
    const HierarchyNode& n = b.h.nodes[static_cast<std::size_t>(node)];
    const PlaceBody& head = *b.src[static_cast<std::size_t>(heaviestLeaf(b.h, node))];
    s.parentId = head.id;
    s.parentR = n.r;
    s.parentV = n.v;
    s.parentMu = n.mu;
    const Vec3 d = point - s.parentR;
    s.distance = norm(d);
    s.velocity = s.parentV;
    s.insideParent = n.childA < 0 && s.distance < head.radius;
    if (s.distance <= 0.0) return s;
    s.circularSpeed = std::sqrt((s.parentMu + mu) / s.distance);
    // Direction: up x d, normalised (prograde = counter-clockwise about `up`). A point on the up axis has no plane; use +x
    // as a fallback so the result is still finite.
    Vec3 dir = cross(up, d);
    double len = norm(dir);
    if (!(len > 0.0)) {
        dir = cross(up, Vec3{1.0, 0.0, 0.0});
        len = norm(dir);
        if (!(len > 0.0)) return s;
    }
    dir = dir / len;
    if (retrograde) dir = dir * -1.0;
    s.velocity = s.parentV + dir * s.circularSpeed;
    return s;
}

Vec3 velocityFromArrow(const PlaceSnap& snap, const Vec3& arrowWorld, double metresPerSpeed)
{
    if (!(metresPerSpeed > 0.0)) return snap.parentV;
    return snap.parentV + arrowWorld / metresPerSpeed;
}

double arrowScaleFor(const PlaceSnap& snap, double lengthWorld)
{
    const double speed = snap.circularSpeed > 0.0 ? snap.circularSpeed : 1.0;
    const double s = lengthWorld / speed;
    return s > 0.0 && std::isfinite(s) ? s : 1.0;
}

double previewHorizon(const PlaceSnap& snap, double orbits)
{
    if (snap.parentId == 0 || !(snap.circularSpeed > 0.0) || !(snap.distance > 0.0)) return 86400.0;
    const double period = 2.0 * 3.14159265358979323846 * snap.distance / snap.circularSpeed;
    return orbits * period;
}

} // namespace app
} // namespace cosmos
