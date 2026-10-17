#include "core/hierarchy.h"
#include <cmath>
#include <limits>

namespace cosmos {

namespace {

int siblingOf(const Hierarchy& h, int node)
{
    const HierarchyNode& p = h.nodes[h.nodes[node].parent];
    return p.childA == node ? p.childB : p.childA;
}

bool isLighterChild(const Hierarchy& h, int node)
{
    const HierarchyNode& p = h.nodes[h.nodes[node].parent];
    return p.childB == node;
}

int depthOf(const Hierarchy& h, int node)
{
    int d = 0;
    while (h.nodes[node].parent != -1) {
        node = h.nodes[node].parent;
        ++d;
    }
    return d;
}

}

double hillRadius(const Hierarchy& h, int node)
{
    const double inf = std::numeric_limits<double>::infinity();
    if (h.nodes[node].parent == -1) return inf;
    if (!isLighterChild(h, node)) return hillRadius(h, h.nodes[node].parent);

    const HierarchyNode& n = h.nodes[node];
    const HierarchyNode& s = h.nodes[siblingOf(h, node)];
    if (n.mu <= 0.0) return 0.0;
    if (s.mu <= 0.0) return inf;
    return norm(n.r - s.r) * std::cbrt(n.mu / (3.0 * s.mu));
}

Hierarchy buildHierarchy(const std::vector<Vec3>& r, const std::vector<Vec3>& v, const std::vector<double>& mu)
{
    Hierarchy h;
    const int n = static_cast<int>(mu.size());
    h.nodes.resize(mu.size());
    for (int i = 0; i < n; ++i) {
        h.nodes[i].mu = mu[i];
        h.nodes[i].r = r[i];
        h.nodes[i].v = v[i];
    }

    std::vector<int> active;
    for (int i = 0; i < n; ++i) active.push_back(i);

    while (active.size() > 1) {
        std::size_t bi = 0, bj = 1;
        double best = norm2(h.nodes[active[0]].r - h.nodes[active[1]].r);
        for (std::size_t i = 0; i < active.size(); ++i) {
            for (std::size_t j = i + 1; j < active.size(); ++j) {
                const double d2 = norm2(h.nodes[active[i]].r - h.nodes[active[j]].r);
                if (d2 < best) {
                    best = d2;
                    bi = i;
                    bj = j;
                }
            }
        }

        const int a = active[bi], b = active[bj];
        const HierarchyNode& na = h.nodes[a];
        const HierarchyNode& nb = h.nodes[b];

        HierarchyNode node;
        const bool bHeavier = nb.mu > na.mu;
        node.childA = bHeavier ? b : a;
        node.childB = bHeavier ? a : b;
        node.mu = na.mu + nb.mu;
        if (node.mu > 0.0) {
            node.r = (na.mu * na.r + nb.mu * nb.r) / node.mu;
            node.v = (na.mu * na.v + nb.mu * nb.v) / node.mu;
        } else {
            node.r = 0.5 * (na.r + nb.r);
            node.v = 0.5 * (na.v + nb.v);
        }

        const int id = static_cast<int>(h.nodes.size());
        h.nodes.push_back(node);
        h.nodes[a].parent = id;
        h.nodes[b].parent = id;

        active.erase(active.begin() + static_cast<std::ptrdiff_t>(bj));
        active.erase(active.begin() + static_cast<std::ptrdiff_t>(bi));
        active.push_back(id);
    }

    h.bodyParent.assign(mu.size(), -1);
    if (active.empty()) return h;
    h.root = active[0];

    for (int i = n; i < static_cast<int>(h.nodes.size()); ++i) {
        HierarchyNode& p = h.nodes[i];
        const double sep = norm(h.nodes[p.childA].r - h.nodes[p.childB].r);
        p.bound = sep < hillRadius(h, i);
    }

    for (int i = 0; i < n; ++i) {
        int g = i;
        while (h.nodes[g].parent != -1) {
            const int p = h.nodes[g].parent;
            if (isLighterChild(h, g) && h.nodes[p].bound) {
                h.bodyParent[i] = siblingOf(h, g);
                break;
            }
            g = p;
        }
    }
    return h;
}

int shuttleParent(const Hierarchy& h, const Vec3& position)
{
    int best = h.root;
    int bestDepth = 0;
    double bestDist = std::numeric_limits<double>::infinity();
    if (best == -1) return -1;

    for (int i = 0; i < static_cast<int>(h.nodes.size()); ++i) {
        if (i == h.root) continue;
        const double d = norm(position - h.nodes[i].r);
        const double rh = hillRadius(h, i);
        if (!std::isfinite(rh) || d >= rh) continue;
        const int depth = depthOf(h, i);
        if (depth > bestDepth || (depth == bestDepth && d < bestDist)) {
            best = i;
            bestDepth = depth;
            bestDist = d;
        }
    }
    return best;
}

} // namespace cosmos
