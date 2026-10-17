#include "core/placement.h"
#include <algorithm>
#include "core/kepler.h"

namespace cosmos {

namespace {

struct Member {
    int index;
    Vec3 r;
    Vec3 v;
};

struct System {
    std::vector<Member> members;
    double mu = 0.0;
};

System build(int b, const std::vector<PlacementBody>& bodies, const std::vector<std::vector<int>>& children)
{
    System s;
    s.members.push_back({b, {}, {}});
    s.mu = bodies[b].mu;

    for (int c : children[b]) {
        System sub = build(c, bodies, children);

        Elements el;
        el.a = bodies[c].orbit.a;
        el.e = bodies[c].orbit.e;
        el.omega = bodies[c].orbit.omega;
        el.M = bodies[c].orbit.M;
        el.retrograde = bodies[c].orbit.retrograde;
        const RelState rel = elementsToState(el, s.mu + sub.mu);

        const double total = s.mu + sub.mu;
        const double wOld = sub.mu / total;
        const double wNew = s.mu / total;

        for (Member& m : s.members) {
            m.r -= wOld * rel.r;
            m.v -= wOld * rel.v;
        }
        for (Member& m : sub.members) {
            m.r += wNew * rel.r;
            m.v += wNew * rel.v;
            s.members.push_back(m);
        }
        s.mu = total;
    }
    return s;
}

}

std::vector<Placed> placeJacobi(const std::vector<PlacementBody>& bodies)
{
    const int n = static_cast<int>(bodies.size());
    int root = -1;
    std::vector<std::vector<int>> children(bodies.size());

    for (int i = 0; i < n; ++i) {
        const int p = bodies[i].parent;
        if (p == -1) {
            if (root != -1) return {};
            root = i;
        } else if (p < 0 || p >= n || p == i) {
            return {};
        } else {
            children[p].push_back(i);
        }
    }
    if (root == -1) return {};

    for (auto& list : children) {
        std::stable_sort(list.begin(), list.end(),
                         [&](int x, int y) { return bodies[x].orbit.a < bodies[y].orbit.a; });
    }

    const System sys = build(root, bodies, children);
    if (static_cast<int>(sys.members.size()) != n) return {};

    std::vector<Placed> out(bodies.size());
    for (const Member& m : sys.members) out[m.index] = {m.r, m.v};
    return out;
}

} // namespace cosmos
