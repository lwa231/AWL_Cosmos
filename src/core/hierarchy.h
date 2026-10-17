#pragma once
#include <vector>
#include "core/vec3.h"

namespace cosmos {

struct HierarchyNode {
    int childA = -1;   // -1 for a body leaf; childA is the heavier child (ties: the earlier one)
    int childB = -1;
    int parent = -1;
    double mu = 0.0;
    Vec3 r;            // barycentre
    Vec3 v;
    bool bound = true; // internal nodes: the two children are bound (separation inside the node's Hill sphere)
};

struct Hierarchy {
    std::vector<HierarchyNode> nodes;  // [0, n) are the bodies in input order, then barycentre nodes
    int root = -1;
    std::vector<int> bodyParent;       // per body: the node it orbits (a body or a barycentre node), -1 for none
};

// Plan section 15 (2c): closest-pair binary tree with barycentre nodes. Deterministic: ties go to the lowest index.
Hierarchy buildHierarchy(const std::vector<Vec3>& r, const std::vector<Vec3>& v, const std::vector<double>& mu);

// Radius around a node's barycentre inside which things orbit it. A node lighter than its sibling uses
// d * cbrt(mu / (3 mu_sibling)); a heavier one inherits its parent's; the root is infinite.
double hillRadius(const Hierarchy& h, int node);

// The deepest node whose finite Hill sphere contains the point (ties: nearest barycentre); the root (system
// barycentre) if none. The dominant body's infinite sphere counts as the root.
int shuttleParent(const Hierarchy& h, const Vec3& position);

} // namespace cosmos
