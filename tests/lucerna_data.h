#pragma once
#include <vector>
#include "core/placement.h"

namespace lucerna_data {

constexpr double kDegToRad = 3.141592653589793238462643383279502884 / 180.0;

inline cosmos::PlacementBody root(double mu)
{
    cosmos::PlacementBody b;
    b.mu = mu;
    return b;
}

inline cosmos::PlacementBody child(double mu, int parent, double a, double e, double omegaDeg, double MDeg, bool retro = false)
{
    cosmos::PlacementBody b;
    b.mu = mu;
    b.parent = parent;
    b.orbit = {a, e, omegaDeg * kDegToRad, MDeg * kDegToRad, retro};
    return b;
}

inline std::vector<cosmos::PlacementBody> lucerna()
{
    return {
        root(1.25e18),
        child(1.79e11, 0, 5.0e9, 0.08, 77, 12),
        child(5.45e12, 0, 9.4e9, 0.008, 131, 205),
        child(3.767e12, 0, 1.45e10, 0.015, 103, 35),
        child(4.85e10, 3, 7.6e6, 0.01, 45, 290),
        child(1.47e9, 3, 4.5e7, 0.03, 210, 75),
        child(3.70e11, 0, 2.15e10, 0.06, 286, 140),
        child(3.1e7, 6, 9.0e6, 0.07, 30, 300, true),
        child(3.05e14, 0, 5.8e10, 0.035, 15, 255),
        child(1.98e11, 8, 2.5e7, 0.03, 220, 40),
        child(3.53e11, 8, 5.6e7, 0.01, 300, 170),
        child(1.69e12, 8, 2.2e8, 0.02, 60, 285),
        child(5.03e10, 0, 1.27e11, 0.12, 113, 330),
        child(6.0e9, 12, 2.6e6, 0.0, 0, 90),
    };
}


}
