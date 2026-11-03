#pragma once
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include "core/bodies.h"
#include "core/gravity.h"
#include "core/ias15.h"
#include "core/kepler.h"

// Helpers of the impact tests: build states from orbital elements and find the true crossing time by brute-force
// numerical integration (adaptive IAS15, then bisection on its dense output).
namespace impact_test {

using namespace cosmos;

inline const double kPi = 3.141592653589793238462643383279502884;

// SplitMix64 (no global RNG): platform-independent uniform numbers for generated test data.
struct Mix {
    std::uint64_t s;
    std::uint64_t next()
    {
        std::uint64_t z = (s += 0x9E3779B97F4A7C15ull);
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
        return z ^ (z >> 31);
    }
    double uniform() { return double(next() >> 11) * (1.0 / 9007199254740992.0); }
    double range(double lo, double hi) { return lo + (hi - lo) * uniform(); }
};

inline BodyDesc point(double mu, const Vec3& p, const Vec3& v, double radius = 0.0)
{
    BodyDesc d;
    d.mu = mu;
    d.radius = radius;
    d.position = p;
    d.velocity = v;
    return d;
}

// Relative state from elements (kepler.h, the inverse of stateToElements)
inline RelState stateFrom(double a, double e, double omega, double M, bool retrograde, double mu)
{
    Elements el;
    el.a = a;
    el.e = e;
    el.omega = omega;
    el.M = M;
    el.retrograde = retrograde;
    return elementsToState(el, mu);
}

inline double separation(const BodyState& s, std::size_t i, std::size_t j)
{
    return std::sqrt((s.x[j] - s.x[i]) * (s.x[j] - s.x[i]) + (s.y[j] - s.y[i]) * (s.y[j] - s.y[i]) +
                     (s.z[j] - s.z[i]) * (s.z[j] - s.z[i]));
}

// First time (from 0) the distance between bodies i and j drops to R, found by integrating the whole store with
// IAS15 (epsilon as given) until a step ends inside R and then bisecting the step's dense output. nullopt if it does
// not happen before tMax. 0 if already inside.
inline std::optional<double> bruteCrossing(const Bodies& b, std::size_t i, std::size_t j, double R, double tMax,
                                           double epsilon = 1e-11)
{
    BodyState s = b.state;
    if (separation(s, i, j) <= R) return 0.0;
    IAS15Params prm;
    prm.epsilon = epsilon;
    IAS15 ias(prm);
    const ForceFn force = [&b](const BodyState& st, Accel& a) { computeAccelerations(st, b.mu, a); };
    double t = 0.0;
    while (t < tMax) {
        const double dt = ias.step(s, force, t, tMax);
        if (!(dt > 0.0)) return std::nullopt;
        if (separation(s, i, j) <= R) {
            double lo = 0.0, hi = 1.0; // tau; the distance is > R at lo and <= R at hi
            for (int k = 0; k < 100; ++k) {
                const double mid = 0.5 * (lo + hi);
                const Vec3 d = ias.positionOf(j, mid) - ias.positionOf(i, mid);
                (norm(d) <= R ? hi : lo) = mid;
            }
            return ias.stepStartTime() + 0.5 * (lo + hi) * ias.lastDt();
        }
    }
    return std::nullopt;
}

// Massless test particle with relative state (r, v) around a parent of the given mu at the origin.
inline std::optional<double> bruteParentCrossing(const RelState& rs, double mu, double rTrig, double tMax,
                                                 double epsilon = 1e-11)
{
    Bodies b;
    b.add(point(mu, {0, 0, 0}, {0, 0, 0}));
    b.add(point(0.0, rs.r, rs.v));
    return bruteCrossing(b, 0, 1, rTrig, tMax, epsilon);
}

inline double relDiff(double x, double ref) { return std::abs(x - ref) / std::abs(ref); }

} // namespace impact_test
