#pragma once
// Fixtures for the overlay tests (M7): a star, a planet circling it and a moon circling the planet, plus helpers.
#include <doctest.h>
#include <cmath>
#include <cstdint>
#include "app/overlays.h"
#include "app/sim_driver.h"
#include "test_app_driver_util.h"

namespace ovl_test {

using namespace cosmos;
using namespace cosmos::app;

inline const double kPi = 3.14159265358979323846;
constexpr double kStarMu = 1.32712440018e20;
constexpr double kPlanetMu = 3.986004418e14;
constexpr double kMoonMu = 4.9e12;
constexpr double kPlanetA = 1.496e11;
constexpr double kMoonA = 3.844e8;

struct SunPlanetMoon {
    ResolvedScene res;
    std::uint64_t star = 0, planet = 0, moon = 0;
    double planetPeriod = 0.0, moonPeriod = 0.0;

    // A circular planet orbit about the star and a circular moon orbit about the planet (counter-clockwise), the star at
    // rest at the origin (so the planet really moves: 29.8 km/s). `shuttles` extra shuttle can be added by the caller.
    SunPlanetMoon()
    {
        BodyDesc s;
        s.name = "Star";
        s.kind = BodyKind::Star;
        s.mu = kStarMu;
        s.radius = 6.96e8;
        BodyDesc p;
        p.name = "Planet";
        p.mu = kPlanetMu;
        p.radius = 6.371e6;
        p.position = {kPlanetA, 0.0, 0.0};
        p.velocity = {0.0, std::sqrt(kStarMu / kPlanetA), 0.0};
        BodyDesc m;
        m.name = "Moon";
        m.kind = BodyKind::Moon;
        m.mu = kMoonMu;
        m.radius = 1.737e6;
        m.position = p.position + Vec3{kMoonA, 0.0, 0.0};
        m.velocity = p.velocity + Vec3{0.0, std::sqrt((kPlanetMu + kMoonMu) / kMoonA), 0.0};
        star = res.bodies.add(s);
        planet = res.bodies.add(p);
        moon = res.bodies.add(m);
        res.bodyIds = {star, planet, moon};
        planetPeriod = 2.0 * kPi * std::sqrt(kPlanetA * kPlanetA * kPlanetA / kStarMu);
        moonPeriod = 2.0 * kPi * std::sqrt(kMoonA * kMoonA * kMoonA / (kPlanetMu + kMoonMu));
    }
};

inline const DisplayBody* bodyOf(const DisplayState& st, std::uint64_t id)
{
    for (const DisplayBody& b : st.bodies)
        if (b.id == id) return &b;
    return nullptr;
}

inline OverlayView viewAt(const Vec3& centre, double mpp)
{
    OverlayView v;
    v.metresPerPixel = mpp;
    v.centre = centre;
    return v;
}

inline RenderFrame frameOf(int w = 1280, int h = 720)
{
    RenderFrame f;
    f.viewportWidth = w;
    f.viewportHeight = h;
    return f;
}

} // namespace ovl_test
