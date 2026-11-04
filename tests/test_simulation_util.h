#pragma once
#include <doctest.h>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <vector>
#include "core/events.h"
#include "core/kepler.h"
#include "core/scene.h"
#include "core/simulation.h"
#include "test_scene_util.h"

// Helpers shared by the simulation runner tests (M3).
namespace sim_test {

using namespace cosmos;

inline const double kPi = 3.14159265358979323846;

inline bool sameBits(double a, double b) { return std::memcmp(&a, &b, sizeof a) == 0; }

inline BodyDesc bodyDesc(double mu, double radius, const Vec3& pos, const Vec3& vel)
{
    BodyDesc d;
    d.mu = mu;
    d.radius = radius;
    d.position = pos;
    d.velocity = vel;
    return d;
}

// A resolved scene from explicit bodies (no resolver: positions are taken as given).
inline ResolvedScene sceneOf(const std::vector<BodyDesc>& bodies)
{
    ResolvedScene s;
    for (const BodyDesc& d : bodies) s.bodyIds.push_back(s.bodies.add(d));
    return s;
}

inline ResolvedShuttle resolvedShuttle(std::uint64_t id, const Vec3& r, const Vec3& v, const char* controller = "Coast")
{
    ResolvedShuttle s;
    s.desc.id = id;
    s.desc.name = "S" + std::to_string(id);
    s.desc.controller = controller;
    s.position = r;
    s.velocity = v;
    return s;
}

// Position / velocity of a circular, counter-clockwise orbit of radius r about a body with mu, phase angle phi, plus the
// body's own state.
inline void circularOrbit(double mu, double r, double phi, const Vec3& pBody, const Vec3& vBody, Vec3& pos, Vec3& vel)
{
    const double vc = std::sqrt(mu / r);
    pos = pBody + Vec3{r * std::cos(phi), r * std::sin(phi), 0.0};
    vel = vBody + Vec3{-vc * std::sin(phi), vc * std::cos(phi), 0.0};
}

inline Vec3 bodyPos(const Bodies& b, std::size_t i) { return {b.state.x[i], b.state.y[i], b.state.z[i]}; }
inline Vec3 bodyVel(const Bodies& b, std::size_t i) { return {b.state.vx[i], b.state.vy[i], b.state.vz[i]}; }
inline Vec3 shuttlePos(const Shuttles& s, std::size_t i) { return {s.state.x[i], s.state.y[i], s.state.z[i]}; }
inline Vec3 shuttleVel(const Shuttles& s, std::size_t i) { return {s.state.vx[i], s.state.vy[i], s.state.vz[i]}; }

inline std::size_t countOf(const std::vector<Event>& ev, EventType t)
{
    return static_cast<std::size_t>(std::count_if(ev.begin(), ev.end(), [t](const Event& e) { return e.type == t; }));
}

inline const Event* firstOf(const std::vector<Event>& ev, EventType t, BodyId id = 0)
{
    for (const Event& e : ev)
        if (e.type == t && (id == 0 || e.id == id)) return &e;
    return nullptr;
}

// Bitwise comparison of two doubles ranges.
inline bool sameBits(const std::vector<double>& a, const std::vector<double>& b)
{
    return a.size() == b.size() && (a.empty() || std::memcmp(a.data(), b.data(), a.size() * sizeof(double)) == 0);
}

inline bool sameBits(const BodyState& a, const BodyState& b)
{
    return sameBits(a.x, b.x) && sameBits(a.y, b.y) && sameBits(a.z, b.z) && sameBits(a.vx, b.vx) && sameBits(a.vy, b.vy) &&
           sameBits(a.vz, b.vz);
}

inline bool sameBits(const Event& a, const Event& b)
{
    return a.type == b.type && sameBits(a.t, b.t) && a.id == b.id && a.other == b.other && sameBits(a.relSpeed, b.relSpeed) &&
           sameBits(a.point.x, b.point.x) && sameBits(a.point.y, b.point.y) && sameBits(a.point.z, b.point.z) &&
           a.oldKind == b.oldKind && a.newKind == b.newKind;
}

} // namespace sim_test
