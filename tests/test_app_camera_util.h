#pragma once
// Shared fixtures for the camera tests: a tiny scripted world with linearly moving bodies and shuttles that can
// vanish (merge or be destroyed) at a chosen time. Header-only, GL-free.
#include "app/camera.h"
#include <cstdint>
#include <map>

namespace camtest {

using cosmos::Vec3;
using cosmos::app::CameraSources;

struct Mover {
    Vec3 p0, v;
    double dies = 1e300;    // the object does not exist for t >= dies
    Vec3 at(double t) const { return {p0.x + v.x * t, p0.y + v.y * t, 0.0}; }
};

struct FakeWorld {
    std::map<std::uint64_t, Mover> bodies;
    std::map<std::uint64_t, Mover> shuttles;
    std::map<std::uint64_t, std::uint64_t> survivor;    // absorbed -> survivor
    Mover bary;
    bool hasBary = true;

    CameraSources sources() const
    {
        CameraSources s;
        s.bodyPosition = [this](std::uint64_t id, double t, Vec3& out) {
            auto it = bodies.find(id);
            if (it == bodies.end() || t >= it->second.dies) return false;
            out = it->second.at(t);
            return true;
        };
        s.bodyVelocity = [this](std::uint64_t id, double t, Vec3& out) {
            auto it = bodies.find(id);
            if (it == bodies.end() || t >= it->second.dies) return false;
            out = it->second.v;
            return true;
        };
        s.shuttlePosition = [this](std::uint64_t id, double t, Vec3& out) {
            auto it = shuttles.find(id);
            if (it == shuttles.end() || t >= it->second.dies) return false;
            out = it->second.at(t);
            return true;
        };
        s.barycentre = [this](double t, Vec3& out) {
            if (!hasBary) return false;
            out = bary.at(t);
            return true;
        };
        s.barycentreVelocity = [this](double, Vec3& out) {
            if (!hasBary) return false;
            out = bary.v;
            return true;
        };
        s.mergeSurvivor = [this](std::uint64_t id) -> std::uint64_t {
            auto it = survivor.find(id);
            return it == survivor.end() ? 0 : it->second;
        };
        return s;
    }
};

} // namespace camtest
