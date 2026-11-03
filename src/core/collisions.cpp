#include "core/collisions.h"
#include <algorithm>
#include <cmath>

namespace cosmos {

std::optional<double> sweptContactTime(const Vec3& p0, const Vec3& w, double R, double dt)
{
    const double A = dot(w, w);
    const double B = dot(p0, w);
    const double C = dot(p0, p0) - R * R;

    if (C <= 0.0) return 0.0; // already overlapping: collide now
    if (!(B < 0.0)) return std::nullopt; // not approaching (also covers w = 0)
    const double disc = B * B - A * C;
    if (!(disc >= 0.0)) return std::nullopt; // closest approach is outside R
    const double t = C / (-B + std::sqrt(disc));
    if (t <= dt) return t;
    return std::nullopt;
}

std::optional<double> sweptContactTimeBetween(const Vec3& p0, const Vec3& p1, double R, double dt)
{
    if (!(dt > 0.0)) return sweptContactTime(p0, Vec3{}, R, 0.0);
    return sweptContactTime(p0, (p1 - p0) / dt, R, dt);
}

namespace {
void sortContacts(std::vector<BodyContact>& c)
{
    std::sort(c.begin(), c.end(), [](const BodyContact& l, const BodyContact& r) {
        if (l.t != r.t) return l.t < r.t;
        if (l.a != r.a) return l.a < r.a;
        return l.b < r.b;
    });
}
}

std::vector<BodyContact> findBodyContacts(const Bodies& bodies, double dt)
{
    const BodyState& s = bodies.state;
    const std::size_t n = bodies.size();
    std::vector<BodyContact> out;
    for (std::size_t i = 0; i < n; ++i) {
        for (std::size_t j = i + 1; j < n; ++j) {
            const Vec3 p0{s.x[j] - s.x[i], s.y[j] - s.y[i], s.z[j] - s.z[i]};
            const Vec3 w{s.vx[j] - s.vx[i], s.vy[j] - s.vy[i], s.vz[j] - s.vz[i]};
            const double R = bodies.radius[i] + bodies.radius[j];
            if (auto t = sweptContactTime(p0, w, R, dt)) out.push_back({i, j, bodies.ids[i], bodies.ids[j], *t});
        }
    }
    sortContacts(out);
    return out;
}

std::vector<BodyContact> findBodyContacts(const Bodies& bodies, const BodyState& end, double dt)
{
    const BodyState& s = bodies.state;
    const std::size_t n = bodies.size();
    std::vector<BodyContact> out;
    for (std::size_t i = 0; i < n; ++i) {
        for (std::size_t j = i + 1; j < n; ++j) {
            const Vec3 p0{s.x[j] - s.x[i], s.y[j] - s.y[i], s.z[j] - s.z[i]};
            const Vec3 p1{end.x[j] - end.x[i], end.y[j] - end.y[i], end.z[j] - end.z[i]};
            const double R = bodies.radius[i] + bodies.radius[j];
            if (auto t = sweptContactTimeBetween(p0, p1, R, dt)) out.push_back({i, j, bodies.ids[i], bodies.ids[j], *t});
        }
    }
    sortContacts(out);
    return out;
}

} // namespace cosmos
