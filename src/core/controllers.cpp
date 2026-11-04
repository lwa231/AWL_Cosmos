#include "core/controllers.h"
#include <algorithm>
#include <cctype>
#include <cmath>

namespace cosmos {

namespace {
const double PI = 3.14159265358979323846;

std::string lower(const std::string& s)
{
    std::string r = s;
    for (char& ch : r) ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
    return r;
}

std::int64_t floorDiv(std::int64_t a, std::int64_t b)
{
    std::int64_t q = a / b;
    if ((a % b != 0) && ((a < 0) != (b < 0))) --q;
    return q;
}
} // namespace

const char* controllerName(ControllerKind k)
{
    switch (k) {
    case ControllerKind::Coast: return "Coast";
    case ControllerKind::Prograde: return "Prograde";
    case ControllerKind::Retrograde: return "Retrograde";
    case ControllerKind::Wander: return "Wander";
    }
    return "Coast";
}

bool parseControllerKind(const std::string& name, ControllerKind& out)
{
    const std::string n = lower(name);
    for (ControllerKind k : {ControllerKind::Coast, ControllerKind::Prograde, ControllerKind::Retrograde, ControllerKind::Wander}) {
        if (n == lower(controllerName(k))) {
            out = k;
            return true;
        }
    }
    return false;
}

double wrapAngle(double a)
{
    if (a > -PI && a <= PI) return a;
    double r = std::fmod(a + PI, 2.0 * PI);
    if (r <= 0.0) r += 2.0 * PI;
    return r - PI; // in (-pi, pi]
}

double pointingTurn(double targetHeading, double heading, double spin, double authority)
{
    if (!(authority > 0.0)) return 0.0;
    const double scale = kPointingAuthority0 / authority;
    const double err = wrapAngle(targetHeading - heading);
    return std::clamp(scale * (kPointingKp * err - kPointingKd * spin), -1.0, 1.0);
}

ShuttleCommand CoastController::update(const ShuttleObservation&, Rng&) { return {0.0, 0.0}; }

ShuttleCommand BurnController::update(const ShuttleObservation& obs, Rng&)
{
    const double authority = obs.inertia > 0.0 ? obs.maxTorque / obs.inertia : 0.0;
    const Vec3& v = obs.relVelocity;
    ShuttleCommand cmd;
    if (!(norm2(v) > 0.0)) {
        // No relative motion: no direction to point at; just stop spinning.
        cmd.turn = pointingTurn(obs.heading, obs.heading, obs.spin, authority);
        return cmd;
    }
    const double target = std::atan2(sign_ * v.y, sign_ * v.x);
    cmd.turn = pointingTurn(target, obs.heading, obs.spin, authority);
    const double err = std::fabs(wrapAngle(target - obs.heading));
    cmd.throttle = err < kBurnAlignmentLimit ? 1.0 : 0.0;
    return cmd;
}

ShuttleCommand WanderController::commandFor(std::uint64_t shuttleId, std::int64_t tick) const
{
    const std::int64_t seg = std::max<std::int64_t>(1, params_.segmentTicks);
    const std::int64_t first = floorDiv(tick, seg) * seg;
    Rng r = Rng::forTick(shuttleId, static_cast<std::uint64_t>(first));
    // Fixed draw order, independent of the outcomes: burn?, throttle, turn?, turn.
    const double pBurn = r.uniform();
    const double thr = r.uniform(params_.minThrottle, 1.0);
    const double pTurn = r.uniform();
    const double trn = r.uniform(-1.0, 1.0);
    ShuttleCommand cmd;
    cmd.throttle = pBurn < params_.burnProbability ? thr : 0.0;
    cmd.turn = pTurn < params_.turnProbability ? trn : 0.0;
    return cmd;
}

ShuttleCommand WanderController::update(const ShuttleObservation& obs, Rng&) { return commandFor(obs.shuttleId, obs.tick); }

std::int64_t WanderController::nextTick(const ShuttleObservation& obs) const
{
    const std::int64_t seg = std::max<std::int64_t>(1, params_.segmentTicks);
    return (floorDiv(obs.tick, seg) + 1) * seg;
}

ControllerSet::ControllerSet() : byKind_{&coast_, &prograde_, &retrograde_, &wander_} {}

} // namespace cosmos
