#include <doctest.h>
#include <cmath>
#include <map>
#include <set>
#include "core/controllers.h"
#include "test_shuttle_util.h"

using namespace cosmos;
using namespace shuttle_test;

// Controller interface, pointing law and the four autopilots (plan 7.2, 7.3), observation by observation.

namespace {
ShuttleObservation obsWith(const Vec3& relVel, double heading, double spin)
{
    ShuttleObservation o;
    o.shuttleId = 17;
    o.tick = 40;
    o.t_sim = 4.0;
    o.relVelocity = relVel;
    o.velocity = relVel;
    o.heading = heading;
    o.spin = spin;
    o.maxTorque = 5000.0;
    o.inertia = 1.0e4;
    o.mass = 5000.0;
    o.dryMass = 2000.0;
    o.fuelMass = 3000.0;
    return o;
}
} // namespace

TEST_SUITE("controllers")
{
    TEST_CASE("controller names parse case-insensitively and round-trip; unknown names are rejected")
    {
        for (ControllerKind k : {ControllerKind::Coast, ControllerKind::Prograde, ControllerKind::Retrograde, ControllerKind::Wander}) {
            ControllerKind out = ControllerKind::Wander;
            CHECK(parseControllerKind(controllerName(k), out));
            CHECK(out == k);
        }
        ControllerKind out = ControllerKind::Coast;
        CHECK(parseControllerKind("rEtRoGrAdE", out));
        CHECK(out == ControllerKind::Retrograde);
        CHECK_FALSE(parseControllerKind("Hover", out));
        CHECK_FALSE(parseControllerKind("", out));
        CHECK(out == ControllerKind::Retrograde); // untouched on failure
    }

    TEST_CASE("wrapAngle maps to (-pi, pi]")
    {
        CHECK(wrapAngle(0.0) == 0.0);
        CHECK(wrapAngle(PI) == PI);
        CHECK(wrapAngle(-PI) == doctest::Approx(PI).epsilon(1e-15));
        CHECK(wrapAngle(3.0 * PI / 2.0) == doctest::Approx(-PI / 2.0).epsilon(1e-15));
        CHECK(wrapAngle(-3.0 * PI / 2.0) == doctest::Approx(PI / 2.0).epsilon(1e-15));
        CHECK(wrapAngle(100.0 * PI + 0.25) == doctest::Approx(0.25).epsilon(1e-12));
        CHECK(wrapAngle(-100.0 * PI - 0.25) == doctest::Approx(-0.25).epsilon(1e-12));
    }

    TEST_CASE("pointing PD: turn = clamp(Kp e - Kd omega, -1, 1) at the default authority")
    {
        const double alpha0 = 5000.0 / 1.0e4;
        CHECK(alpha0 == kPointingAuthority0);
        // 0.1 rad error, at rest
        CHECK(pointingTurn(0.1, 0.0, 0.0, alpha0) == doctest::Approx(kPointingKp * 0.1).epsilon(1e-15));
        // spin damps
        CHECK(pointingTurn(0.1, 0.0, 0.05, alpha0) == doctest::Approx(kPointingKp * 0.1 - kPointingKd * 0.05).epsilon(1e-15));
        // sign follows the error, by the short way round
        CHECK(pointingTurn(-0.1, 0.0, 0.0, alpha0) < 0.0);
        CHECK(pointingTurn(0.1, 2.0 * PI - 0.1, 0.0, alpha0) == doctest::Approx(kPointingKp * 0.2).epsilon(1e-12));
        // saturates
        CHECK(pointingTurn(3.0, 0.0, 0.0, alpha0) == 1.0);
        CHECK(pointingTurn(-3.0, 0.0, 0.0, alpha0) == -1.0);
        CHECK(pointingTurn(0.0, 0.0, 5.0, alpha0) == -1.0);
        // no authority, no command
        CHECK(pointingTurn(1.0, 0.0, 0.0, 0.0) == 0.0);
    }

    TEST_CASE("pointing PD gains are rescaled with the authority so the loop is the same for every shuttle")
    {
        // Four times the authority: a quarter of the gain for the same error
        CHECK(pointingTurn(0.1, 0.0, 0.0, 4.0 * kPointingAuthority0) ==
              doctest::Approx(0.25 * pointingTurn(0.1, 0.0, 0.0, kPointingAuthority0)).epsilon(1e-15));
        CHECK(pointingTurn(0.1, 0.0, 0.02, 0.25 * kPointingAuthority0) ==
              doctest::Approx(4.0 * pointingTurn(0.1, 0.0, 0.02, kPointingAuthority0)).epsilon(1e-15));
    }

    TEST_CASE("Coast: throttle 0, no turn, never asks to be called again")
    {
        CoastController c;
        Rng rng(1);
        ShuttleObservation o = obsWith({1000.0, 0.0, 0.0}, 1.0, 0.3);
        const ShuttleCommand cmd = c.update(o, rng);
        CHECK(cmd.throttle == 0.0);
        CHECK(cmd.turn == 0.0);
        CHECK(c.nextTick(o) == std::numeric_limits<std::int64_t>::max());
    }

    TEST_CASE("Prograde points along the velocity relative to the parent and burns once aligned")
    {
        ProgradeController c;
        Rng rng(1);
        // Velocity along +y (angle pi/2). Aligned: full throttle, no turn
        ShuttleCommand cmd = c.update(obsWith({0.0, 3000.0, 0.0}, PI / 2.0, 0.0), rng);
        CHECK(cmd.throttle == 1.0);
        CHECK(cmd.turn == 0.0);
        // Within the alignment limit: still burns, and turns toward the target
        cmd = c.update(obsWith({0.0, 3000.0, 0.0}, PI / 2.0 - 0.3, 0.0), rng);
        CHECK(cmd.throttle == 1.0);
        CHECK(cmd.turn > 0.0);
        // Beyond it: no burn, turn the short way
        cmd = c.update(obsWith({0.0, 3000.0, 0.0}, PI / 2.0 + 0.5, 0.0), rng);
        CHECK(cmd.throttle == 0.0);
        CHECK(cmd.turn < 0.0);
        // Exactly at the limit angle: off (strictly inside to burn)
        cmd = c.update(obsWith({0.0, 3000.0, 0.0}, PI / 2.0 - kBurnAlignmentLimit, 0.0), rng);
        CHECK(cmd.throttle == 0.0);
        // Opposite: turns (full torque) but does not burn
        cmd = c.update(obsWith({1000.0, 0.0, 0.0}, PI, 0.0), rng);
        CHECK(cmd.throttle == 0.0);
        CHECK(cmd.turn == 1.0); // an error of exactly pi wraps to +pi, and Kp pi saturates
        // the velocity relative to the parent, not the absolute one, sets the direction
        ShuttleObservation o = obsWith({0.0, 3000.0, 0.0}, PI / 2.0, 0.0);
        o.velocity = {-30000.0, 0.0, 0.0};
        CHECK(c.update(o, rng).throttle == 1.0);
        // asks for every tick
        CHECK(c.nextTick(o) == o.tick + 1);
    }

    TEST_CASE("Prograde at error pi saturates the torque; Retrograde is the mirror image")
    {
        ProgradeController pro;
        RetrogradeController retro;
        Rng rng(1);
        // Heading 0, velocity along -x: target is pi away
        ShuttleCommand cmd = pro.update(obsWith({-1000.0, 0.0, 0.0}, 0.0, 0.0), rng);
        CHECK(cmd.throttle == 0.0);
        CHECK(std::fabs(cmd.turn) == 1.0);
        // Retrograde of the same state points along +x (angle 0): aligned, burns
        cmd = retro.update(obsWith({-1000.0, 0.0, 0.0}, 0.0, 0.0), rng);
        CHECK(cmd.throttle == 1.0);
        CHECK(cmd.turn == 0.0);
        // Retrograde target is opposite to the velocity
        cmd = retro.update(obsWith({0.0, 3000.0, 0.0}, -PI / 2.0, 0.0), rng);
        CHECK(cmd.throttle == 1.0);
        cmd = retro.update(obsWith({0.0, 3000.0, 0.0}, PI / 2.0, 0.0), rng);
        CHECK(cmd.throttle == 0.0);
    }

    TEST_CASE("Prograde / Retrograde with no relative motion hold the heading and damp the spin")
    {
        ProgradeController pro;
        Rng rng(1);
        ShuttleCommand cmd = pro.update(obsWith({0.0, 0.0, 0.0}, 0.7, 0.2), rng);
        CHECK(cmd.throttle == 0.0);
        CHECK(cmd.turn < 0.0);
        CHECK(pro.update(obsWith({0.0, 0.0, 0.0}, 0.7, 0.0), rng).turn == 0.0);
    }

    TEST_CASE("the burn autopilots scale the pointing gains with the shuttle's own authority")
    {
        ProgradeController pro;
        Rng rng(1);
        ShuttleObservation weak = obsWith({0.0, 3000.0, 0.0}, PI / 2.0 - 0.1, 0.0);
        weak.inertia = 4.0e4; // a quarter of the authority: four times the gain
        ShuttleObservation normal = obsWith({0.0, 3000.0, 0.0}, PI / 2.0 - 0.1, 0.0);
        CHECK(pro.update(weak, rng).turn == doctest::Approx(4.0 * pro.update(normal, rng).turn).epsilon(1e-12));
        ShuttleObservation none = normal;
        none.maxTorque = 0.0;
        CHECK(pro.update(none, rng).turn == 0.0);
    }

    TEST_CASE("Wander is a pure function of (shuttle id, tick): reproducible, segment-constant, independent of the rng argument")
    {
        WanderController w;
        const WanderParams p;
        ShuttleObservation o;
        o.shuttleId = 1234;
        Rng r1(1), r2(999);
        for (std::int64_t tick : {0, 1, 19, 20, 21, 59, 60, 1000, 123456}) {
            o.tick = tick;
            const ShuttleCommand a = w.update(o, r1);
            const ShuttleCommand b = w.update(o, r2);
            CAPTURE(tick);
            CHECK(a == b);
            CHECK(a == w.commandFor(1234, tick));
            // the whole segment shares the command
            const std::int64_t first = tick / p.segmentTicks * p.segmentTicks;
            CHECK(a == w.commandFor(1234, first));
            CHECK(a == w.commandFor(1234, first + p.segmentTicks - 1));
            // ranges
            CHECK(a.throttle >= 0.0);
            CHECK(a.throttle <= 1.0);
            CHECK(std::fabs(a.turn) <= 1.0);
            // the next call is at the start of the next segment
            CHECK(w.nextTick(o) == first + p.segmentTicks);
        }
        // a different object gives the same answers (no hidden state)
        WanderController w2;
        for (std::int64_t tick = 0; tick < 500; ++tick) CHECK(w.commandFor(77, tick) == w2.commandFor(77, tick));
    }

    TEST_CASE("Wander: shuttles differ, segments differ, burns are short and random with the configured odds")
    {
        WanderController w;
        const WanderParams p;
        int burns = 0, turns = 0;
        std::set<double> throttles;
        const int segments = 4000;
        for (int s = 0; s < segments; ++s) {
            const ShuttleCommand c = w.commandFor(5, static_cast<std::int64_t>(s) * p.segmentTicks);
            if (c.throttle > 0.0) {
                ++burns;
                CHECK(c.throttle >= p.minThrottle);
                throttles.insert(c.throttle);
            }
            if (c.turn != 0.0) ++turns;
        }
        // binomial sigma = sqrt(n p (1-p)) = 31 and 29; allow 5 sigma
        CHECK(std::fabs(burns - p.burnProbability * segments) < 160.0);
        CHECK(std::fabs(turns - p.turnProbability * segments) < 150.0);
        CHECK(throttles.size() > 1000u);
        // two shuttles do not walk in step
        int same = 0;
        for (int s = 0; s < 200; ++s) same += w.commandFor(5, s * p.segmentTicks) == w.commandFor(6, s * p.segmentTicks) ? 1 : 0;
        CHECK(same < 100);
        // A burn is short: the burn segment lasts segmentTicks * tick = 2 s at the default tick
        CHECK(p.segmentTicks * 0.1 <= 2.0 + 1e-12);
    }

    TEST_CASE("ControllerSet serves the four built-in autopilots and accepts replacements")
    {
        ControllerSet cs;
        for (ControllerKind k : {ControllerKind::Coast, ControllerKind::Prograde, ControllerKind::Retrograde, ControllerKind::Wander})
            CHECK(cs.get(k) != nullptr);
        CHECK(cs.get(ControllerKind::Coast) != cs.get(ControllerKind::Wander));
        ConstantController mine(0.5, -0.25);
        cs.set(ControllerKind::Wander, &mine);
        Rng rng(1);
        const ShuttleCommand cmd = cs.get(ControllerKind::Wander)->update(ShuttleObservation{}, rng);
        CHECK(cmd.throttle == 0.5);
        CHECK(cmd.turn == -0.25);
        CHECK(cs.get(ControllerKind::Prograde) != &mine);
    }
}
