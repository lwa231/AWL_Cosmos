#include <doctest.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <vector>
#include "core/diagnostics.h"
#include "core/simulation.h"
#include "test_simulation_util.h"

using namespace cosmos;
using namespace sim_test;

// Plan 14 "Figure-eight" and "Pythagorean three-body" run through the whole Simulation runner in point-mass mode (every
// radius 0, so nothing can overlap or merge). The integrator-level versions are in test_ias15_adaptive.cpp and
// test_ias15_nbody.cpp; these check that the runner (adaptive body steps, merge detection, event queue) leaves the
// point-mass dynamics alone.

namespace {
double distance2D(const Vec3& a, const Vec3& b) { return std::hypot(a.x - b.x, a.y - b.y); }
}

TEST_SUITE("validation point-mass runner")
{
    TEST_CASE("Pythagorean three-body through Simulation: mu = 3 escapes, 4 and 5 bind, last close approach near t = 59.4, no merge")
    {
        const ResolvedScene res = sceneOf({bodyDesc(3.0, 0.0, {1.0, 3.0, 0.0}, {}), bodyDesc(4.0, 0.0, {-2.0, -1.0, 0.0}, {}),
                                           bodyDesc(5.0, 0.0, {1.0, -1.0, 0.0}, {})});
        const std::vector<BodyId> ids = res.bodyIds;
        Simulation sim(res, SceneSettings{});
        const Conserved c0 = conserved(sim.bodies());

        double closest = 1e300, closestT = 0.0, maxdE = 0.0;
        long steps = 0;
        while (sim.time() < 100.0 && steps < 300000) {
            sim.advanceTo(std::min(100.0, sim.time() + 1e-9)); // exactly one body step per call
            REQUIRE(!sim.failed());
            ++steps;
            const Conserved c = conserved(sim.bodies());
            maxdE = std::max(maxdE, std::abs((c.energyG - c0.energyG) / c0.energyG));
            if (sim.time() > 50.0 && sim.hasBodyStep()) {
                for (int k = 1; k <= 16; ++k) {
                    const double t = sim.bodyStepStart() + (sim.bodyStepEnd() - sim.bodyStepStart()) * k / 16.0;
                    Vec3 p0, p1, p2;
                    REQUIRE(sim.bodyPositionAt(ids[0], t, p0));
                    REQUIRE(sim.bodyPositionAt(ids[1], t, p1));
                    REQUIRE(sim.bodyPositionAt(ids[2], t, p2));
                    const double d = std::min(distance2D(p0, p1), distance2D(p0, p2));
                    if (d < closest) {
                        closest = d;
                        closestT = t;
                    }
                }
            }
        }
        REQUIRE(sim.time() >= 100.0);
        CHECK(sim.stats().merges == 0);
        CHECK(sim.bodies().size() == 3);

        const Bodies& b = sim.bodies();
        const std::size_t i0 = *b.find(ids[0]), i1 = *b.find(ids[1]), i2 = *b.find(ids[2]);
        auto vel = [&](std::size_t i) { return Vec3{b.state.vx[i], b.state.vy[i], b.state.vz[i]}; };
        auto pos = [&](std::size_t i) { return Vec3{b.state.x[i], b.state.y[i], b.state.z[i]}; };
        // body 0 against the centre of mass of (1, 2): energy per unit reduced mass, + means unbound
        const double mg = b.mu[i1] + b.mu[i2];
        const Vec3 cg = (b.mu[i1] * pos(i1) + b.mu[i2] * pos(i2)) / mg, vg = (b.mu[i1] * vel(i1) + b.mu[i2] * vel(i2)) / mg;
        const double eEscape = 0.5 * norm2(vel(i0) - vg) - (b.mu[i0] + mg) / norm(pos(i0) - cg);
        const double eBinary = 0.5 * norm2(vel(i1) - vel(i2)) - mg / norm(pos(i1) - pos(i2));
        std::printf("[validation] Pythagorean via Simulation: E(0 vs binary) %.4g, E(binary) %.4g, last close approach %.4f at t = %.4f, "
                    "%ld body steps, max|dE/E| %.3g\n", eEscape, eBinary, closest, closestT, steps, maxdE);
        CHECK(eEscape > 1.0);
        CHECK(eBinary < -1.0);
        CHECK(distance2D(pos(i0), pos(i1)) > 50.0);
        CHECK(closestT > 59.3);
        CHECK(closestT < 59.5);
        CHECK(maxdE < 1e-8);
    }

    TEST_CASE("Figure-eight through Simulation: 100 periods on the choreography, energy and angular momentum conserved")
    {
        const double v3x = -0.93240737, v3y = -0.86473146;
        const ResolvedScene res = sceneOf({bodyDesc(1.0, 0.0, {0.97000436, -0.24308753, 0.0}, {-v3x / 2, -v3y / 2, 0.0}),
                                           bodyDesc(1.0, 0.0, {-0.97000436, 0.24308753, 0.0}, {-v3x / 2, -v3y / 2, 0.0}),
                                           bodyDesc(1.0, 0.0, {0.0, 0.0, 0.0}, {v3x, v3y, 0.0})});
        const double period = 6.32591398;
        Simulation sim(res, SceneSettings{});
        const Bodies b0 = sim.bodies();
        const Conserved c0 = conserved(b0);
        double maxdE = 0.0, maxdL = 0.0, maxdev = 0.0;
        for (int k = 1; k <= 100; ++k) {
            // land on a whole period: a fresh Simulation would be needed to hit it exactly, so measure at the first body step end
            // past it and compare with the choreography's position there (time-shifted by the overshoot)
            REQUIRE(sim.advanceTo(k * period));
            REQUIRE(!sim.failed());
            const Conserved c = conserved(sim.bodies());
            maxdE = std::max(maxdE, std::abs((c.energyG - c0.energyG) / c0.energyG));
            maxdL = std::max(maxdL, std::abs(c.angularMomentumG.z - c0.angularMomentumG.z));
            // evaluate each body at exactly k * period from the dense output of the latest step
            double dev = 0.0;
            for (std::size_t i = 0; i < 3; ++i) {
                Vec3 p;
                REQUIRE(sim.bodyPositionAt(b0.ids[i], k * period, p));
                dev = std::max(dev, distance2D(p, bodyPos(b0, i)));
            }
            maxdev = std::max(maxdev, dev);
        }
        std::printf("[validation] figure-eight via Simulation, 100 periods: max|dE/E| %.3g, max|dL| %.3g, max return deviation %.3g\n",
                    maxdE, maxdL, maxdev);
        CHECK(sim.stats().merges == 0);
        CHECK(maxdE < 1e-12);
        CHECK(maxdL < 1e-12);
        CHECK(maxdev < 1e-5);
    }
}
