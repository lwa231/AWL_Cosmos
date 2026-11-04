#include <doctest.h>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include "app/presets.h"
#include "core/scene_json.h"
#include "test_app_runcontrol_util.h"

using namespace cosmos;
using namespace cosmos::app;
using namespace rc_test;

namespace {

std::string tempDir(const char* leaf)
{
    const std::filesystem::path p = std::filesystem::temp_directory_path() / (std::string("awl_cosmos_test_") + leaf);
    std::filesystem::remove_all(p);
    return p.string();
}

} // namespace

// "Save current as scene" (plan Q17, 10.3): the state of a run kept as a scene with explicit states.
TEST_SUITE("app runcontrol save")
{
    TEST_CASE("the captured scene has merged bodies as one and destroyed shuttles removed")
    {
        RunControl rc(options());
        std::string err;
        REQUIRE(rc.loadScene(collisionScene(), err));
        rc.driver().warp().set(10.0);
        REQUIRE(rc.start().ok);
        REQUIRE(runUntilMergeAndLoss(rc));

        Scene cap;
        REQUIRE(rc.captureCurrent(cap));   // while Running
        const Simulation& sim = rc.driver().sim();
        CHECK(cap.name == "Collision (saved)");
        CHECK(cap.epoch_s == sim.time());
        CHECK(cap.units == "SI");

        // Two bodies: the merged pair (combined mass) and the bystander
        REQUIRE(cap.bodies.size() == 2);
        CHECK(sim.bodies().size() == 2);
        double muSum = 0.0;
        for (const SceneBody& b : cap.bodies) {
            const auto idx = sim.bodies().find(b.id);
            REQUIRE(idx.has_value());
            CHECK(b.name == sim.bodies().name[*idx]);
            CHECK(sameBits(b.mu, sim.bodies().mu[*idx]));
            CHECK(sameBits(b.radius, sim.bodies().radius[*idx]));
            CHECK(sameBits(b.position, Vec3{sim.bodies().state.x[*idx], sim.bodies().state.y[*idx], sim.bodies().state.z[*idx]}));
            CHECK(sameBits(b.velocity, Vec3{sim.bodies().state.vx[*idx], sim.bodies().state.vy[*idx], sim.bodies().state.vz[*idx]}));
            CHECK_FALSE(b.orbit.has_value());
            muSum += b.mu;
        }
        CHECK(muSum == doctest::Approx(1.0e10 + 1.0e10 + 1.0e12).epsilon(1e-12));   // the merge keeps the total mass
        CHECK(findSceneBody(cap, 3) >= 0);
        CHECK((findSceneBody(cap, 1) >= 0) != (findSceneBody(cap, 2) >= 0));     // one of A, B survives

        // The doomed shuttle is gone, the survivor is there with the fuel it has left
        REQUIRE(cap.shuttles.size() == 1);
        const SceneShuttle& s = cap.shuttles[0];
        CHECK(s.id == 11);
        CHECK(s.name == "Survivor");
        CHECK(s.controller == "Prograde");
        const auto si = sim.shuttles().find(11);
        REQUIRE(si.has_value());
        CHECK(sameBits(s.fuelMass, sim.shuttles().fuelMass[*si]));
        CHECK(s.fuelMass < 3000.0);   // it burned
        CHECK(s.fuelMass > 0.0);
        CHECK(sameBits(s.position, Vec3{sim.shuttles().state.x[*si], sim.shuttles().state.y[*si], sim.shuttles().state.z[*si]}));
        CHECK(sameBits(s.velocity, Vec3{sim.shuttles().state.vx[*si], sim.shuttles().state.vy[*si], sim.shuttles().state.vz[*si]}));
        CHECK(s.dryMass == 2000.0);
        CHECK_FALSE(s.orbit.has_value());

        // The bystander keeps its properties
        const SceneBody& c = cap.bodies[static_cast<std::size_t>(findSceneBody(cap, 3))];
        CHECK(c.atmosphere.has_value());
        CHECK(c.atmosphere->scaleHeight == 5000.0);
        CHECK(c.colour[2] == 0.6f);
        CHECK(c.spinPeriod_h == doctest::Approx(5.0).epsilon(1e-14));
    }

    TEST_CASE("a captured scene resolves back to the same states")
    {
        RunControl rc(options());
        std::string err;
        REQUIRE(rc.loadScene(collisionScene(), err));
        rc.driver().warp().set(10.0);
        REQUIRE(rc.start().ok);
        REQUIRE(runUntilMergeAndLoss(rc));
        runFrames(rc, 4, 0.25);

        Scene cap;
        REQUIRE(rc.captureCurrent(cap));
        ResolvedScene res;
        REQUIRE_MESSAGE(resolveScene(cap, res, err), err);
        CHECK(res.idsPreserved);

        const Simulation& sim = rc.driver().sim();
        REQUIRE(res.bodies.size() == sim.bodies().size());
        for (std::size_t i = 0; i < res.bodies.size(); ++i) {
            const auto j = sim.bodies().find(res.bodies.ids[i]);
            REQUIRE(j.has_value());
            const Bodies &a = res.bodies, &b = sim.bodies();
            CHECK(sameBits(a.state.x[i], b.state.x[*j]));
            CHECK(sameBits(a.state.y[i], b.state.y[*j]));
            CHECK(sameBits(a.state.z[i], b.state.z[*j]));
            CHECK(sameBits(a.state.vx[i], b.state.vx[*j]));
            CHECK(sameBits(a.state.vy[i], b.state.vy[*j]));
            CHECK(sameBits(a.state.vz[i], b.state.vz[*j]));
            CHECK(sameBits(a.mu[i], b.mu[*j]));
            CHECK(sameBits(a.radius[i], b.radius[*j]));
            CHECK(a.name[i] == b.name[*j]);
            CHECK(a.kind[i] == b.kind[*j]);
            CHECK(a.kindLocked[i] == b.kindLocked[*j]);
            CHECK(a.colour[i] == b.colour[*j]);
            // spin period <-> rate: a few ulp
            CHECK(std::fabs(a.spinRate[i] - b.spinRate[*j]) <= 4e-16 * std::fabs(b.spinRate[*j]) + 0.0);
        }
        REQUIRE(res.shuttles.size() == 1);
        const auto si = sim.shuttles().find(11);
        REQUIRE(si.has_value());
        CHECK(sameBits(res.shuttles[0].position.x, sim.shuttles().state.x[*si]));
        CHECK(sameBits(res.shuttles[0].velocity.y, sim.shuttles().state.vy[*si]));

        // And the driver built from it starts exactly there
        std::unique_ptr<SimDriver> d = buildDriver(cap, ResolveOptions{}, options().sim, options().driver, err);
        REQUIRE_MESSAGE(d != nullptr, err);
        CHECK(d->simTime() == cap.epoch_s);
        CHECK(d->state().t == cap.epoch_s);
        for (const DisplayBody& b : d->state().bodies) {
            const auto j = sim.bodies().find(b.id);
            REQUIRE(j.has_value());
            CHECK(sameBits(b.r.x, sim.bodies().state.x[*j]));
        }
    }

    TEST_CASE("the run goes on from a saved scene as it would have")
    {
        RunControl rc(options());
        std::string err;
        REQUIRE(rc.loadScene(lucernaWithShuttles(2), err));
        rc.driver().warp().set(1.0e4);
        REQUIRE(rc.start().ok);
        runFrames(rc, 20);
        Scene cap;
        REQUIRE(rc.captureCurrent(cap));
        const double t0 = cap.epoch_s;

        // Continue the original run, and a second RunControl that starts from the capture
        runFrames(rc, 10);
        RunControl other(options());
        REQUIRE(other.loadScene(cap, err));
        other.driver().warp().set(1.0e4);
        REQUIRE(other.start().ok);
        // Same display time: advance the capture's run to where the first one is
        other.driver().advanceTo(rc.driver().targetTime());
        CHECK(other.driver().targetTime() > t0);

        // The physics is the same, the step grid is not: agreement to far better than a body radius
        const DisplayState& a = rc.driver().state();
        const DisplayState& b = other.driver().state();
        REQUIRE(a.bodies.size() == b.bodies.size());
        for (const DisplayBody& x : a.bodies) {
            const DisplayBody* y = driver_test::findBody(b, x.id);
            REQUIRE(y != nullptr);
            CHECK(norm(x.r - y->r) < 1.0e-3 * x.radius + 1.0e-4);
        }
    }

    TEST_CASE("Paused applies the edits first; Stopped saves the scene as it is, orbits and all")
    {
        RunControl rc(options());
        std::string err;
        const Scene scene = lucernaWithShuttles(1);
        REQUIRE(rc.loadScene(scene, err));

        Scene stopped;
        REQUIRE(rc.captureCurrent(stopped, "My design"));
        CHECK(stopped.name == "My design");
        Scene expect = scene;
        expect.name = "My design";
        CHECK(sameScene(stopped, expect));
        CHECK(stopped.bodies[1].orbit.has_value());   // not flattened

        rc.driver().warp().set(1.0e3);
        REQUIRE(rc.start().ok);
        runFrames(rc, 10);
        REQUIRE(rc.pause().ok);
        const std::uint64_t domus = idOf(rc.editor().scene(), "Domus");
        REQUIRE(rc.editor().setRadius(domus, 7.7e5).ok);
        Scene paused;
        REQUIRE(rc.captureCurrent(paused));
        CHECK(paused.bodies[static_cast<std::size_t>(findSceneBody(paused, domus))].radius == 7.7e5);   // the edit is in
        CHECK_FALSE(rc.dirty());
        for (const SceneBody& b : paused.bodies) CHECK_FALSE(b.orbit.has_value());
        CHECK(paused.name == "Lucerna (saved)");
    }

    TEST_CASE("saved to a file and loaded again, the scene is the same")
    {
        RunControl rc(options());
        std::string err;
        REQUIRE(rc.loadScene(collisionScene(), err));
        rc.driver().warp().set(10.0);
        REQUIRE(rc.start().ok);
        REQUIRE(runUntilMergeAndLoss(rc));

        const std::string dir = tempDir("save_current");
        const std::string path = dir + "/sub/mid_run";   // no extension, folders that do not exist yet
        REQUIRE_MESSAGE(rc.saveCurrent(path, err, "Mid run"), err);
        CHECK(std::filesystem::exists(dir + "/sub/mid_run.json"));

        Scene loaded;
        REQUIRE_MESSAGE(loadSceneFromPath(dir + "/sub/mid_run.json", loaded, err), err);
        Scene cap;
        REQUIRE(rc.captureCurrent(cap, "Mid run"));
        CHECK(sameScene(loaded, cap));   // the JSON round trip keeps every double

        // ... and it starts and runs
        RunControl second(options());
        REQUIRE(second.loadScene(loaded, err));
        REQUIRE(second.start().ok);
        runFrames(second, 5);
        CHECK(second.driver().targetTime() > loaded.epoch_s);
        std::filesystem::remove_all(dir);
    }

    TEST_CASE("saving to an impossible path reports an error")
    {
        RunControl rc(options());
        std::string err;
        REQUIRE(rc.loadScene(lucernaWithShuttles(0), err));
        CHECK_FALSE(rc.saveCurrent("", err));
        CHECK_FALSE(err.empty());

        // A path below a regular file cannot be created
        const std::string dir = tempDir("save_fail");
        std::filesystem::create_directories(dir);
        const std::string blocker = dir + "/file.txt";
        { std::FILE* f = std::fopen(blocker.c_str(), "w"); REQUIRE(f != nullptr); std::fputs("x", f); std::fclose(f); }
        err.clear();
        CHECK_FALSE(rc.saveCurrent(blocker + "/inside/scene.json", err));
        CHECK_FALSE(err.empty());
        std::filesystem::remove_all(dir);
    }
}
