#include <doctest.h>
#include "test_app_runcontrol_util.h"

using namespace cosmos;
using namespace cosmos::app;
using namespace rc_test;

namespace {

// A run of Lucerna with three shuttles, paused after 20 frames at 1e4x
struct PausedLucerna {
    RunControl rc{options()};
    Scene scene = lucernaWithShuttles(3);
    DisplayState initial;
    PausedLucerna()
    {
        std::string err;
        REQUIRE(rc.loadScene(scene, err));
        rc.driver().warp().set(1.0e4);
        initial = rc.driver().state();
        REQUIRE(rc.start().ok);
        runFrames(rc, 20);
        REQUIRE(rc.pause().ok);
    }
};

} // namespace

// Editing while Paused (plan Q19): the pause captures the run into the editor, applyEdits rebuilds the simulation from the
// edited scene at the time of the pause.
TEST_SUITE("app runcontrol edit")
{
    TEST_CASE("pausing captures the run: explicit states, the time of the pause, no recentring")
    {
        PausedLucerna p;
        RunControl& rc = p.rc;
        const Scene& cap = rc.editor().scene();
        const Simulation& sim = rc.driver().sim();

        CHECK(rc.state() == RunState::Paused);
        CHECK_FALSE(rc.editor().locked());
        CHECK(rc.editor().recentre() == Recentre::Never);
        CHECK(sim.time() > 0.0);
        CHECK(cap.epoch_s == sim.time());
        REQUIRE(cap.bodies.size() == sim.bodies().size());
        REQUIRE(cap.shuttles.size() == 3);
        for (std::size_t i = 0; i < cap.bodies.size(); ++i) {
            const SceneBody& b = cap.bodies[i];
            CHECK_FALSE(b.orbit.has_value());
            const auto idx = sim.bodies().find(b.id);
            REQUIRE(idx.has_value());
            CHECK(sameBits(b.position, Vec3{sim.bodies().state.x[*idx], sim.bodies().state.y[*idx], sim.bodies().state.z[*idx]}));
            CHECK(sameBits(b.mu, sim.bodies().mu[*idx]));
            CHECK(b.name == sim.bodies().name[*idx]);
        }
        for (const SceneShuttle& s : cap.shuttles) CHECK_FALSE(s.orbit.has_value());
        CHECK_FALSE(rc.dirty());   // the driver IS the captured scene
        // The template names are untouched and still consume no counter
        CHECK(rc.editor().namesIssued(BodyKind::Planet) == 0);
    }

    TEST_CASE("an edit while paused rebuilds the simulation at the time of the pause and moves nothing else")
    {
        PausedLucerna p;
        RunControl& rc = p.rc;
        EditorModel& ed = rc.editor();
        const Scene captured = ed.scene();
        const double epoch = captured.epoch_s;
        const std::uint64_t gen = rc.generation();
        const std::uint64_t favilla = idOf(captured, "Favilla");

        const SceneBody* f = ed.body(favilla);
        const Vec3 oldV = f->velocity;
        REQUIRE(ed.setState(favilla, f->position, oldV + Vec3{0.0, 150.0, 0.0}).ok);
        CHECK(rc.dirty());
        CHECK(rc.generation() == gen);   // nothing is rebuilt until applyEdits
        const EditResult applied = rc.applyEdits();
        REQUIRE(applied.ok);
        CHECK_FALSE(rc.dirty());
        CHECK(rc.generation() > gen);

        // The new simulation starts at the epoch of the pause
        CHECK(rc.driver().simTime() == epoch);
        CHECK(rc.driver().targetTime() == epoch);
        CHECK(rc.driver().state().t == epoch);
        REQUIRE(rc.driver().state().bodies.size() == captured.bodies.size());
        for (const DisplayBody& b : rc.driver().state().bodies) {
            const SceneBody* c = ed.body(b.id);
            REQUIRE(c != nullptr);
            CHECK(sameBits(b.r, c->position));   // positions as captured, bit for bit (no recentring shift)
            if (b.id == favilla) {
                CHECK(b.v.y == oldV.y + 150.0);
            } else {
                CHECK(sameBits(b.v, captured.bodies[static_cast<std::size_t>(findSceneBody(captured, b.id))].velocity));
            }
        }
        CHECK(rc.driver().warp().value() == 1.0e4);   // the warp setting is not touched
        CHECK(rc.driver().paused());

        // Resume: it runs on from the epoch
        REQUIRE(rc.start().ok);
        runFrames(rc, 5);
        CHECK(rc.driver().targetTime() > epoch + 1.0e3);
        // Stop returns to the scene as it was when Start was pressed, not to the edited one
        REQUIRE(rc.stop().ok);
        CHECK(driver_test::sameBits(rc.driver().state(), p.initial));
        CHECK(sameScene(rc.editor().scene(), p.scene));
    }

    TEST_CASE("a body added while paused joins the run; the others are not moved")
    {
        PausedLucerna p;
        RunControl& rc = p.rc;
        EditorModel& ed = rc.editor();
        const Scene captured = ed.scene();
        const std::uint64_t domus = idOf(captured, "Domus");

        const EditResult added = ed.addBody(BodyKind::Moon);
        REQUIRE(added.ok);
        REQUIRE(ed.setOrbitAround(added.id, domus, 3.0e7).ok);
        CHECK(ed.body(added.id)->name == "Moon 1");
        REQUIRE(rc.applyEdits().ok);

        const DisplayState& st = rc.driver().state();
        CHECK(st.bodies.size() == captured.bodies.size() + 1);
        const DisplayBody* fresh = driver_test::findBody(st, added.id);
        REQUIRE(fresh != nullptr);
        const DisplayBody* dom = driver_test::findBody(st, domus);
        REQUIRE(dom != nullptr);
        // Placed about Domus (where Domus is in the paused run, not where the template put it) at the requested distance
        CHECK(norm(fresh->r - dom->r) == doctest::Approx(3.0e7).epsilon(1e-9));
        // Recentre::Never: every body of the run is exactly where the pause left it. Domus, the parent of the new moon, is
        // the root of a small system and is placed as (given - jacobi) + jacobi, which can differ by rounding.
        for (const SceneBody& b : captured.bodies) {
            const DisplayBody* d = driver_test::findBody(st, b.id);
            REQUIRE(d != nullptr);
            if (b.id == domus) {
                CHECK(norm(d->r - b.position) < 1e-12 * norm(b.position));
                CHECK(norm(d->v - b.velocity) < 1e-12 * norm(b.velocity));
            } else {
                CHECK(sameBits(d->r, b.position));
                CHECK(sameBits(d->v, b.velocity));
            }
        }
        REQUIRE(rc.start().ok);
        runFrames(rc, 5);
        CHECK(driver_test::findBody(rc.driver().state(), added.id) != nullptr);
    }

    TEST_CASE("deleting and adding shuttles while paused")
    {
        PausedLucerna p;
        RunControl& rc = p.rc;
        EditorModel& ed = rc.editor();
        REQUIRE(ed.remove(901).ok);
        const EditResult sh = ed.addShuttle(ControllerKind::Coast);
        REQUIRE(sh.ok);
        CHECK(ed.shuttle(sh.id)->name == "Shuttle 4");   // Shuttle 1..3 exist
        CHECK(sh.id > 902);                               // IDs are not reused
        const std::uint64_t domus = idOf(ed.scene(), "Domus");
        REQUIRE(ed.setOrbitAround(sh.id, domus, 1.0e6).ok);
        REQUIRE(rc.applyEdits().ok);
        const DisplayState& st = rc.driver().state();
        CHECK(st.shuttles.size() == 3);
        CHECK(driver_test::findShuttle(st, 901) == nullptr);
        CHECK(driver_test::findShuttle(st, 900) != nullptr);
        CHECK(driver_test::findShuttle(st, sh.id) != nullptr);
        REQUIRE(rc.start().ok);
        runFrames(rc, 3);
        CHECK(driver_test::findShuttle(rc.driver().state(), sh.id) != nullptr);
    }

    TEST_CASE("changing a controller while paused takes effect on resume")
    {
        PausedLucerna p;
        RunControl& rc = p.rc;
        REQUIRE(rc.editor().setController(900, ControllerKind::Prograde).ok);
        REQUIRE(rc.applyEdits().ok);
        const auto idx = rc.driver().sim().shuttles().find(900);
        REQUIRE(idx.has_value());
        CHECK(rc.driver().sim().shuttles().controller[*idx] == ControllerKind::Prograde);
    }

    TEST_CASE("an invalid edit never reaches the simulation, an unresolvable scene keeps the old driver")
    {
        PausedLucerna p;
        RunControl& rc = p.rc;
        const std::uint64_t gen = rc.generation();
        EditorModel& ed = rc.editor();
        // The model refuses it, so there is nothing to apply
        CHECK_FALSE(ed.setFieldText(idOf(ed.scene(), "Domus"), Field::Mass, "-3 kg").ok);
        CHECK_FALSE(rc.dirty());
        CHECK(rc.applyEdits().ok);
        CHECK(rc.generation() == gen);

        // A scene the resolver rejects, forced through updateBody-free means: break it behind the model's back via restoreScene
        Scene broken = ed.scene();
        broken.bodies[0].mu = -1.0;
        ed.restoreScene(broken);
        CHECK(rc.dirty());
        const EditResult r = rc.applyEdits();
        CHECK_FALSE(r.ok);
        CHECK_FALSE(r.error.empty());
        CHECK(rc.generation() == gen);   // the old driver stays
        CHECK(rc.lastError() == r.error);
        CHECK_FALSE(rc.start().ok);      // and Start does not resume on a scene that cannot be built
        CHECK(rc.state() == RunState::Paused);
    }

    TEST_CASE("edits made while Stopped apply to the next run")
    {
        RunControl rc(options());
        std::string err;
        REQUIRE(rc.loadScene(lucernaWithShuttles(1), err));
        EditorModel& ed = rc.editor();
        const std::uint64_t domus = idOf(ed.scene(), "Domus");
        REQUIRE(ed.setFieldText(domus, Field::Radius, "700 km").ok);
        CHECK(rc.dirty());
        CHECK(rc.applyEdits().ok);
        const DisplayBody* d = driver_test::findBody(rc.driver().state(), domus);
        REQUIRE(d != nullptr);
        CHECK(d->radius == 7.0e5);

        REQUIRE(ed.setFieldText(domus, Field::Radius, "710 km").ok);
        REQUIRE(rc.start().ok);   // Start applies pending edits itself
        CHECK(driver_test::findBody(rc.driver().state(), domus)->radius == 7.1e5);
        CHECK(rc.snapshot()->bodies[static_cast<std::size_t>(findSceneBody(*rc.snapshot(), domus))].radius == 7.1e5);
        runFrames(rc, 3);
        REQUIRE(rc.stop().ok);
        CHECK(ed.body(domus)->radius == 7.1e5);   // the edited scene is the one Stop returns to
    }

    TEST_CASE("advance applies pending edits while not running")
    {
        RunControl rc(options());
        std::string err;
        REQUIRE(rc.loadScene(lucernaWithShuttles(0), err));
        const std::uint64_t domus = idOf(rc.editor().scene(), "Domus");
        REQUIRE(rc.editor().setRadius(domus, 6.5e5).ok);
        CHECK(rc.dirty());
        CHECK_FALSE(rc.advance(0.1));
        CHECK_FALSE(rc.dirty());
        CHECK(driver_test::findBody(rc.driver().state(), domus)->radius == 6.5e5);
    }
}
