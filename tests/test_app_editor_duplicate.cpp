#include <doctest.h>
#include <cmath>
#include "app/editor_model.h"
#include "test_app_editor_util.h"

using namespace cosmos;
using namespace cosmos::app;
using namespace editor_test;

// V1.1 steps 1 and 2: the user-edit counter behind "unsaved changes", and Duplicate of the object table's row menu.
TEST_SUITE("app editor duplicate")
{
    TEST_CASE("editCount moves on user edits only")
    {
        EditorModel m;
        const std::uint64_t e0 = m.editCount();
        const std::uint64_t star = m.addBody(BodyKind::Star).id;
        CHECK(m.editCount() == e0 + 1);
        CHECK(m.setName(star, "Sol").ok);
        CHECK(m.editCount() == e0 + 2);
        CHECK_FALSE(m.setRadius(star, -1.0).ok);   // a refused edit changes nothing
        CHECK(m.editCount() == e0 + 2);
        m.select(star);
        m.clearSelection();
        CHECK(m.editCount() == e0 + 2);
        const std::uint64_t before = m.editCount();
        m.restoreScene(m.scene());   // Pause / Stop: not an edit
        m.setScene(m.scene());       // a load: not an edit either
        CHECK(m.editCount() == before);
        m.setLocked(true);
        CHECK_FALSE(m.addBody(BodyKind::Planet).ok);
        CHECK(m.editCount() == before);
        m.setLocked(false);
        const std::uint64_t planet = m.addBody(BodyKind::Planet).id;
        CHECK(m.remove(planet).ok);
        CHECK(m.editCount() == before + 2);
    }

    TEST_CASE("a duplicate in orbit mode is half an orbit further on, with a new id and a copy name")
    {
        EditorModel m;
        const std::uint64_t star = m.addBody(BodyKind::Star).id;
        const std::uint64_t planet = m.addBody(BodyKind::Planet).id;
        REQUIRE(m.setName(planet, "Domus").ok);
        REQUIRE(m.setOrbitAround(planet, star, 1.5e11).ok);
        const std::uint64_t edits = m.editCount();
        const EditResult r = m.duplicate(planet);
        REQUIRE(r.ok);
        CHECK(r.id != planet);
        CHECK(r.id != star);
        CHECK(m.selected() == r.id);
        CHECK(m.editCount() == edits + 1);
        const SceneBody* c = m.body(r.id);
        REQUIRE(c);
        CHECK(c->name == "Domus copy");
        REQUIRE(c->orbit);
        CHECK(c->orbit->parent == star);
        CHECK(c->orbit->a == 1.5e11);
        CHECK(c->orbit->meanAnomaly_deg == doctest::Approx(180.0));
        CHECK(c->mu == m.body(planet)->mu);
        // on the far side of the star
        Vec3 rs, vs, rp, vp, rc, vc;
        REQUIRE(m.resolvedState(star, rs, vs));
        REQUIRE(m.resolvedState(planet, rp, vp));
        REQUIRE(m.resolvedState(r.id, rc, vc));
        CHECK(dot(rp - rs, rc - rs) < 0.0);
        // a second copy gets the next free name
        CHECK(m.body(m.duplicate(planet).id)->name == "Domus copy 2");
        CHECK(m.copyName("Domus") == "Domus copy 3");
    }

    TEST_CASE("a duplicate in state mode starts beside the original; shuttles too")
    {
        EditorModel m;
        const std::uint64_t star = m.addBody(BodyKind::Star, Vec3{1.0e9, 0.0, 0.0}, Vec3{0.0, 5.0, 0.0}).id;
        const EditResult r = m.duplicate(star);
        REQUIRE(r.ok);
        const SceneBody* c = m.body(r.id);
        REQUIRE(c);
        CHECK_FALSE(c->orbit);
        CHECK(c->position.x == doctest::Approx(1.0e9 + 4.0 * m.body(star)->radius));
        CHECK(c->velocity.y == 5.0);

        const std::uint64_t sh = m.addShuttle(ControllerKind::Prograde, Vec3{2.0e9, 0.0, 0.0}).id;
        const EditResult rs = m.duplicate(sh);
        REQUIRE(rs.ok);
        const SceneShuttle* cs = m.shuttle(rs.id);
        REQUIRE(cs);
        CHECK(cs->position.x == doctest::Approx(2.0e9 + 1000.0));
        CHECK(cs->controller == m.shuttle(sh)->controller);
        CHECK(cs->name == m.shuttle(sh)->name + " copy");

        CHECK_FALSE(m.duplicate(987654).ok);
        m.setLocked(true);
        CHECK_FALSE(m.duplicate(star).ok);
    }

    TEST_CASE("copy names stay within 64 characters")
    {
        EditorModel m;
        const std::uint64_t b = m.addBody(BodyKind::Moon).id;
        REQUIRE(m.setName(b, std::string(64, 'x')).ok);
        const EditResult r = m.duplicate(b);
        REQUIRE(r.ok);
        CHECK(m.body(r.id)->name.size() <= 64);
        CHECK(m.body(r.id)->name.substr(m.body(r.id)->name.size() - 5) == " copy");
    }
}
