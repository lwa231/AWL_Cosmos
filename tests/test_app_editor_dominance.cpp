#include <doctest.h>
#include "test_app_editor_util.h"

using namespace cosmos;
using namespace cosmos::app;
using namespace editor_test;

// Plan 10.1: a warning appears when a new body would dominate the scene, i.e. its mu is more than 10% of the mu of the most
// massive other body.
TEST_SUITE("app editor dominance")
{
    TEST_CASE("the first body of a scene and small companions do not warn")
    {
        EditorModel m;
        const EditResult first = m.addBody(BodyKind::Star);
        CHECK(first.ok);
        CHECK_FALSE(anyContains(first.warnings, "mass"));
        CHECK(m.dominanceWarning(first.id).empty());

        const EditResult earth = m.addBody(BodyKind::Planet);
        CHECK_FALSE(anyContains(earth.warnings, "most massive"));
        const EditResult moon = m.addBody(BodyKind::Moon);
        CHECK_FALSE(anyContains(moon.warnings, "most massive"));
        const EditResult rock = m.addBody(BodyKind::Asteroid);
        CHECK_FALSE(anyContains(rock.warnings, "most massive"));
    }

    TEST_CASE("a new body above 10% of the heaviest one warns, and the warning names both")
    {
        EditorModel m;
        m.setScene(lucerna());   // the star has mu = 1.25e18

        EditResult r = m.addBody(BodyKind::Planet);   // 4e14: far below
        CHECK_FALSE(anyContains(r.warnings, "most massive"));

        r = m.addBody(BodyKind::GasGiant);   // Jupiter, 1.267e17 = 10.1% of the star
        REQUIRE(r.ok);
        REQUIRE(anyContains(r.warnings, "most massive"));
        CHECK(anyContains(r.warnings, "Gas Giant 1"));
        CHECK(anyContains(r.warnings, "Lucerna"));
        CHECK(anyContains(r.warnings, "10.1%"));
        CHECK(m.dominanceWarning(r.id) == r.warnings[0]);

        r = m.addBody(BodyKind::Star);   // a whole Sun: 106 times Lucerna
        REQUIRE(r.ok);
        CHECK(anyContains(r.warnings, "dominate"));
        CHECK(anyContains(r.warnings, "times as massive"));

        r = m.addBody(BodyKind::BlackHole);
        CHECK(anyContains(r.warnings, "dominate"));
        r = m.addBody(BodyKind::NeutronStar);
        CHECK(anyContains(r.warnings, "most massive"));
        // The warning is advice: the bodies are in the scene
        CHECK(m.scene().bodies.size() == 14 + 5);
    }

    TEST_CASE("the threshold is 10% of the most massive OTHER body")
    {
        EditorModel m;
        const std::uint64_t big = m.addBody(BodyKind::Star).id;
        const std::uint64_t mid = m.addBody(BodyKind::Planet).id;
        const double muBig = 1.0e20;
        REQUIRE(m.setMu(big, muBig).ok);

        EditResult r = m.setMu(mid, 0.099 * muBig);
        REQUIRE(r.ok);
        CHECK_FALSE(anyContains(r.warnings, "most massive"));
        CHECK(m.dominanceWarning(mid).empty());

        r = m.setMu(mid, 0.101 * muBig);
        REQUIRE(r.ok);
        CHECK(anyContains(r.warnings, "most massive"));
        CHECK(anyContains(r.warnings, "10.1%"));
        CHECK_FALSE(m.dominanceWarning(mid).empty());

        // Already above the line: changing the mass again is not news
        r = m.setMu(mid, 0.2 * muBig);
        CHECK_FALSE(anyContains(r.warnings, "most massive"));
        r = m.setMu(big, 2.0 * muBig);   // the heaviest body itself, 10 times the other: no warning for its own edits either
        CHECK_FALSE(anyContains(r.warnings, "most massive"));

        // Back below the line: no warning, and crossing it again warns again
        r = m.setMu(mid, 0.05 * muBig);
        CHECK_FALSE(anyContains(r.warnings, "most massive"));
        CHECK(m.dominanceWarning(mid).empty());
        r = m.setMu(mid, 0.5 * muBig);
        CHECK(anyContains(r.warnings, "most massive"));
    }

    TEST_CASE("editing the mass in a text field warns the same way; other edits do not repeat it")
    {
        EditorModel m;
        const std::uint64_t star = m.addBody(BodyKind::Star).id;
        const std::uint64_t planet = m.addBody(BodyKind::Planet).id;
        (void)star;

        EditResult r = m.setFieldText(planet, Field::Mass, "0.2 M_sun");
        REQUIRE(r.ok);
        CHECK(anyContains(r.warnings, "most massive"));
        CHECK(anyContains(r.warnings, "20%"));

        r = m.setFieldText(planet, Field::Radius, "10 Mm");   // not a mass edit
        REQUIRE(r.ok);
        CHECK_FALSE(anyContains(r.warnings, "most massive"));

        r = m.setFieldText(planet, Field::Mass, "1 M_earth");
        REQUIRE(r.ok);
        CHECK_FALSE(anyContains(r.warnings, "most massive"));
    }

    TEST_CASE("giving a body a heavier kind with its defaults warns; a kind change alone does not")
    {
        EditorModel m;
        m.addBody(BodyKind::Star);
        const std::uint64_t p = m.addBody(BodyKind::Planet).id;
        EditResult r = m.setKind(p, BodyKind::Moon);
        CHECK_FALSE(anyContains(r.warnings, "most massive"));
        r = m.setKind(p, BodyKind::BlackHole, true);   // 10 solar masses
        REQUIRE(r.ok);
        CHECK(anyContains(r.warnings, "dominate"));
    }

    TEST_CASE("shuttles have no mass in the scene and never warn")
    {
        EditorModel m;
        m.addBody(BodyKind::Star);
        const EditResult r = m.addShuttle();
        CHECK_FALSE(anyContains(r.warnings, "most massive"));
        CHECK(m.dominanceWarning(r.id).empty());
    }
}
