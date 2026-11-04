#include <doctest.h>
#include "test_app_editor_util.h"

using namespace cosmos;
using namespace cosmos::app;
using namespace editor_test;

// Editor model (plan 10.1, 12.1, 12.2): adding, naming, editing, deleting, locking.
TEST_SUITE("app editor")
{
    TEST_CASE("a new body gets the table 12.1 defaults, a generic name and the selection")
    {
        EditorModel m;
        const EditResult r = m.addBody(BodyKind::Planet, {1.0, 2.0, 0.0}, {3.0, 4.0, 0.0});
        REQUIRE(r.ok);
        REQUIRE(m.scene().bodies.size() == 1);
        const SceneBody& b = m.scene().bodies[0];
        const BodyDesc d = defaultBodyDesc(BodyKind::Planet);
        CHECK(b.id == r.id);
        CHECK(b.name == "Planet 1");
        CHECK(b.kind == BodyKind::Planet);
        CHECK(b.mu == d.mu);
        CHECK(b.radius == d.radius);
        CHECK(b.colour == d.colour);
        CHECK(b.spinPeriod_h == 0.0);
        CHECK_FALSE(b.atmosphere.has_value());
        CHECK_FALSE(b.orbit.has_value());
        CHECK(b.position.x == 1.0);
        CHECK(b.velocity.y == 4.0);
        CHECK_FALSE(b.kindLocked);
        CHECK(m.selected() == r.id);
        CHECK(m.selectedType() == ObjectType::Body);
        std::string err;
        CHECK(m.validate(err));
    }

    TEST_CASE("every kind of table 12.1 can be added")
    {
        EditorModel m;
        const BodyKind kinds[] = {BodyKind::Star, BodyKind::Planet, BodyKind::GasGiant, BodyKind::DwarfPlanet,
                                  BodyKind::Moon, BodyKind::Asteroid, BodyKind::NeutronStar, BodyKind::BlackHole};
        for (BodyKind k : kinds) {
            const EditResult r = m.addBody(k);
            REQUIRE(r.ok);
            CHECK(m.body(r.id)->name == std::string(kindName(k)) + " 1");
            CHECK(m.body(r.id)->mu == defaultBodyDesc(k).mu);
        }
        std::string err;
        CHECK(m.validate(err));
    }

    TEST_CASE("the name counter is per kind and is never reused")
    {
        EditorModel m;
        const std::uint64_t p1 = m.addBody(BodyKind::Planet).id;
        const std::uint64_t p2 = m.addBody(BodyKind::Planet).id;
        const std::uint64_t s1 = m.addBody(BodyKind::Star).id;
        CHECK(m.body(p2)->name == "Planet 2");
        CHECK(m.body(s1)->name == "Star 1");   // another kind, its own counter

        REQUIRE(m.remove(p2).ok);
        CHECK(m.addBody(BodyKind::Planet).id != 0);
        CHECK(m.scene().bodies.back().name == "Planet 3");   // Planet 2 is not given out again

        REQUIRE(m.setName(p1, "Home").ok);
        CHECK(m.addBody(BodyKind::Planet).id != 0);
        CHECK(m.scene().bodies.back().name == "Planet 4");   // renaming does not free Planet 1 either

        // A kind change leaves the name and the counters alone
        REQUIRE(m.setKind(s1, BodyKind::Moon).ok);
        CHECK(m.body(s1)->name == "Star 1");
        CHECK(m.addBody(BodyKind::Star).id != 0);
        CHECK(m.scene().bodies.back().name == "Star 2");
        CHECK(m.namesIssued(BodyKind::Planet) == 4);
        CHECK(m.namesIssued(BodyKind::Moon) == 0);
    }

    TEST_CASE("shuttles are named Shuttle <n>, with the defaults of table 7.1")
    {
        EditorModel m;
        const EditResult a = m.addShuttle(ControllerKind::Prograde, {5.0, 0.0, 0.0}, {0.0, 7.0, 0.0});
        const EditResult b = m.addShuttle();
        REQUIRE(a.ok);
        REQUIRE(b.ok);
        const SceneShuttle* s = m.shuttle(a.id);
        REQUIRE(s != nullptr);
        CHECK(s->name == "Shuttle 1");
        CHECK(m.shuttle(b.id)->name == "Shuttle 2");
        CHECK(s->controller == "Prograde");
        CHECK(m.shuttle(b.id)->controller == "Coast");
        CHECK(s->dryMass == 2000.0);
        CHECK(s->fuelMass == 3000.0);
        CHECK(s->maxThrust == 60000.0);
        CHECK(s->exhaustVelocity == 3000.0);
        CHECK(s->cdA == 2.0);
        CHECK(s->radius == 2.0);
        CHECK(s->maxTorque == 5000.0);
        CHECK(s->inertia == 1.0e4);
        CHECK(s->position.x == 5.0);
        CHECK(s->velocity.y == 7.0);
        CHECK(m.selected() == b.id);
        CHECK(m.selectedType() == ObjectType::Shuttle);
        REQUIRE(m.remove(b.id).ok);
        CHECK(m.addShuttle().id != 0);
        CHECK(m.scene().shuttles.back().name == "Shuttle 3");
    }

    TEST_CASE("template names consume no counter")
    {
        EditorModel m;
        m.setScene(lucerna());   // Favilla, Velum, Domus ... are template names, not "<Kind> <n>"
        CHECK(m.namesIssued(BodyKind::Planet) == 0);
        CHECK(m.namesIssued(BodyKind::Moon) == 0);
        CHECK(m.namesIssued(BodyKind::Star) == 0);
        const EditResult r = m.addBody(BodyKind::Planet);
        CHECK(m.body(r.id)->name == "Planet 1");
        CHECK(m.addBody(BodyKind::Moon).id != 0);
        CHECK(m.scene().bodies.back().name == "Moon 1");
    }

    TEST_CASE("loading a scene continues the numbering past the names it contains")
    {
        Scene s;
        SceneBody a;
        a.id = 5;
        a.name = "Planet 7";
        a.mu = 1.0e12;
        a.radius = 1.0e5;
        SceneBody b = a;
        b.id = 6;
        b.name = "Gas Giant 2";
        SceneBody c = a;
        c.id = 7;
        c.name = "Planet 12x";   // not a generic name
        s.bodies = {a, b, c};
        SceneShuttle sh;
        sh.id = 9;
        sh.name = "Shuttle 4";
        s.shuttles.push_back(sh);

        EditorModel m;
        m.setScene(s);
        CHECK(m.addBody(BodyKind::Planet).id != 0);
        CHECK(m.scene().bodies.back().name == "Planet 8");
        CHECK(m.addBody(BodyKind::GasGiant).id != 0);
        CHECK(m.scene().bodies.back().name == "Gas Giant 3");
        CHECK(m.addShuttle().id != 0);
        CHECK(m.scene().shuttles.back().name == "Shuttle 5");
        CHECK(m.nextId() == 13);   // 5, 6, 7, 9 in the scene, then 10, 11, 12 given out

        // restoreScene never lowers a counter (Stop restoring the snapshot): the Planet 8 above is not handed out again
        m.restoreScene(s);
        CHECK(m.addBody(BodyKind::Planet).id != 0);
        CHECK(m.scene().bodies.back().name == "Planet 9");
        // while setScene (a new document) starts over from the scene
        m.setScene(s);
        CHECK(m.addBody(BodyKind::Planet).id != 0);
        CHECK(m.scene().bodies.back().name == "Planet 8");
    }

    TEST_CASE("IDs are unique, shared by bodies and shuttles, and not reused after a delete")
    {
        EditorModel m;
        m.setScene(lucerna());
        const std::uint64_t top = m.nextId() - 1;
        CHECK(top == 14);
        const std::uint64_t a = m.addBody(BodyKind::Moon).id;
        const std::uint64_t b = m.addShuttle().id;
        CHECK(a == 15);
        CHECK(b == 16);
        REQUIRE(m.remove(b).ok);
        REQUIRE(m.remove(a).ok);
        CHECK(m.addBody(BodyKind::Moon).id == 17);
        std::string err;
        CHECK(m.validate(err));
    }

    TEST_CASE("typed edits change the scene and bump the revision")
    {
        EditorModel m;
        const std::uint64_t id = m.addBody(BodyKind::Planet).id;
        std::uint64_t rev = m.revision();

        REQUIRE(m.setName(id, "  Terra  ").ok);
        CHECK(m.body(id)->name == "Terra");   // trimmed
        CHECK(m.revision() > rev);
        rev = m.revision();

        REQUIRE(m.setKind(id, BodyKind::Moon).ok);
        CHECK(m.body(id)->kind == BodyKind::Moon);
        CHECK(m.body(id)->mu == defaultBodyDesc(BodyKind::Planet).mu);   // mass and radius stay

        REQUIRE(m.setKind(id, BodyKind::GasGiant, true).ok);
        CHECK(m.body(id)->mu == defaultBodyDesc(BodyKind::GasGiant).mu);
        CHECK(m.body(id)->radius == defaultBodyDesc(BodyKind::GasGiant).radius);
        CHECK(m.body(id)->colour == defaultBodyDesc(BodyKind::GasGiant).colour);

        REQUIRE(m.setMu(id, 4.0e14).ok);
        CHECK(m.body(id)->mu == 4.0e14);
        REQUIRE(m.setMassKg(id, 1.0e22).ok);
        CHECK(m.body(id)->mu == doctest::Approx(1.0e22 * 6.67430e-11).epsilon(1e-15));
        REQUIRE(m.setRadius(id, 1.5e6).ok);
        CHECK(m.body(id)->radius == 1.5e6);
        REQUIRE(m.setColour(id, {0.1f, 0.2f, 0.3f}).ok);
        CHECK(m.body(id)->colour[1] == 0.2f);
        REQUIRE(m.setSpinPeriodHours(id, -5.8).ok);
        CHECK(m.body(id)->spinPeriod_h == -5.8);
        REQUIRE(m.setState(id, {1.0e9, -2.0e9, 0.0}, {10.0, 20.0, 0.0}).ok);
        CHECK(m.body(id)->position.y == -2.0e9);
        CHECK(m.body(id)->velocity.x == 10.0);
        CHECK(m.revision() > rev);
    }

    TEST_CASE("atmosphere toggle and values")
    {
        EditorModel m;
        const std::uint64_t id = m.addBody(BodyKind::Planet).id;
        REQUIRE(m.enableAtmosphere(id, true).ok);
        REQUIRE(m.body(id)->atmosphere.has_value());
        CHECK(m.body(id)->atmosphere->rho0 == 1.225);
        CHECK(m.body(id)->atmosphere->scaleHeight == 8500.0);
        REQUIRE(m.setFieldText(id, Field::AtmoRho0, "4.7 kg/m^3").ok);
        REQUIRE(m.setFieldText(id, Field::AtmoScaleHeight, "9 km").ok);
        CHECK(m.body(id)->atmosphere->rho0 == 4.7);
        CHECK(m.body(id)->atmosphere->scaleHeight == 9000.0);
        CHECK_FALSE(m.setFieldText(id, Field::AtmoScaleHeight, "0 km").ok);
        CHECK_FALSE(m.setFieldText(id, Field::AtmoRho0, "-1").ok);
        CHECK(m.body(id)->atmosphere->scaleHeight == 9000.0);
        REQUIRE(m.enableAtmosphere(id, false).ok);
        CHECK_FALSE(m.body(id)->atmosphere.has_value());
        CHECK(m.fieldText(id, Field::AtmoRho0).empty());
        const EditResult r = m.setFieldText(id, Field::AtmoRho0, "1");
        CHECK_FALSE(r.ok);
        CHECK(r.error.find("atmosphere") != std::string::npos);
    }

    TEST_CASE("a rejected edit leaves the scene exactly as it was")
    {
        EditorModel m;
        const std::uint64_t id = m.addBody(BodyKind::Planet).id;
        const std::uint64_t other = m.addBody(BodyKind::Moon).id;
        const Scene before = m.scene();
        const std::uint64_t rev = m.revision();

        CHECK_FALSE(m.setMu(id, 0.0).ok);
        CHECK_FALSE(m.setMu(id, -1.0).ok);
        CHECK_FALSE(m.setMu(id, std::nan("")).ok);
        CHECK_FALSE(m.setRadius(id, 0.0).ok);
        CHECK_FALSE(m.setRadius(id, -5.0).ok);
        CHECK_FALSE(m.setName(id, "   ").ok);
        CHECK_FALSE(m.setName(id, std::string(65, 'x')).ok);
        CHECK_FALSE(m.setColour(id, {1.5f, 0.0f, 0.0f}).ok);
        CHECK_FALSE(m.setState(id, {std::nan(""), 0.0, 0.0}, {}).ok);
        CHECK_FALSE(m.setAtmosphere(id, Atmosphere{-1.0, 8000.0}).ok);
        CHECK_FALSE(m.setSpinPeriodHours(id, std::nan("")).ok);
        CHECK_FALSE(m.setMu(9999, 1.0).ok);   // no such object
        CHECK_FALSE(m.setController(id, ControllerKind::Prograde).ok);   // a body has no controller
        CHECK_FALSE(m.updateBody(id, [](SceneBody& b) { b.mu = -3.0; }).ok);
        CHECK_FALSE(m.updateBody(id, [&](SceneBody& b) { b.orbit = SceneOrbit{other, 1.0e9, 1.5, 0.0, 0.0, false}; }).ok);
        CHECK(sameScene(m.scene(), before));
        CHECK(m.revision() == rev);

        const EditResult r = m.setMu(id, 0.0);
        CHECK_FALSE(r.error.empty());
    }

    TEST_CASE("updateBody commits a valid change and cannot change the id")
    {
        EditorModel m;
        const std::uint64_t id = m.addBody(BodyKind::Asteroid).id;
        REQUIRE(m.updateBody(id, [](SceneBody& b) {
                     b.id = 777;
                     b.fictionalDensity = true;
                     b.inertiaFactor = 0.3;
                 }).ok);
        REQUIRE(m.body(id) != nullptr);
        CHECK(m.body(id)->fictionalDensity);
        CHECK(m.body(id)->inertiaFactor == 0.3);
        CHECK(m.body(777) == nullptr);
    }

    TEST_CASE("shuttle fields and controller")
    {
        EditorModel m;
        const std::uint64_t id = m.addShuttle().id;
        REQUIRE(m.setController(id, ControllerKind::Wander).ok);
        CHECK(m.shuttle(id)->controller == "Wander");
        REQUIRE(m.setFieldText(id, Field::DryMass, "2.5 t").ok);
        REQUIRE(m.setFieldText(id, Field::FuelMass, "0 kg").ok);
        REQUIRE(m.setFieldText(id, Field::MaxThrust, "80 kN").ok);
        REQUIRE(m.setFieldText(id, Field::ExhaustVelocity, "3.2 km/s").ok);
        REQUIRE(m.setFieldText(id, Field::CdA, "3 m^2").ok);
        REQUIRE(m.setFieldText(id, Field::CollisionRadius, "3").ok);
        REQUIRE(m.setFieldText(id, Field::MaxTorque, "6 kN m").ok);
        REQUIRE(m.setFieldText(id, Field::Inertia, "2e4").ok);
        const SceneShuttle* s = m.shuttle(id);
        CHECK(s->dryMass == 2500.0);
        CHECK(s->fuelMass == 0.0);
        CHECK(s->maxThrust == 80000.0);
        CHECK(s->exhaustVelocity == 3200.0);
        CHECK(s->cdA == 3.0);
        CHECK(s->radius == 3.0);
        CHECK(s->maxTorque == 6000.0);
        CHECK(s->inertia == 2.0e4);
        CHECK_FALSE(m.setFieldText(id, Field::DryMass, "0 kg").ok);       // Shuttles::add would reject it
        CHECK_FALSE(m.setFieldText(id, Field::Inertia, "0").ok);
        CHECK_FALSE(m.setFieldText(id, Field::FuelMass, "-1 kg").ok);
        CHECK_FALSE(m.setFieldText(id, Field::Mass, "1 kg").ok);          // a body field
        CHECK(s->dryMass == 2500.0);
        EditorModel bodies;
        const std::uint64_t b = bodies.addBody(BodyKind::Planet).id;
        CHECK_FALSE(bodies.setFieldText(b, Field::DryMass, "1 kg").ok);
    }

    TEST_CASE("text fields: parse with units, apply, show back, never round an untouched value")
    {
        EditorModel m;
        const std::uint64_t id = m.addBody(BodyKind::Planet).id;
        CHECK(m.fieldText(id, Field::Name) == "Planet 1");
        CHECK(m.fieldText(id, Field::Mass) == "1 M_earth");
        CHECK(m.fieldText(id, Field::Radius) == "6371 km");
        CHECK(m.fieldText(id, Field::SpinPeriod) == "0 h");
        CHECK(m.fieldText(id, Field::OrbitA).empty());   // state mode
        CHECK(m.fieldText(id, Field::DryMass).empty());  // not a shuttle field

        REQUIRE(m.setFieldText(id, Field::Mass, "1 M_sun").ok);
        CHECK(m.body(id)->mu == kSolarMu);
        REQUIRE(m.setFieldText(id, Field::Radius, "6.96e8").ok);   // bare number: metres
        CHECK(m.body(id)->radius == 6.96e8);
        REQUIRE(m.setFieldText(id, Field::Radius, "1.4 Mm").ok);
        CHECK(m.body(id)->radius == 1.4e6);
        REQUIRE(m.setFieldText(id, Field::SpinPeriod, "-1.5 d").ok);
        CHECK(m.body(id)->spinPeriod_h == -36.0);
        CHECK(m.fieldText(id, Field::SpinPeriod) == "-36 h");
        REQUIRE(m.setFieldText(id, Field::PosX, "0.09 AU").ok);
        CHECK(m.body(id)->position.x == doctest::Approx(0.09 * 149597870700.0).epsilon(1e-15));
        REQUIRE(m.setFieldText(id, Field::VelY, "29.78 km/s").ok);
        CHECK(m.body(id)->velocity.y == doctest::Approx(29780.0).epsilon(1e-15));
        REQUIRE(m.setFieldText(id, Field::Name, "Terra Nova").ok);
        CHECK(m.body(id)->name == "Terra Nova");

        // Pressing Enter in a field that still shows the stored value changes nothing, even though the text is rounded
        EditorModel n;
        const std::uint64_t pid = n.addBody(BodyKind::Planet).id;
        REQUIRE(n.setRadius(pid, 6371234.5).ok);
        const std::string shown = n.fieldText(pid, Field::Radius);
        CHECK(shown == "6371 km");
        const std::uint64_t rev = n.revision();
        CHECK(n.setFieldText(pid, Field::Radius, shown).ok);
        CHECK(n.body(pid)->radius == 6371234.5);
        CHECK(n.revision() == rev);
    }

    TEST_CASE("text field errors name the field and say what is wrong")
    {
        EditorModel m;
        const std::uint64_t id = m.addBody(BodyKind::Planet).id;
        const double radius = m.body(id)->radius;

        EditResult r = m.setFieldText(id, Field::Radius, "abc");
        CHECK_FALSE(r.ok);
        CHECK(r.error.find("Radius") != std::string::npos);
        CHECK(r.error.find("not a number") != std::string::npos);

        r = m.setFieldText(id, Field::Radius, "5 kg");
        CHECK_FALSE(r.ok);
        CHECK(r.error.find("unit of mass") != std::string::npos);

        r = m.setFieldText(id, Field::Radius, "-5 km");
        CHECK_FALSE(r.ok);
        CHECK(r.error.find("greater than zero") != std::string::npos);

        r = m.setFieldText(id, Field::Mass, "0 kg");
        CHECK_FALSE(r.ok);
        CHECK(r.error.find("Mass") != std::string::npos);

        r = m.setFieldText(id, Field::Mass, "");
        CHECK_FALSE(r.ok);

        r = m.setFieldText(id, Field::OrbitA, "1 AU");   // not in orbit mode
        CHECK_FALSE(r.ok);
        CHECK(r.error.find("orbit") != std::string::npos);

        r = m.setFieldText(4242, Field::Name, "x");
        CHECK_FALSE(r.ok);
        CHECK(m.body(id)->radius == radius);
    }

    TEST_CASE("deleting a body or a shuttle")
    {
        EditorModel m;
        const std::uint64_t a = m.addBody(BodyKind::Planet).id;
        const std::uint64_t b = m.addBody(BodyKind::Moon).id;
        const std::uint64_t s = m.addShuttle().id;
        REQUIRE(m.select(b));
        REQUIRE(m.remove(b).ok);
        CHECK(m.body(b) == nullptr);
        CHECK(m.selected() == 0);   // the selection followed the deletion
        CHECK(m.scene().bodies.size() == 1);
        REQUIRE(m.select(s));
        REQUIRE(m.remove(s).ok);
        CHECK(m.scene().shuttles.empty());
        CHECK_FALSE(m.remove(s).ok);   // already gone
        CHECK_FALSE(m.remove(0).ok);
        CHECK(m.typeOf(a) == ObjectType::Body);
        CHECK_FALSE(m.select(b));
        CHECK(m.selected() == 0);
    }

    TEST_CASE("the object list: bodies then shuttles")
    {
        EditorModel m;
        const std::uint64_t a = m.addBody(BodyKind::Star).id;
        const std::uint64_t s = m.addShuttle(ControllerKind::Retrograde).id;
        const std::uint64_t b = m.addBody(BodyKind::Moon).id;
        const std::vector<ObjectRow> rows = m.objects();
        REQUIRE(rows.size() == 3);
        CHECK(rows[0].id == a);
        CHECK(rows[0].type == ObjectType::Body);
        CHECK(rows[0].kind == BodyKind::Star);
        CHECK(rows[1].id == b);
        CHECK(rows[2].id == s);
        CHECK(rows[2].type == ObjectType::Shuttle);
        CHECK(rows[2].controller == "Retrograde");
        CHECK(rows[2].name == "Shuttle 1");
    }

    TEST_CASE("editing is blocked while locked, and nothing changes")
    {
        EditorModel m;
        const std::uint64_t id = m.addBody(BodyKind::Planet).id;
        const std::uint64_t sh = m.addShuttle().id;
        const Scene before = m.scene();
        const std::uint64_t rev = m.revision();
        m.setLocked(true);
        CHECK(m.locked());

        const auto blocked = [&](const EditResult& r) {
            CHECK_FALSE(r.ok);
            CHECK(r.error == kLockedMessage);
        };
        blocked(m.addBody(BodyKind::Star));
        blocked(m.addShuttle());
        blocked(m.remove(id));
        blocked(m.setName(id, "x"));
        blocked(m.setKind(id, BodyKind::Moon));
        blocked(m.setMu(id, 1.0e12));
        blocked(m.setRadius(id, 1.0e6));
        blocked(m.setColour(id, {0, 0, 0}));
        blocked(m.setSpinPeriodHours(id, 3.0));
        blocked(m.enableAtmosphere(id, true));
        blocked(m.enableAtmosphere(id, false));
        blocked(m.setState(id, {}, {}));
        blocked(m.setOrbitAround(sh, id, 1.0e7));
        blocked(m.bakeState(id));
        blocked(m.setController(sh, ControllerKind::Wander));
        blocked(m.setFieldText(id, Field::Radius, "1 km"));
        blocked(m.setFieldText(id, Field::Name, "x"));
        blocked(m.updateBody(id, [](SceneBody& b) { b.mu = 1.0; }));
        blocked(m.updateShuttle(sh, [](SceneShuttle& s) { s.dryMass = 1.0; }));
        CHECK(sameScene(m.scene(), before));
        CHECK(m.revision() == rev);

        // Reading and selecting still work
        CHECK(m.select(sh));
        CHECK(m.fieldText(id, Field::Radius) == "6371 km");

        m.setLocked(false);
        CHECK(m.setRadius(id, 1.0e6).ok);
    }

    TEST_CASE("reset gives an empty scene with fresh counters")
    {
        EditorModel m;
        m.addBody(BodyKind::Planet);
        m.addShuttle();
        m.reset();
        CHECK(m.scene().bodies.empty());
        CHECK(m.scene().shuttles.empty());
        CHECK(m.selected() == 0);
        CHECK(m.addBody(BodyKind::Planet).id == 1);
        CHECK(m.scene().bodies.back().name == "Planet 1");
    }

    TEST_CASE("fieldInfo and fieldApplies")
    {
        CHECK(fieldApplies(ObjectType::Body, Field::Mass));
        CHECK_FALSE(fieldApplies(ObjectType::Shuttle, Field::Mass));
        CHECK(fieldApplies(ObjectType::Shuttle, Field::DryMass));
        CHECK_FALSE(fieldApplies(ObjectType::Body, Field::DryMass));
        CHECK(fieldApplies(ObjectType::Shuttle, Field::OrbitA));
        CHECK(fieldApplies(ObjectType::Body, Field::Name));
        CHECK_FALSE(fieldApplies(ObjectType::None, Field::Name));
        CHECK(fieldInfo(Field::Radius).quantity == Quantity::Length);
        CHECK(fieldInfo(Field::SpinPeriod).quantity == Quantity::Time);
        CHECK(std::string(fieldInfo(Field::SpinPeriod).defaultUnit) == "h");
        CHECK(std::string(fieldInfo(Field::OrbitOmega).defaultUnit) == "deg");
    }
}
