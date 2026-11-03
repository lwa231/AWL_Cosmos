#include <doctest.h>
#include <string>
#include "core/bodies.h"
#include "core/constants.h"
#include "core/kinds.h"

using namespace cosmos;

TEST_SUITE("kinds")
{
    TEST_CASE("kind names")
    {
        CHECK(std::string(kindName(BodyKind::Star)) == "Star");
        CHECK(std::string(kindName(BodyKind::Planet)) == "Planet");
        CHECK(std::string(kindName(BodyKind::GasGiant)) == "Gas Giant");
        CHECK(std::string(kindName(BodyKind::DwarfPlanet)) == "Dwarf Planet");
        CHECK(std::string(kindName(BodyKind::Moon)) == "Moon");
        CHECK(std::string(kindName(BodyKind::Asteroid)) == "Asteroid");
        CHECK(std::string(kindName(BodyKind::NeutronStar)) == "Neutron Star");
        CHECK(std::string(kindName(BodyKind::BlackHole)) == "Black Hole");
    }

    TEST_CASE("defaults follow table 12.1")
    {
        const double G = constants::G;
        BodyDesc d = defaultBodyDesc(BodyKind::Star);
        CHECK(d.kind == BodyKind::Star);
        CHECK(d.mu == 1.32712440018e20);
        CHECK(d.radius == 6.957e8);

        d = defaultBodyDesc(BodyKind::Planet);
        CHECK(d.mu == 5.972e24 * G);
        CHECK(d.radius == 6.371e6);

        d = defaultBodyDesc(BodyKind::GasGiant);
        CHECK(d.mu == 1.898e27 * G);
        CHECK(d.radius == 7.1492e7);

        d = defaultBodyDesc(BodyKind::Moon);
        CHECK(d.mu == 7.35e22 * G);
        CHECK(d.radius == 1.737e6);

        d = defaultBodyDesc(BodyKind::Asteroid);
        CHECK(d.mu == 1.0e15 * G);
        CHECK(d.radius == 5.0e3);

        d = defaultBodyDesc(BodyKind::NeutronStar);
        CHECK(d.mu == 1.4 * 1.32712440018e20);
        CHECK(d.radius == 1.2e4);

        d = defaultBodyDesc(BodyKind::BlackHole);
        CHECK(d.mu == 10.0 * 1.32712440018e20);
        CHECK(d.radius == 2.0 * d.mu / (constants::c * constants::c));
        CHECK(d.radius == doctest::Approx(29.5e3).epsilon(5e-3)); // "about 29.5 km" in the table
    }

    TEST_CASE("defaults are plain bodies: k = 0.4, no spin, no atmosphere, unlocked, unnamed")
    {
        for (BodyKind k : {BodyKind::Star, BodyKind::Planet, BodyKind::GasGiant, BodyKind::DwarfPlanet, BodyKind::Moon,
                           BodyKind::Asteroid, BodyKind::NeutronStar, BodyKind::BlackHole}) {
            const BodyDesc d = defaultBodyDesc(k);
            CHECK(d.kind == k);
            CHECK(d.inertiaFactor == 0.4);
            CHECK(d.spinRate == 0.0);
            CHECK(!d.atmosphere.has_value());
            CHECK(!d.kindLocked);
            CHECK(d.name.empty());
            CHECK(d.mu > 0.0);
            CHECK(d.radius > 0.0);
        }
    }

    TEST_CASE("a default body can be added to the store")
    {
        Bodies b;
        const BodyId id = b.add(defaultBodyDesc(BodyKind::Moon));
        CHECK(b.kind[*b.find(id)] == BodyKind::Moon);
        CHECK(b.consistent());
    }

    TEST_CASE("naming: <Kind> <n>, one counter per kind starting at 1")
    {
        NameCounter c;
        CHECK(c.next(BodyKind::Planet) == "Planet 1");
        CHECK(c.next(BodyKind::Planet) == "Planet 2");
        CHECK(c.next(BodyKind::Moon) == "Moon 1");
        CHECK(c.next(BodyKind::GasGiant) == "Gas Giant 1");
        CHECK(c.next(BodyKind::Planet) == "Planet 3");
        CHECK(c.next(BodyKind::BlackHole) == "Black Hole 1");
        CHECK(c.next(BodyKind::NeutronStar) == "Neutron Star 1");
        CHECK(c.issued(BodyKind::Planet) == 3);
        CHECK(c.issued(BodyKind::Moon) == 1);
        CHECK(c.issued(BodyKind::Asteroid) == 0);
    }

    TEST_CASE("naming: numbers are never reused after delete or rename")
    {
        NameCounter c;
        Bodies b;
        const BodyId p1 = b.add(newBodyDesc(BodyKind::Planet, c));
        const BodyId p2 = b.add(newBodyDesc(BodyKind::Planet, c));
        CHECK(b.name[*b.find(p1)] == "Planet 1");
        CHECK(b.name[*b.find(p2)] == "Planet 2");

        b.remove(p2); // delete the newest: its number stays used
        CHECK(newBodyDesc(BodyKind::Planet, c).name == "Planet 3");

        b.name[*b.find(p1)] = "Home"; // rename the oldest: Planet 1 is not handed out again either
        CHECK(newBodyDesc(BodyKind::Planet, c).name == "Planet 4");
    }

    TEST_CASE("template names consume no counter")
    {
        NameCounter c;
        Bodies b;
        BodyDesc d = defaultBodyDesc(BodyKind::Planet);
        d.name = "Domus";
        d.kindLocked = true;
        b.add(d); // explicit name: counter untouched
        d.name = "Gelu";
        d.kind = BodyKind::Moon;
        b.add(d);
        CHECK(c.issued(BodyKind::Planet) == 0);
        CHECK(c.issued(BodyKind::Moon) == 0);
        CHECK(newBodyDesc(BodyKind::Planet, c).name == "Planet 1");
        CHECK(newBodyDesc(BodyKind::Moon, c).name == "Moon 1");
    }

    TEST_CASE("counter state can be saved and restored")
    {
        NameCounter a;
        a.next(BodyKind::Star);
        a.next(BodyKind::Star);
        NameCounter b;
        b.setIssued(BodyKind::Star, a.issued(BodyKind::Star));
        CHECK(b.next(BodyKind::Star) == "Star 3");
    }

    TEST_CASE("reclassification: the heavier body's kind is kept by default")
    {
        CHECK(mergedKind(BodyKind::Planet, false, BodyKind::Moon, 1.0) == BodyKind::Planet);
        CHECK(mergedKind(BodyKind::Star, false, BodyKind::Planet, 1.0) == BodyKind::Star);
        CHECK(mergedKind(BodyKind::GasGiant, false, BodyKind::GasGiant, 1.0) == BodyKind::GasGiant);
        CHECK(mergedKind(BodyKind::Moon, false, BodyKind::Asteroid, 1.0) == BodyKind::Moon);
        CHECK(mergedKind(BodyKind::Asteroid, false, BodyKind::Asteroid, 1.0) == BodyKind::Asteroid);
        CHECK(mergedKind(BodyKind::Star, false, BodyKind::NeutronStar, 10.0 * kSolarMu) == BodyKind::Star);
    }

    TEST_CASE("reclassification: anything merging with a black hole becomes a black hole")
    {
        CHECK(mergedKind(BodyKind::BlackHole, false, BodyKind::Planet, 1.0) == BodyKind::BlackHole);
        CHECK(mergedKind(BodyKind::Star, false, BodyKind::BlackHole, 1.0) == BodyKind::BlackHole);
        CHECK(mergedKind(BodyKind::Planet, false, BodyKind::BlackHole, 1.0) == BodyKind::BlackHole);
        CHECK(mergedKind(BodyKind::BlackHole, false, BodyKind::BlackHole, 1.0) == BodyKind::BlackHole);
    }

    TEST_CASE("reclassification: a neutron star above 2.3 solar masses becomes a black hole")
    {
        CHECK(kNeutronStarMaxMu == 2.3 * 1.32712440018e20);
        CHECK(mergedKind(BodyKind::NeutronStar, false, BodyKind::NeutronStar, 2.8 * kSolarMu) == BodyKind::BlackHole);
        CHECK(mergedKind(BodyKind::NeutronStar, false, BodyKind::Asteroid, 2.30001 * kSolarMu) == BodyKind::BlackHole);
        CHECK(mergedKind(BodyKind::NeutronStar, false, BodyKind::NeutronStar, 2.2 * kSolarMu) == BodyKind::NeutronStar);
        // exactly at the limit is not above it
        CHECK(mergedKind(BodyKind::NeutronStar, false, BodyKind::NeutronStar, kNeutronStarMaxMu) == BodyKind::NeutronStar);
    }

    TEST_CASE("reclassification is upward only: nothing becomes a neutron star or a lighter kind")
    {
        for (BodyKind k : {BodyKind::Star, BodyKind::Planet, BodyKind::GasGiant, BodyKind::Moon, BodyKind::Asteroid}) {
            CHECK(mergedKind(k, false, BodyKind::NeutronStar, 100.0 * kSolarMu) == k);
        }
        // a heavy non-remnant kind never turns into a black hole by mass alone
        CHECK(mergedKind(BodyKind::Star, false, BodyKind::Star, 500.0 * kSolarMu) == BodyKind::Star);
    }

    TEST_CASE("kindLocked survivors are never reclassified")
    {
        CHECK(mergedKind(BodyKind::Star, true, BodyKind::BlackHole, 1.0) == BodyKind::Star);
        CHECK(mergedKind(BodyKind::NeutronStar, true, BodyKind::NeutronStar, 5.0 * kSolarMu) == BodyKind::NeutronStar);
        CHECK(mergedKind(BodyKind::Planet, true, BodyKind::Moon, 1.0) == BodyKind::Planet);
        // a locked body that is absorbed does not protect the survivor
        CHECK(mergedKind(BodyKind::Planet, false, BodyKind::BlackHole, 1.0) == BodyKind::BlackHole);
    }
}
