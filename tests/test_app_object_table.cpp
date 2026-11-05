#include <doctest.h>
#include <cmath>
#include "app/object_table.h"

using namespace cosmos;
using namespace cosmos::app;

// The object table of the left panel (V1.1 step 2): parent, distance and speed; filter; sort.
namespace {

ObjectRow bodyRow(std::uint64_t id, const char* name, BodyKind kind)
{
    ObjectRow r;
    r.id = id;
    r.type = ObjectType::Body;
    r.name = name;
    r.kind = kind;
    return r;
}

ObjectRow shuttleRow(std::uint64_t id, const char* name)
{
    ObjectRow r;
    r.id = id;
    r.type = ObjectType::Shuttle;
    r.name = name;
    r.controller = "Coast";
    return r;
}

TableState st(std::uint64_t id, Vec3 r, Vec3 v, double mu, std::uint64_t declared = 0)
{
    TableState s;
    s.id = id;
    s.r = r;
    s.v = v;
    s.mu = mu;
    s.declaredParent = declared;
    return s;
}

// A star, a planet at 1 AU with a moon, and a shuttle near the planet
struct System {
    std::vector<ObjectRow> objects = {bodyRow(1, "Sol", BodyKind::Star), bodyRow(2, "Terra", BodyKind::Planet),
                                      bodyRow(3, "Luna", BodyKind::Moon), shuttleRow(4, "Probe")};
    std::vector<TableState> bodies = {st(1, {0, 0, 0}, {0, 0, 0}, 1.327e20), st(2, {1.496e11, 0, 0}, {0, 29780, 0}, 3.986e14),
                                      st(3, {1.496e11 + 3.844e8, 0, 0}, {0, 29780 + 1022, 0}, 4.905e12)};
    std::vector<TableState> shuttles = {st(4, {1.496e11 - 7.0e6, 0, 0}, {0, 29780 - 7500, 0}, 0.0)};
};

} // namespace

TEST_SUITE("app object table")
{
    TEST_CASE("parents follow the hierarchy; distance and speed are relative to the parent")
    {
        System s;
        const std::vector<TableRow> rows = buildTableRows(s.objects, s.bodies, s.shuttles);
        REQUIRE(rows.size() == 4);
        CHECK(rows[0].parent == 0);   // the star
        CHECK(std::isnan(rows[0].distance));
        CHECK(rows[0].speed == 0.0);
        CHECK(rows[0].satellites == 1);
        CHECK(rows[1].parent == 1);
        CHECK(rows[1].parentName == "Sol");
        CHECK(rows[1].distance == doctest::Approx(1.496e11));
        CHECK(rows[1].speed == doctest::Approx(29780.0));
        CHECK(rows[1].satellites == 2);   // the moon and the shuttle
        CHECK(rows[2].parent == 2);
        CHECK(rows[2].distance == doctest::Approx(3.844e8));
        CHECK(rows[2].speed == doctest::Approx(1022.0));
        CHECK(rows[3].parent == 2);
        CHECK(rows[3].kind == "Coast");
        CHECK(rows[3].distance == doctest::Approx(7.0e6));
        CHECK(rows[3].speed == doctest::Approx(7500.0));
        CHECK(rows[1].kind == "Planet");
    }

    TEST_CASE("an orbit-mode parent wins; an object without a state has no numbers")
    {
        System s;
        s.bodies[2].declaredParent = 1;   // the user chose "orbit around Sol"
        s.objects.push_back(bodyRow(9, "Ghost", BodyKind::Asteroid));
        const std::vector<TableRow> rows = buildTableRows(s.objects, s.bodies, s.shuttles);
        CHECK(rows[2].parent == 1);
        CHECK(rows[2].parentName == "Sol");
        CHECK(rows[4].parent == 0);
        CHECK(std::isnan(rows[4].distance));
        CHECK(std::isnan(rows[4].speed));
        CHECK(buildTableRows({}, {}, {}).empty());
    }

    TEST_CASE("the filter matches name, kind and parent, case-insensitively")
    {
        System s;
        const std::vector<TableRow> rows = buildTableRows(s.objects, s.bodies, s.shuttles);
        CHECK(matchesFilter(rows[2], ""));
        CHECK(matchesFilter(rows[2], "   "));
        CHECK(matchesFilter(rows[2], "LUN"));
        CHECK(matchesFilter(rows[2], " moon "));
        CHECK(matchesFilter(rows[2], "terr"));   // its parent
        CHECK_FALSE(matchesFilter(rows[2], "sol"));
        CHECK(filterAndSort(rows, "terra", TableColumn::Name, true).size() == 3);   // Terra, and the two children of Terra
        CHECK(filterAndSort(rows, "zzz", TableColumn::Name, true).empty());
    }

    TEST_CASE("sorting by each column; unknown numbers and missing parents last")
    {
        System s;
        s.objects.push_back(bodyRow(9, "ghost", BodyKind::Asteroid));
        std::vector<TableRow> rows = buildTableRows(s.objects, s.bodies, s.shuttles);
        auto ids = [](const std::vector<TableRow>& r) {
            std::vector<std::uint64_t> out;
            for (const TableRow& t : r) out.push_back(t.id);
            return out;
        };
        sortRows(rows, TableColumn::Name, true);
        CHECK(ids(rows) == std::vector<std::uint64_t>{9, 3, 4, 1, 2});   // ghost, Luna, Probe, Sol, Terra
        sortRows(rows, TableColumn::Name, false);
        CHECK(ids(rows) == std::vector<std::uint64_t>{2, 1, 4, 3, 9});
        sortRows(rows, TableColumn::Distance, true);
        CHECK(ids(rows) == std::vector<std::uint64_t>{4, 3, 2, 1, 9});   // NaN (Sol, ghost) last, in their previous order
        sortRows(rows, TableColumn::Distance, false);
        CHECK(ids(rows).front() == 2);
        CHECK(ids(rows)[2] == 4);
        CHECK((ids(rows)[3] == 1 && ids(rows)[4] == 9));
        sortRows(rows, TableColumn::Speed, false);
        CHECK(ids(rows).front() == 2);
        CHECK(ids(rows).back() == 9);
        sortRows(rows, TableColumn::Parent, true);
        CHECK(rows[0].parentName == "Sol");
        CHECK(rows[3].parent == 0);
        CHECK(rows[4].parent == 0);
        sortRows(rows, TableColumn::Kind, true);
        CHECK(rows[0].kind == "Asteroid");
        CHECK(rows[1].kind == "Coast");
    }
}
