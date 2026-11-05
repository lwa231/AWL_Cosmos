#include "app/object_table.h"
#include <algorithm>
#include <cctype>
#include <map>
#include "core/hierarchy.h"
#include "core/kinds.h"

namespace cosmos {
namespace app {

namespace {

std::string lowered(std::string_view s)
{
    std::string out(s);
    for (char& c : out) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return out;
}

// The body a hierarchy node stands for: the leaf itself, or the heavier child chain down to a leaf (childA is heavier).
int leafOf(const Hierarchy& h, int node, int bodyCount)
{
    while (node >= bodyCount) node = h.nodes[static_cast<std::size_t>(node)].childA;
    return node;
}

} // namespace

std::vector<TableRow> buildTableRows(const std::vector<ObjectRow>& objects, const std::vector<TableState>& bodies,
                                     const std::vector<TableState>& shuttles)
{
    // Parent by the hierarchy of the bodies
    std::map<std::uint64_t, std::uint64_t> parentOf;   // id -> parent id
    std::map<std::uint64_t, const TableState*> stateOf;
    const int n = static_cast<int>(bodies.size());
    if (n > 0) {
        std::vector<Vec3> r(bodies.size()), v(bodies.size());
        std::vector<double> mu(bodies.size());
        for (std::size_t i = 0; i < bodies.size(); ++i) {
            r[i] = bodies[i].r;
            v[i] = bodies[i].v;
            mu[i] = bodies[i].mu;
        }
        const Hierarchy h = buildHierarchy(r, v, mu);
        for (int i = 0; i < n; ++i) {
            const int p = h.bodyParent[static_cast<std::size_t>(i)];
            if (p >= 0) {
                const int leaf = leafOf(h, p, n);
                if (leaf != i) parentOf[bodies[static_cast<std::size_t>(i)].id] = bodies[static_cast<std::size_t>(leaf)].id;
            }
        }
        if (!shuttles.empty()) {
            HillTable table;
            buildHillTable(h, table);
            for (const TableState& s : shuttles) {
                const int node = shuttleParent(h, table, s.r);
                if (node >= 0) parentOf[s.id] = bodies[static_cast<std::size_t>(leafOf(h, node, n))].id;
            }
        }
    }
    for (const TableState& b : bodies) stateOf[b.id] = &b;
    for (const TableState& s : shuttles) stateOf[s.id] = &s;
    // The orbit-mode parent wins (it is what the user chose), as long as that body has a state
    for (const auto* list : {&bodies, &shuttles})
        for (const TableState& s : *list)
            if (s.declaredParent != 0 && s.declaredParent != s.id) {
                auto it = stateOf.find(s.declaredParent);
                if (it != stateOf.end() && it->second->mu > 0.0) parentOf[s.id] = s.declaredParent;
            }

    std::map<std::uint64_t, std::string> nameOf;
    for (const ObjectRow& o : objects) nameOf[o.id] = o.name;

    std::vector<TableRow> rows;
    rows.reserve(objects.size());
    for (const ObjectRow& o : objects) {
        TableRow t;
        t.id = o.id;
        t.type = o.type;
        t.name = o.name;
        t.kind = o.type == ObjectType::Body ? std::string(kindName(o.kind)) : o.controller;
        auto st = stateOf.find(o.id);
        auto pa = parentOf.find(o.id);
        if (pa != parentOf.end()) {
            t.parent = pa->second;
            auto nm = nameOf.find(t.parent);
            t.parentName = nm != nameOf.end() ? nm->second : "#" + std::to_string(t.parent);
        }
        if (st != stateOf.end()) {
            const TableState& me = *st->second;
            auto ps = t.parent != 0 ? stateOf.find(t.parent) : stateOf.end();
            if (ps != stateOf.end()) {
                t.distance = norm(me.r - ps->second->r);
                t.speed = norm(me.v - ps->second->v);
            } else {
                t.speed = norm(me.v);
            }
        }
        rows.push_back(std::move(t));
    }
    std::map<std::uint64_t, int> count;
    for (const TableRow& t : rows)
        if (t.parent != 0) ++count[t.parent];
    for (TableRow& t : rows) {
        auto it = count.find(t.id);
        if (it != count.end()) t.satellites = it->second;
    }
    return rows;
}

bool matchesFilter(const TableRow& row, std::string_view filter)
{
    std::size_t a = 0, b = filter.size();
    while (a < b && std::isspace(static_cast<unsigned char>(filter[a]))) ++a;
    while (b > a && std::isspace(static_cast<unsigned char>(filter[b - 1]))) --b;
    if (a == b) return true;
    const std::string f = lowered(filter.substr(a, b - a));
    return lowered(row.name).find(f) != std::string::npos || lowered(row.kind).find(f) != std::string::npos ||
           lowered(row.parentName).find(f) != std::string::npos;
}

void sortRows(std::vector<TableRow>& rows, TableColumn column, bool ascending)
{
    auto textLess = [ascending](const std::string& x, const std::string& y, bool& decided) {
        const std::string lx = lowered(x), ly = lowered(y);
        decided = lx != ly;
        return ascending ? lx < ly : ly < lx;
    };
    auto numLess = [ascending](double x, double y, bool& decided) {
        const bool nx = std::isnan(x), ny = std::isnan(y);
        if (nx || ny) {
            decided = nx != ny;
            return ny && !nx;   // NaN last in both directions
        }
        decided = x != y;
        return ascending ? x < y : y < x;
    };
    std::stable_sort(rows.begin(), rows.end(), [&](const TableRow& x, const TableRow& y) {
        bool decided = false;
        bool less = false;
        switch (column) {
        case TableColumn::Name: less = textLess(x.name, y.name, decided); break;
        case TableColumn::Kind: less = textLess(x.kind, y.kind, decided); break;
        case TableColumn::Parent:
            if ((x.parent == 0) != (y.parent == 0)) return y.parent == 0;   // no parent last
            less = textLess(x.parentName, y.parentName, decided);
            break;
        case TableColumn::Distance: less = numLess(x.distance, y.distance, decided); break;
        case TableColumn::Speed: less = numLess(x.speed, y.speed, decided); break;
        }
        return decided && less;
    });
}

std::vector<TableRow> filterAndSort(std::vector<TableRow> rows, std::string_view filter, TableColumn column, bool ascending)
{
    rows.erase(std::remove_if(rows.begin(), rows.end(), [&](const TableRow& r) { return !matchesFilter(r, filter); }), rows.end());
    sortRows(rows, column, ascending);
    return rows;
}

} // namespace app
} // namespace cosmos
