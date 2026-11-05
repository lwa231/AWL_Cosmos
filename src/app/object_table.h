#pragma once
// The rows of the left panel's object table (V1.1 step 2): Name | Kind | Parent | Distance to parent | Speed, with a search
// filter and sorting by any column. GL-free and ImGui-free: the panel collects the states, this file fills the parent,
// distance and speed, filters and sorts.
//
// Parent. An object placed in orbit mode (the editor's "Orbit around X") shows that parent. Every other object gets the body
// it orbits by the hierarchy of plan 15 (2c), the same choice the overlays make: a body's parent is the sibling it is bound to
// (a barycentre node stands for its heaviest body), a shuttle's the smallest Hill sphere that contains it. The dominant body
// has no parent. Distance and speed are relative to the parent (centre to centre, and the speed in the parent's frame); without
// a parent the speed is the one in the simulation frame and the distance is unknown.
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <string_view>
#include <vector>
#include "app/editor_model.h"
#include "core/vec3.h"

namespace cosmos {
namespace app {

enum class TableColumn { Name = 0, Kind, Parent, Distance, Speed };

struct TableRow {
    std::uint64_t id = 0;
    ObjectType type = ObjectType::Body;
    std::string name;
    std::string kind;                 // the body kind's name, or the shuttle's controller
    std::uint64_t parent = 0;         // 0: none
    std::string parentName;
    double distance = std::numeric_limits<double>::quiet_NaN();   // m, to the parent's centre (NaN: no parent or no state)
    double speed = std::numeric_limits<double>::quiet_NaN();      // m/s, relative to the parent (NaN: no state)
    int satellites = 0;               // rows whose parent is this one
};

// The state of an object at the display time. mu = 0 for a shuttle. declaredParent: the orbit-mode parent, 0 if none.
struct TableState {
    std::uint64_t id = 0;
    Vec3 r, v;
    double mu = 0.0;
    std::uint64_t declaredParent = 0;
};

// Rows in the order of `objects`, with parent, distance, speed and satellite counts filled from the states (matched by id;
// an object without a state keeps NaN distance and speed). `bodies` are the body states, `shuttles` the shuttle states.
std::vector<TableRow> buildTableRows(const std::vector<ObjectRow>& objects, const std::vector<TableState>& bodies,
                                     const std::vector<TableState>& shuttles);

// Case-insensitive substring match of the filter against the name, the kind and the parent's name. An empty (or all-blank)
// filter matches everything.
bool matchesFilter(const TableRow& row, std::string_view filter);

// Stable sort by one column. Text columns compare case-insensitively; NaN distances and speeds (and rows without a parent in
// the Parent column) always go last, in either direction. Equal keys keep their order.
void sortRows(std::vector<TableRow>& rows, TableColumn column, bool ascending);

// matchesFilter, then sortRows.
std::vector<TableRow> filterAndSort(std::vector<TableRow> rows, std::string_view filter, TableColumn column, bool ascending);

} // namespace app
} // namespace cosmos
