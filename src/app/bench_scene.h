#pragma once
// Benchmark scene helper (plan 14 "Performance (M8)", 17): "Lucerna + N shuttles" for the release benchmark of the app
// (--bench-shuttles N). Adds N shuttles to the scene of an EditorModel through the editor's own API (add, orbit, fuel), so
// what is measured is a scene the user could have built by hand. No OpenGL, no ImGui.
//
// Placement is deterministic (the counter RNG of the shuttle number) and spread like the "Lucerna + 200 shuttles"
// determinism test of tests/test_determinism.cpp: the four autopilots in turn, the hosts in a fixed ten-slot cycle that
// puts half of the fleet around Domus (the body a benchmark follows) and the rest around its moon, the gas giant, the
// other planets and the star. A host that the scene does not have is skipped; when none of the names exist every
// body that has a radius is used in turn.
//
// Every shuttle gets `fuelKg` of fuel (default 150 kg, about 220 m/s of delta-v, against the stock 3000 kg): the stock tank
// lets a Retrograde autopilot brake out of any orbit and fall into its planet within the first minutes, so a high-warp run
// would lose a good part of the fleet in the warm-up and measure fewer than N shuttles. With the small tank all N stay
// in orbit (the orbits are placed at 2 to 6 body radii, clear of the atmosphere even after the burn) and the physics cost
// per live shuttle is the one of a coasting shuttle, which is what the plan 17 budget talks about.
#include <cstdint>
#include <string>
#include <vector>
#include "app/editor_model.h"
#include "core/controllers.h"

namespace cosmos {
namespace app {

inline constexpr double kBenchDefaultFuel_kg = 150.0;

// Adds `count` shuttles (count >= 0). Fails (nothing guaranteed about partial additions) with the editor's message when the
// editor is locked or an edit is refused. `ids` (optional) receives the new shuttle ids in order.
// `controllers` (optional, non-empty) replaces the default cycle Coast, Prograde, Retrograde, Wander.
EditResult addBenchShuttles(EditorModel& editor, int count, double fuelKg = kBenchDefaultFuel_kg,
                            std::vector<std::uint64_t>* ids = nullptr, const std::vector<ControllerKind>* controllers = nullptr);

} // namespace app
} // namespace cosmos
