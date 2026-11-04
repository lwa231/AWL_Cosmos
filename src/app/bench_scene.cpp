#include "app/bench_scene.h"
#include <array>
#include <cmath>
#include "core/controllers.h"
#include "core/rng.h"

namespace cosmos {
namespace app {

namespace {

constexpr double kPi = 3.14159265358979323846;

struct HostSlot {
    const char* name;
    double lo, hi;   // orbit radius range in body radii
};

// Ten slots; the shuttle number picks slot j % 10. Domus first and five times: half of the fleet is where the benchmark looks.
constexpr std::array<HostSlot, 10> kSlots = {{
    {"Domus", 2.2, 4.5}, {"Domus", 2.2, 4.5}, {"Domus", 2.2, 4.5}, {"Domus", 2.2, 4.5}, {"Domus", 2.2, 4.5},
    {"Custos", 2.0, 4.0}, {"Maxima", 1.6, 3.0}, {"Rubor", 2.0, 6.0}, {"Velum", 2.0, 6.0}, {"Lucerna", 2.0, 6.0},
}};

constexpr std::array<ControllerKind, 4> kControllers = {ControllerKind::Coast, ControllerKind::Prograde, ControllerKind::Retrograde,
                                                        ControllerKind::Wander};

const SceneBody* byName(const Scene& s, const char* name)
{
    for (const SceneBody& b : s.bodies)
        if (b.name == name) return &b;
    return nullptr;
}

} // namespace

EditResult addBenchShuttles(EditorModel& ed, int count, double fuelKg, std::vector<std::uint64_t>* ids,
                            const std::vector<ControllerKind>* controllers)
{
    if (count < 0) return EditResult::failure("The shuttle count must not be negative.");
    // Hosts present in the scene, in slot order; a scene without any of the named bodies uses all its bodies in turn.
    struct Host {
        std::uint64_t id;
        double radius, lo, hi;
    };
    std::vector<Host> hosts;
    for (const HostSlot& s : kSlots)
        if (const SceneBody* b = byName(ed.scene(), s.name)) hosts.push_back({b->id, b->radius, s.lo, s.hi});
    if (hosts.empty())
        for (const SceneBody& b : ed.scene().bodies)
            if (b.radius > 0.0) hosts.push_back({b.id, b.radius, 2.0, 6.0});
    if (hosts.empty()) return EditResult::failure("The scene has no body for the benchmark shuttles to orbit.");

    for (int j = 0; j < count; ++j) {
        Rng rng = Rng::forTick(static_cast<std::uint64_t>(j) + 1, 4242);
        const Host& h = hosts[static_cast<std::size_t>(j) % hosts.size()];
        const ControllerKind kind = (controllers && !controllers->empty())
                                        ? (*controllers)[static_cast<std::size_t>(j) % controllers->size()]
                                        : kControllers[static_cast<std::size_t>(j) % kControllers.size()];
        EditResult r = ed.addShuttle(kind);
        if (!r) return r;
        const std::uint64_t id = r.id;
        SceneOrbit o;
        o.parent = h.id;
        o.a = h.radius * rng.uniform(h.lo, h.hi);
        o.e = rng.uniform(0.0, 0.03);
        o.argPeriapsis_deg = rng.uniform(0.0, 360.0);
        o.meanAnomaly_deg = rng.uniform(0.0, 360.0);
        o.retrograde = rng.uniform() < 0.25;
        r = ed.setOrbit(id, o);
        if (!r) return r;
        r = ed.updateShuttle(id, [fuelKg](SceneShuttle& s) { s.fuelMass = fuelKg; });
        if (!r) return r;
        if (ids) ids->push_back(id);
    }
    ed.clearSelection();
    return EditResult::success();
}

} // namespace app
} // namespace cosmos
