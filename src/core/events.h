#pragma once
#include <cstdint>
#include <vector>
#include "core/bodies.h"
#include "core/vec3.h"

namespace cosmos {

// Typed events (plan section 4.7). Consumers: effects layer, UI log, warp control, later ML.
// The enum order is also the tie-break order in the queue, so for one body at one time a Collision sorts
// before its Merge, and the Merge before its KindChanged.
enum class EventType : std::uint8_t {
    Collision,        // id = a,        other = b,    relSpeed, point
    Merge,            // id = survivor, other = absorbed
    ShuttleDestroyed, // id = shuttle,  other = by (the body hit; 0 = none)
    AtmosphereEntry,  // id = shuttle,  other = body
    KindChanged,      // id = body,     oldKind -> newKind
    FuelEmpty         // id = shuttle
};

struct Event {
    EventType type = EventType::Collision;
    double t = 0.0;          // t_sim at which it happened
    BodyId id = 0;           // primary subject (the id of the sort key)
    BodyId other = 0;        // secondary subject, see EventType
    double relSpeed = 0.0;   // Collision only
    Vec3 point;              // Collision only: contact point
    BodyKind oldKind = BodyKind::Planet; // KindChanged only
    BodyKind newKind = BodyKind::Planet;
};

Event makeCollision(double t, BodyId a, BodyId b, double relSpeed, const Vec3& point);
Event makeMerge(double t, BodyId survivor, BodyId absorbed);
Event makeShuttleDestroyed(double t, BodyId id, BodyId by);
Event makeAtmosphereEntry(double t, BodyId shuttle, BodyId body);
Event makeKindChanged(double t, BodyId id, BodyKind oldKind, BodyKind newKind);
Event makeFuelEmpty(double t, BodyId id);

// Order by (t_sim, id) as in plan 4.5 step 3, with ties broken by type, secondary id and the remaining payload so
// the sorted result is independent of the order the events were pushed in (events equal under all of these are
// identical, so their relative order is invisible). NaN times are not supported.
bool eventBefore(const Event& l, const Event& r);

class EventQueue {
public:
    void push(const Event& e) { items.push_back(e); }
    void append(const std::vector<Event>& more) { items.insert(items.end(), more.begin(), more.end()); }

    // Stable sort by eventBefore. Call before anything consumes the queue (plan 4.5 step 3).
    void sort();

    // Concatenate per-thread buffers (in buffer order) and sort. With a fixed event set the result does not
    // depend on how the work was split over threads.
    static EventQueue merge(const std::vector<std::vector<Event>>& perThread);

    const std::vector<Event>& events() const { return items; }
    std::size_t size() const { return items.size(); }
    bool empty() const { return items.empty(); }
    void clear() { items.clear(); }

    // Hand the queued events to the caller and leave the queue empty.
    std::vector<Event> drain();

private:
    std::vector<Event> items;
};

} // namespace cosmos
