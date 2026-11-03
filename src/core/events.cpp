#include "core/events.h"
#include <algorithm>
#include <utility>

namespace cosmos {

namespace {
Event base(EventType type, double t, BodyId id, BodyId other)
{
    Event e;
    e.type = type;
    e.t = t;
    e.id = id;
    e.other = other;
    return e;
}
}

Event makeCollision(double t, BodyId a, BodyId b, double relSpeed, const Vec3& point)
{
    Event e = base(EventType::Collision, t, a, b);
    e.relSpeed = relSpeed;
    e.point = point;
    return e;
}

Event makeMerge(double t, BodyId survivor, BodyId absorbed) { return base(EventType::Merge, t, survivor, absorbed); }

Event makeShuttleDestroyed(double t, BodyId id, BodyId by) { return base(EventType::ShuttleDestroyed, t, id, by); }

Event makeAtmosphereEntry(double t, BodyId shuttle, BodyId body) { return base(EventType::AtmosphereEntry, t, shuttle, body); }

Event makeKindChanged(double t, BodyId id, BodyKind oldKind, BodyKind newKind)
{
    Event e = base(EventType::KindChanged, t, id, 0);
    e.oldKind = oldKind;
    e.newKind = newKind;
    return e;
}

Event makeFuelEmpty(double t, BodyId id) { return base(EventType::FuelEmpty, t, id, 0); }

bool eventBefore(const Event& l, const Event& r)
{
    if (l.t != r.t) return l.t < r.t;
    if (l.id != r.id) return l.id < r.id;
    if (l.type != r.type) return l.type < r.type;
    if (l.other != r.other) return l.other < r.other;
    // Remaining payload, so the order never depends on which thread buffer an event came from.
    if (l.relSpeed != r.relSpeed) return l.relSpeed < r.relSpeed;
    if (l.point.x != r.point.x) return l.point.x < r.point.x;
    if (l.point.y != r.point.y) return l.point.y < r.point.y;
    if (l.point.z != r.point.z) return l.point.z < r.point.z;
    if (l.oldKind != r.oldKind) return l.oldKind < r.oldKind;
    return l.newKind < r.newKind;
}

void EventQueue::sort() { std::stable_sort(items.begin(), items.end(), eventBefore); }

EventQueue EventQueue::merge(const std::vector<std::vector<Event>>& perThread)
{
    EventQueue q;
    for (const auto& buf : perThread) q.append(buf);
    q.sort();
    return q;
}

std::vector<Event> EventQueue::drain()
{
    std::vector<Event> out;
    out.swap(items);
    return out;
}

} // namespace cosmos
