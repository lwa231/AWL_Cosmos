#include <doctest.h>
#include <cstdint>
#include <vector>
#include "core/events.h"

using namespace cosmos;

namespace {
// Deterministic pseudo-random numbers for building test data (SplitMix64; no global RNG).
struct Mix {
    std::uint64_t s;
    std::uint64_t next()
    {
        std::uint64_t z = (s += 0x9E3779B97F4A7C15ull);
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
        return z ^ (z >> 31);
    }
    std::uint64_t below(std::uint64_t n) { return next() % n; }
};

// A varied set of events with deliberately many ties in t and id.
std::vector<Event> sampleEvents(std::size_t count, std::uint64_t seed)
{
    Mix r{seed};
    std::vector<Event> v;
    for (std::size_t k = 0; k < count; ++k) {
        const double t = static_cast<double>(r.below(6)) * 0.5; // 6 distinct times
        const BodyId a = 1 + r.below(5), b = 1 + r.below(5);
        switch (r.below(6)) {
        case 0: v.push_back(makeCollision(t, a, b, static_cast<double>(r.below(4)), {double(r.below(3)), 0, 0})); break;
        case 1: v.push_back(makeMerge(t, a, b)); break;
        case 2: v.push_back(makeShuttleDestroyed(t, a, b)); break;
        case 3: v.push_back(makeAtmosphereEntry(t, a, b)); break;
        case 4: v.push_back(makeKindChanged(t, a, BodyKind::Planet, r.below(2) ? BodyKind::Star : BodyKind::BlackHole)); break;
        default: v.push_back(makeFuelEmpty(t, a)); break;
        }
    }
    return v;
}

bool sameEvent(const Event& l, const Event& r)
{
    return l.type == r.type && l.t == r.t && l.id == r.id && l.other == r.other && l.relSpeed == r.relSpeed &&
           l.point.x == r.point.x && l.point.y == r.point.y && l.point.z == r.point.z &&
           l.oldKind == r.oldKind && l.newKind == r.newKind;
}

bool sameSequence(const std::vector<Event>& l, const std::vector<Event>& r)
{
    if (l.size() != r.size()) return false;
    for (std::size_t i = 0; i < l.size(); ++i) if (!sameEvent(l[i], r[i])) return false;
    return true;
}

// Split events into `threads` buffers: event k goes to buffer (k * stride + offset) % threads.
std::vector<std::vector<Event>> split(const std::vector<Event>& all, std::size_t threads, std::size_t stride, std::size_t offset)
{
    std::vector<std::vector<Event>> bufs(threads);
    for (std::size_t k = 0; k < all.size(); ++k) bufs[(k * stride + offset) % threads].push_back(all[k]);
    return bufs;
}
}

TEST_SUITE("events")
{
    TEST_CASE("factories fill the documented fields")
    {
        const Event c = makeCollision(1.5, 3, 7, 12.0, {1, 2, 3});
        CHECK(c.type == EventType::Collision);
        CHECK(c.t == 1.5);
        CHECK(c.id == 3);
        CHECK(c.other == 7);
        CHECK(c.relSpeed == 12.0);
        CHECK(c.point.y == 2.0);

        const Event m = makeMerge(2.0, 7, 3);
        CHECK(m.type == EventType::Merge);
        CHECK(m.id == 7);
        CHECK(m.other == 3);

        const Event d = makeShuttleDestroyed(3.0, 11, 7);
        CHECK(d.type == EventType::ShuttleDestroyed);
        CHECK(d.id == 11);
        CHECK(d.other == 7);

        const Event e = makeAtmosphereEntry(4.0, 11, 7);
        CHECK(e.type == EventType::AtmosphereEntry);
        CHECK(e.id == 11);
        CHECK(e.other == 7);

        const Event k = makeKindChanged(5.0, 7, BodyKind::NeutronStar, BodyKind::BlackHole);
        CHECK(k.type == EventType::KindChanged);
        CHECK(k.oldKind == BodyKind::NeutronStar);
        CHECK(k.newKind == BodyKind::BlackHole);

        const Event f = makeFuelEmpty(6.0, 11);
        CHECK(f.type == EventType::FuelEmpty);
        CHECK(f.id == 11);
    }

    TEST_CASE("sorted by (t_sim, id)")
    {
        EventQueue q;
        q.push(makeFuelEmpty(2.0, 4));
        q.push(makeMerge(1.0, 9, 2));
        q.push(makeFuelEmpty(1.0, 3));
        q.push(makeShuttleDestroyed(2.0, 1, 8));
        q.push(makeKindChanged(0.5, 9, BodyKind::Planet, BodyKind::Star));
        q.sort();
        const auto& e = q.events();
        REQUIRE(e.size() == 5);
        CHECK(e[0].t == 0.5);
        CHECK(e[1].t == 1.0); CHECK(e[1].id == 3);
        CHECK(e[2].t == 1.0); CHECK(e[2].id == 9);
        CHECK(e[3].t == 2.0); CHECK(e[3].id == 1);
        CHECK(e[4].t == 2.0); CHECK(e[4].id == 4);
    }

    TEST_CASE("same time and id: Collision, then Merge, then KindChanged")
    {
        EventQueue q;
        q.push(makeKindChanged(1.0, 5, BodyKind::NeutronStar, BodyKind::BlackHole));
        q.push(makeMerge(1.0, 5, 6));
        q.push(makeCollision(1.0, 5, 6, 3.0, {}));
        q.sort();
        CHECK(q.events()[0].type == EventType::Collision);
        CHECK(q.events()[1].type == EventType::Merge);
        CHECK(q.events()[2].type == EventType::KindChanged);
    }

    TEST_CASE("merging per-thread buffers gives the same sequence for any thread count and split")
    {
        const std::vector<Event> all = sampleEvents(200, 12345);
        const EventQueue ref = EventQueue::merge(split(all, 1, 1, 0));
        REQUIRE(ref.size() == all.size());

        // sorted by (t, id) throughout
        for (std::size_t i = 1; i < ref.size(); ++i) {
            const Event& p = ref.events()[i - 1];
            const Event& c = ref.events()[i];
            CHECK((p.t < c.t || (p.t == c.t && p.id <= c.id)));
        }

        for (std::size_t threads : {2u, 3u, 4u, 7u, 16u}) {
            for (std::size_t stride : {1u, 5u}) {
                for (std::size_t offset : {0u, 1u}) {
                    const EventQueue q = EventQueue::merge(split(all, threads, stride, offset));
                    CHECK(sameSequence(q.events(), ref.events()));
                }
            }
        }
    }

    TEST_CASE("buffer order does not matter, even when events tie on (t, id)")
    {
        // heavy ties: every event at t = 1 with id 1 or 2
        std::vector<Event> all;
        for (int k = 0; k < 40; ++k) {
            all.push_back(makeCollision(1.0, 1 + (k % 2), 10 + (k % 5), double(k % 3), {double(k % 4), 0, 0}));
            all.push_back(makeMerge(1.0, 1 + (k % 2), 20 + (k % 3)));
        }
        const EventQueue ref = EventQueue::merge({all});
        std::vector<Event> rev(all.rbegin(), all.rend());
        CHECK(sameSequence(EventQueue::merge({rev}).events(), ref.events()));
        CHECK(sameSequence(EventQueue::merge(split(all, 5, 3, 2)).events(), ref.events()));
        CHECK(sameSequence(EventQueue::merge(split(rev, 3, 1, 1)).events(), ref.events()));
    }

    TEST_CASE("sort is idempotent and merge keeps every event")
    {
        const std::vector<Event> all = sampleEvents(60, 99);
        EventQueue q = EventQueue::merge(split(all, 4, 1, 0));
        const std::vector<Event> once = q.events();
        q.sort();
        CHECK(sameSequence(q.events(), once));
        CHECK(q.size() == all.size());
    }

    TEST_CASE("drain returns the events and empties the queue")
    {
        EventQueue q;
        CHECK(q.empty());
        q.push(makeFuelEmpty(1.0, 2));
        q.append({makeFuelEmpty(2.0, 3), makeFuelEmpty(3.0, 4)});
        CHECK(q.size() == 3);
        const std::vector<Event> out = q.drain();
        CHECK(out.size() == 3);
        CHECK(q.empty());
        q.push(makeFuelEmpty(1.0, 2));
        q.clear();
        CHECK(q.empty());
    }
}
