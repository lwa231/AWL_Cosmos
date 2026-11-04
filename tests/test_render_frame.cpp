#include <doctest.h>
#include "app/render_frame.h"

using namespace cosmos::app;

TEST_SUITE("render_frame")
{
    TEST_CASE("line batch counts one quad per segment of every strip plus loose segments")
    {
        LineBatch b;
        CHECK(b.empty());
        CHECK(b.segmentCount() == 0);

        std::vector<LineVertex> pts(5);
        b.addStrip(pts, 2.0f);                // 4 segments
        b.addStrip(pts.data(), 2, 1.0f);      // 1 segment
        b.addStrip(pts.data(), 1, 1.0f);      // a single point is not a line: ignored
        b.addSegment(pts[0], pts[1], 3.0f);   // 1 loose segment
        CHECK(b.strips.size() == 2);
        CHECK(b.vertices.size() == 7);
        CHECK(b.strips[1].first == 5);
        CHECK(b.segmentCount() == 6);
        CHECK_FALSE(b.empty());
        b.clear();
        CHECK(b.empty());
    }

    TEST_CASE("clearObjects empties every array but keeps capacity")
    {
        RenderFrame f;
        f.bodies.resize(100);
        f.lights.resize(3);
        f.shuttles.resize(200);
        f.overlay.addSegment(LineVertex{}, LineVertex{}, 1.0f);
        const std::size_t cap = f.shuttles.capacity();
        f.clearObjects();
        CHECK(f.bodies.empty());
        CHECK(f.lights.empty());
        CHECK(f.shuttles.empty());
        CHECK(f.overlay.empty());
        CHECK(f.shuttles.capacity() == cap);
    }
}
