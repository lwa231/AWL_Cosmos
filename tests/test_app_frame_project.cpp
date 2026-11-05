#include <doctest.h>
#include <cmath>
#include "app/frame_project.h"

using namespace cosmos;
using namespace cosmos::app;

namespace {
RenderBody bodyAt(float x, float y, float z, float r)
{
    RenderBody b;
    b.x = x; b.y = y; b.z = z; b.radius = r;
    return b;
}
} // namespace

TEST_CASE("frame project: top-down orthographic is the identity")
{
    View3D v;
    ProjectContext c{&v, 1.0e5, 800, 600};
    RenderFrame f;
    f.bodies = {bodyAt(1.0e7f, -2.0e7f, 0.0f, 6.0e6f), bodyAt(0.0f, 0.0f, 0.0f, 7.0e8f)};
    RenderShuttle s; s.x = 3.0e6f; s.heading = 0.7f;
    f.shuttles.push_back(s);
    LineVertex a{1.0f, 2.0f}, b{3.0f, 4.0f};
    f.overlay.addSegment(a, b, 2.0f);
    projectFrame(f, c);
    REQUIRE(f.bodies.size() == 2);
    CHECK(f.bodies[0].x == doctest::Approx(1.0e7));
    CHECK(f.bodies[0].y == doctest::Approx(-2.0e7));
    CHECK(f.bodies[0].radius == doctest::Approx(6.0e6));
    CHECK(f.shuttles[0].heading == doctest::Approx(0.7));
    REQUIRE(f.overlay.segments.size() == 1);
    CHECK(f.overlay.segments[0].b.y == doctest::Approx(4.0));
}

TEST_CASE("frame project: tilted perspective sorts far to near and drops what is behind the eye")
{
    View3D v;
    v.setProjection(Projection::Perspective);
    v.setOrientation(-1.5707963267948966, 0.3);   // looking along +y, slightly from above
    ProjectContext c{&v, 1.0e5, 800, 600};
    const double D = v.eyeDistance(c.metresPerPixel, c.viewportHeight);
    RenderFrame f;
    f.bodies = {bodyAt(0.0f, static_cast<float>(-0.5 * D), 0.0f, 1.0e6f),   // nearer the eye
                bodyAt(0.0f, static_cast<float>(0.5 * D), 0.0f, 1.0e6f),    // further
                bodyAt(0.0f, static_cast<float>(-3.0 * D), 0.0f, 1.0e6f)};  // behind the eye
    std::vector<std::uint32_t> order;
    projectFrame(f, c, &order);
    REQUIRE(f.bodies.size() == 2);
    REQUIRE(order.size() == 2);
    CHECK(order[0] == 1);   // far first
    CHECK(order[1] == 0);
    CHECK(f.bodies[1].radius > f.bodies[0].radius);   // the near one looks bigger

    // A strip that passes behind the eye is split
    RenderFrame g;
    std::vector<LineVertex> strip;
    for (int k = -4; k <= 4; ++k) {
        LineVertex lv;
        lv.y = static_cast<float>(k * D);
        strip.push_back(lv);
    }
    g.underlay.addStrip(strip, 1.0f);
    projectFrame(g, c);
    REQUIRE(g.underlay.strips.size() == 1);
    CHECK(g.underlay.strips[0].count < strip.size());
}

TEST_CASE("frame project: circles become loops and the grids are built")
{
    View3D v;
    v.setOrientation(0.0, 0.5);
    ProjectContext c{&v, 1.0e5, 800, 600};
    RenderFrame f;
    RenderCircle ci; ci.radius = 1.0e7f;
    f.circles.push_back(ci);
    RenderWell w; w.mu = 1.0e20f; w.radius = 7.0e8f;
    f.wells.push_back(w);
    projectFrame(f, c);
    CHECK(f.circles.empty());
    CHECK(f.wells.empty());   // tilted: the sheet replaces the rings
    CHECK(f.underlay.strips.size() == 1);

    LineBatch grid;
    appendEclipticGrid(grid, c, {1.234e9, -5.0e8, 0.0});
    CHECK(grid.strips.size() > 10);
    LineBatch sheet;
    std::vector<RenderWell> wells{w};
    appendWellSheet(sheet, wells, c, {});
    CHECK(sheet.strips.size() == 2 * 41);
    // Depth: deepest at the mass, zero at the edge potential
    const double uEdge = std::log(1.0e20 / 1.0e12);
    CHECK(wellSheetDepth(wells, 1.0e12, 0.0, uEdge, 100.0) == doctest::Approx(0.0));
    CHECK(wellSheetDepth(wells, 0.0, 0.0, uEdge, 100.0) == doctest::Approx(-100.0 * std::log(1.0e12 / 7.0e8) / 8.0));
    CHECK(wellSheetDepth(wells, 1.0e10, 0.0, uEdge, 100.0) < 0.0);
}
