#include "app/frame_project.h"
#include <algorithm>
#include <cmath>
#include <numeric>

namespace cosmos {
namespace app {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr int kCircleSegments = 72;
constexpr double kSheetSpan = 8.0;   // e-folds of the potential from the sheet edge to full depth

double gridExtent(const ProjectContext& c)
{
    // Half-size of the patch of plane the grids cover: a few screens, so a tilted perspective view reaches its horizon
    // without spending lines far outside the view.
    return 2.5 * static_cast<double>(std::max(c.viewportWidth, c.viewportHeight)) * c.metresPerPixel;
}

void projectBatch(LineBatch& batch, const ProjectContext& c)
{
    if (batch.empty()) return;
    static thread_local LineBatch out;
    static thread_local std::vector<LineVertex> run;
    out.clear();
    auto proj = [&c](LineVertex v, bool& ok) {
        double sx, sy, scale, depth;
        ok = projectToScreen(c, {v.x, v.y, v.z}, sx, sy, scale, depth);
        v.x = static_cast<float>(sx);
        v.y = static_cast<float>(sy);
        v.z = 0.0f;
        return v;
    };
    for (const LineStrip& s : batch.strips) {
        if (s.count < 2 || std::size_t(s.first) + s.count > batch.vertices.size()) continue;
        run.clear();
        for (std::uint32_t k = 0; k < s.count; ++k) {
            bool ok = false;
            const LineVertex v = proj(batch.vertices[s.first + k], ok);
            if (ok) {
                run.push_back(v);
            } else {
                out.addStrip(run, s.widthPx);
                run.clear();
            }
        }
        out.addStrip(run, s.widthPx);
    }
    for (const LineSegment& seg : batch.segments) {
        bool oa = false, ob = false;
        const LineVertex a = proj(seg.a, oa), b = proj(seg.b, ob);
        if (oa && ob) out.addSegment(a, b, seg.widthPx);
    }
    std::swap(batch.vertices, out.vertices);
    std::swap(batch.strips, out.strips);
    std::swap(batch.segments, out.segments);
}

void circleColour(CircleKind k, LineVertex& v)
{
    switch (k) {
    case CircleKind::Hill: v.r = 0.35f; v.g = 0.85f; v.b = 1.00f; v.a = 0.80f; break;
    case CircleKind::Soi: v.r = 0.45f; v.g = 1.00f; v.b = 0.50f; v.a = 0.80f; break;
    case CircleKind::Roche: v.r = 1.00f; v.g = 0.55f; v.b = 0.25f; v.a = 0.90f; break;
    }
}

} // namespace

bool projectToScreen(const ProjectContext& c, const Vec3& rel, double& sx, double& sy, double& scale, double& depth)
{
    const Projected p = c.view->project(rel, c.metresPerPixel, c.viewportHeight);
    sx = p.x * c.metresPerPixel;
    sy = p.y * c.metresPerPixel;
    scale = p.pixelsPerMetre * c.metresPerPixel;
    depth = p.depth;
    return p.visible;
}

void projectFrame(RenderFrame& frame, const ProjectContext& c, std::vector<std::uint32_t>* bodyOrder)
{
    if (!c.view) return;
    double sx, sy, scale, depth;

    // Circles first: they become line loops of the underlay, which is projected below.
    for (const RenderCircle& ci : frame.circles) {
        static thread_local std::vector<LineVertex> loop;
        loop.clear();
        for (int k = 0; k <= kCircleSegments; ++k) {
            const double a = 2.0 * kPi * k / kCircleSegments;
            LineVertex v;
            v.x = static_cast<float>(ci.x + ci.radius * std::cos(a));
            v.y = static_cast<float>(ci.y + ci.radius * std::sin(a));
            v.z = ci.z;
            circleColour(ci.kind, v);
            loop.push_back(v);
        }
        frame.underlay.addStrip(loop, 1.5f);
    }
    frame.circles.clear();

    // Bodies: project, drop the ones behind the eye, sort far to near.
    {
        struct Item { double depth; std::uint32_t index; };
        static thread_local std::vector<Item> items;
        static thread_local std::vector<RenderBody> sorted;
        items.clear();
        for (std::uint32_t i = 0; i < frame.bodies.size(); ++i) {
            RenderBody& b = frame.bodies[i];
            if (!projectToScreen(c, {b.x, b.y, b.z}, sx, sy, scale, depth)) continue;
            b.x = static_cast<float>(sx);
            b.y = static_cast<float>(sy);
            b.radius = static_cast<float>(b.radius * scale);
            b.z = 0.0f;
            items.push_back({depth, i});
        }
        std::stable_sort(items.begin(), items.end(), [](const Item& a, const Item& b) { return a.depth > b.depth; });
        sorted.clear();
        if (bodyOrder) bodyOrder->clear();
        for (const Item& it : items) {
            sorted.push_back(frame.bodies[it.index]);
            if (bodyOrder) bodyOrder->push_back(it.index);
        }
        std::swap(frame.bodies, sorted);
    }
    for (RenderLight& l : frame.lights) {   // a light behind the eye still lights: keep its direction, push it far away
        if (projectToScreen(c, {l.x, l.y, l.z}, sx, sy, scale, depth)) {
            l.x = static_cast<float>(sx);
            l.y = static_cast<float>(sy);
        }
        l.z = 0.0f;
    }
    {
        std::size_t n = 0;
        const double D = c.view->eyeDistance(c.metresPerPixel, c.viewportHeight);
        for (RenderShuttle s : frame.shuttles) {
            if (!projectToScreen(c, {s.x, s.y, s.z}, sx, sy, scale, depth)) continue;
            // Heading: project a short step along the nose
            const double step = 1e-3 * D;
            double hx, hy, hs, hd;
            if (projectToScreen(c, {s.x + step * std::cos(s.heading), s.y + step * std::sin(s.heading), s.z}, hx, hy, hs, hd) &&
                (hx != sx || hy != sy))
                s.heading = static_cast<float>(std::atan2(hy - sy, hx - sx));
            s.x = static_cast<float>(sx);
            s.y = static_cast<float>(sy);
            s.z = 0.0f;
            frame.shuttles[n++] = s;
        }
        frame.shuttles.resize(n);
    }
    {
        std::size_t n = 0;
        for (RenderAtmosphere a : frame.atmospheres) {
            if (!projectToScreen(c, {a.x, a.y, a.z}, sx, sy, scale, depth)) continue;
            a.x = static_cast<float>(sx);
            a.y = static_cast<float>(sy);
            a.radius = static_cast<float>(a.radius * scale);
            a.top = static_cast<float>(a.top * scale);
            a.scaleHeight = static_cast<float>(a.scaleHeight * scale);
            a.z = 0.0f;
            frame.atmospheres[n++] = a;
        }
        frame.atmospheres.resize(n);
    }
    if (c.view->isTopDown()) {
        // Top-down (perspective or orthographic): the contour rings still work, at the plane's scale.
        std::size_t n = 0;
        for (RenderWell w : frame.wells) {
            if (!projectToScreen(c, {w.x, w.y, w.z}, sx, sy, scale, depth)) continue;
            w.x = static_cast<float>(sx);
            w.y = static_cast<float>(sy);
            w.radius = static_cast<float>(w.radius * scale);
            w.mu = static_cast<float>(w.mu * scale);   // keeps Phi's shape: mu / r scales like r
            w.z = 0.0f;
            frame.wells[n++] = w;
        }
        frame.wells.resize(n);
    } else {
        frame.wells.clear();   // tilted: the caller drew the warped sheet instead (appendWellSheet)
    }
    projectBatch(frame.underlay, c);
    projectBatch(frame.overlay, c);
}

void appendEclipticGrid(LineBatch& out, const ProjectContext& c, const Vec3& lookAtWorld, double planeZ)
{
    const double extent = gridExtent(c);
    const double spacing = std::pow(10.0, std::floor(std::log10(extent / 6.0)));
    if (!(spacing > 0.0) || !std::isfinite(spacing)) return;
    const int half = static_cast<int>(std::ceil(extent / spacing));
    const double baseX = std::floor(lookAtWorld.x / spacing) * spacing - lookAtWorld.x;
    const double baseY = std::floor(lookAtWorld.y / spacing) * spacing - lookAtWorld.y;
    const double zRel = planeZ - lookAtWorld.z;
    const int samples = 2 * half;
    static thread_local std::vector<LineVertex> line;
    for (int axis = 0; axis < 2; ++axis) {
        for (int k = -half; k <= half; ++k) {
            const double fixed = (axis == 0 ? baseX : baseY) + k * spacing;
            const double worldFixed = fixed + (axis == 0 ? lookAtWorld.x : lookAtWorld.y);
            const bool major = std::fabs(std::remainder(worldFixed, 10.0 * spacing)) < 0.5 * spacing;
            line.clear();
            for (int s = 0; s <= samples; ++s) {
                const double run = (axis == 0 ? baseY : baseX) + (s - half) * spacing;
                LineVertex v;
                v.x = static_cast<float>(axis == 0 ? fixed : run);
                v.y = static_cast<float>(axis == 0 ? run : fixed);
                v.z = static_cast<float>(zRel);
                const double r = std::hypot(v.x, v.y) / extent;
                const double fade = std::max(0.0, 1.0 - r * r);
                v.r = 0.45f; v.g = 0.55f; v.b = 0.75f;
                v.a = static_cast<float>((major ? 0.22 : 0.10) * fade);
                line.push_back(v);
            }
            out.addStrip(line, 1.0f);
        }
    }
}

double wellSheetDepth(const std::vector<RenderWell>& wells, double x, double y, double uEdge, double depthMax)
{
    double phi = 0.0;
    for (const RenderWell& w : wells) {
        const double d = std::max(std::hypot(x - w.x, y - w.y), static_cast<double>(w.radius));
        if (d > 0.0) phi += static_cast<double>(w.mu) / d;
    }
    if (!(phi > 0.0)) return 0.0;
    const double t = std::clamp((std::log(phi) - uEdge) / kSheetSpan, 0.0, 1.0);
    return -depthMax * t;
}

void appendWellSheet(LineBatch& out, const std::vector<RenderWell>& wells, const ProjectContext& c, const Vec3& lookAtWorld, int lines)
{
    if (wells.empty() || lines < 2) return;
    const double extent = gridExtent(c) * 0.6;
    const double step = 2.0 * extent / (lines - 1);
    const int samples = 2 * (lines - 1);
    const double sub = 2.0 * extent / samples;
    // The sheet is fixed to the look-at point (not snapped): it is a picture of the field around what is being looked at.
    (void)lookAtWorld;
    // Edge potential: the shallowest corner, so the rim of the sheet sits at z = 0
    double uEdge = 1e300;
    for (int cx = -1; cx <= 1; cx += 2)
        for (int cy = -1; cy <= 1; cy += 2) {
            double phi = 0.0;
            for (const RenderWell& w : wells) {
                const double d = std::max(std::hypot(cx * extent - w.x, cy * extent - w.y), static_cast<double>(w.radius));
                if (d > 0.0) phi += static_cast<double>(w.mu) / d;
            }
            if (phi > 0.0) uEdge = std::min(uEdge, std::log(phi));
        }
    if (!(uEdge < 1e299)) return;
    const double depthMax = 0.5 * extent;
    static thread_local std::vector<LineVertex> line;
    for (int axis = 0; axis < 2; ++axis) {
        for (int k = 0; k < lines; ++k) {
            const double fixed = -extent + k * step;
            line.clear();
            for (int s = 0; s <= samples; ++s) {
                const double run = -extent + s * sub;
                LineVertex v;
                const double x = axis == 0 ? fixed : run, y = axis == 0 ? run : fixed;
                v.x = static_cast<float>(x);
                v.y = static_cast<float>(y);
                const double z = wellSheetDepth(wells, x, y, uEdge, depthMax);
                v.z = static_cast<float>(z);
                const double r = std::hypot(x, y) / (extent * 1.42);
                const double deep = -z / depthMax;
                v.r = static_cast<float>(0.35 + 0.25 * deep);
                v.g = static_cast<float>(0.45 + 0.15 * deep);
                v.b = 0.95f;
                v.a = static_cast<float>((0.18 + 0.30 * deep) * std::max(0.0, 1.0 - r * r));
                line.push_back(v);
            }
            out.addStrip(line, 1.0f);
        }
    }
}

} // namespace app
} // namespace cosmos
