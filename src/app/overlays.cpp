#include "app/overlays.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include "core/atmosphere.h"
#include "core/gravity.h"

namespace cosmos {
namespace app {

namespace {

const double kPi = 3.141592653589793238462643383279502884;

constexpr Rgba kVelocityColour{0.30f, 0.95f, 0.45f, 0.95f};
constexpr Rgba kAccelerationColour{1.00f, 0.60f, 0.20f, 0.95f};
constexpr float kDefaultShuttleColour[3] = {0.75f, 0.88f, 1.0f};
constexpr double kFallbackHorizon = 1.0e6;          // s, a target with no parent (the heaviest body)
constexpr double kHorizonPeriods = 1.1;             // automatic horizon in osculating periods
constexpr double kMinHorizon = 10.0;                // s
constexpr double kMarkPx = 4.0;                     // half size of the periapsis / apoapsis marks

// A target's osculating state relative to its parent node.
struct ParentRel {
    bool ok = false;
    int node = -1;
    Vec3 parentPos, parentVel;
    Vec3 r, v;        // relative to the parent node
    double mu = 0.0;  // parent node mu + the body's own (0 for a shuttle)
};

ParentRel bodyParentRel(const Hierarchy& h, const DisplayState& st, std::size_t i)
{
    ParentRel p;
    if (i >= h.bodyParent.size() || h.bodyParent[i] < 0) return p;
    const HierarchyNode& n = h.nodes[static_cast<std::size_t>(h.bodyParent[i])];
    const DisplayBody& b = st.bodies[i];
    p.ok = n.mu + b.mu > 0.0;
    p.node = h.bodyParent[i];
    p.parentPos = n.r;
    p.parentVel = n.v;
    p.r = b.r - n.r;
    p.v = b.v - n.v;
    p.mu = n.mu + b.mu;
    return p;
}

ParentRel shuttleParentRel(const Hierarchy& h, const DisplayShuttle& s)
{
    ParentRel p;
    if (h.root < 0) return p;
    const int node = shuttleParent(h, s.r);
    if (node < 0) return p;
    const HierarchyNode& n = h.nodes[static_cast<std::size_t>(node)];
    p.ok = n.mu > 0.0;
    p.node = node;
    p.parentPos = n.r;
    p.parentVel = n.v;
    p.r = s.r - n.r;
    p.v = s.v - n.v;
    p.mu = n.mu;      // a shuttle is massless
    return p;
}

} // namespace

OverlayView OverlayView::fromCamera(const Camera& c)
{
    OverlayView v;
    v.metresPerPixel = c.metresPerPixel();
    v.centre = c.centreWorld();
    v.frameMode = c.frameMode();
    v.frameBody = c.frameBody();
    v.follow = c.following();
    return v;
}

OverlayBuilder::OverlayBuilder() = default;

void OverlayBuilder::reset()
{
    trails_.clear();
    predictor_.cancel();
    haveTime_ = false;
    reqLive_ = false;
    reqTarget_ = {};
    reqReal_ = -1e300;
    haveHier_ = false;
}

void OverlayBuilder::onObjectRemoved(std::uint64_t id)
{
    trails_.erase(id);
    if (reqLive_ && reqTarget_.id == id) {
        predictor_.cancel();
        reqLive_ = false;
    }
}

void OverlayBuilder::onEvent(const Event& e)
{
    if (e.type == EventType::Merge) onObjectRemoved(e.other);
    else if (e.type == EventType::ShuttleDestroyed) onObjectRemoved(e.id);
}

Rgba OverlayBuilder::objectColour(const DisplayBody& b, float alpha) const
{
    return {b.colour[0], b.colour[1], b.colour[2], alpha};
}

Rgba OverlayBuilder::objectColour(const DisplayShuttle& s, float alpha) const
{
    if (palette_) {
        const SrgbColour c = palette_->coloursFor(s.id)[0];
        return {c.r, c.g, c.b, alpha};
    }
    return {kDefaultShuttleColour[0], kDefaultShuttleColour[1], kDefaultShuttleColour[2], alpha};
}

OverlayBuilder::FrameOrigin OverlayBuilder::frameOrigin(const DisplayState& st, const OverlayView& view) const
{
    FrameOrigin o;
    if (view.frameMode == FrameMode::FollowBody) {
        for (const DisplayBody& b : st.bodies) {
            if (b.id != view.frameBody) continue;
            o.mode = FrameMode::FollowBody;
            o.body = b.id;
            o.pos = {b.r.x, b.r.y, 0.0};
            o.vel = {b.v.x, b.v.y, 0.0};
            return o;
        }
        return o;   // the body vanished: the camera falls back to Inertial too
    }
    if (view.frameMode == FrameMode::Barycentre) {
        double m = 0.0, x = 0.0, y = 0.0, vx = 0.0, vy = 0.0;
        for (const DisplayBody& b : st.bodies) {
            m += b.mu;
            x += b.mu * b.r.x;
            y += b.mu * b.r.y;
            vx += b.mu * b.v.x;
            vy += b.mu * b.v.y;
        }
        if (m > 0.0) {
            o.mode = FrameMode::Barycentre;
            o.pos = {x / m, y / m, 0.0};
            o.vel = {vx / m, vy / m, 0.0};
        }
    }
    return o;
}

void OverlayBuilder::ensureHierarchy(const DisplayState& st)
{
    if (haveHier_) return;
    const std::size_t n = st.bodies.size();
    hr_.resize(n);
    hv_.resize(n);
    hmu_.resize(n);
    for (std::size_t i = 0; i < n; ++i) {
        hr_[i] = st.bodies[i].r;
        hv_[i] = st.bodies[i].v;
        hmu_[i] = st.bodies[i].mu;
    }
    hier_ = buildHierarchy(hr_, hv_, hmu_);
    haveHier_ = true;
}

// ---------------------------------------------------------------------------------------------------------------

void OverlayBuilder::build(const SimDriver& driver, std::uint64_t generation, const OverlayView& view, RenderFrame& frame)
{
    build(driver.state(), driver.sim(), generation, view, frame);
}

void OverlayBuilder::build(const DisplayState& st, const Simulation& sim, std::uint64_t generation, const OverlayView& view,
                           RenderFrame& frame)
{
    if (generation != generation_ || (haveTime_ && st.t < lastT_)) reset();
    generation_ = generation;
    lastT_ = st.t;
    haveTime_ = true;
    ++stamp_;
    haveHier_ = false;
    stats_ = OverlayStats{};

    OverlayView v = view;
    if (!(v.metresPerPixel > 0.0) || !std::isfinite(v.metresPerPixel)) v.metresPerPixel = 1.0;
    const FrameOrigin origin = frameOrigin(st, v);

    using Clock = std::chrono::steady_clock;
    auto ms = [](Clock::time_point a, Clock::time_point b) { return std::chrono::duration<double, std::milli>(b - a).count(); };

    // Trails: record when shown (or when asked to keep the history), drop the history when switched off.
    const Clock::time_point c0 = Clock::now();
    const bool recording = settings.trails || settings.keepTrailsWhileHidden;
    if (recording) {
        recordTrails(st, v);
        trailsWereOn_ = true;
    } else if (trailsWereOn_) {
        trails_.clear();
        trailsWereOn_ = false;
    }
    const Clock::time_point c1 = Clock::now();
    if (settings.trails) drawTrails(st, v, origin, frame);
    const Clock::time_point c2 = Clock::now();
    if (settings.osculatingOrbits) drawOrbits(st, v, frame);
    const Clock::time_point c3 = Clock::now();
    updatePrediction(st, sim, v, origin, frame);
    const Clock::time_point c4 = Clock::now();
    if (settings.velocityArrows || settings.accelerationArrows) drawArrows(st, sim, v, origin, frame);
    const Clock::time_point c5 = Clock::now();
    stats_.trailRecordMs = ms(c0, c1);
    stats_.trailDrawMs = ms(c1, c2);
    stats_.orbitsMs = ms(c2, c3);
    stats_.predictMs = ms(c3, c4);
    stats_.arrowsMs = ms(c4, c5);
}

// ---------------------------------------------------------------------------------------------------------------
// Trails

void OverlayBuilder::recordTrails(const DisplayState& st, const OverlayView& view)
{
    const double spacing = kTrailSpacingPx * view.metresPerPixel;
    auto rec = [&](std::uint64_t id, const Vec3& r, const Vec3& v) {
        trails_.touch(id, stamp_).record({st.t, r.x, r.y, v.x, v.y}, spacing);
    };
    double m = 0.0, x = 0.0, y = 0.0, vx = 0.0, vy = 0.0;
    for (const DisplayBody& b : st.bodies) {
        rec(b.id, b.r, b.v);
        m += b.mu;
        x += b.mu * b.r.x;
        y += b.mu * b.r.y;
        vx += b.mu * b.v.x;
        vy += b.mu * b.v.y;
    }
    for (const DisplayShuttle& s : st.shuttles) rec(s.id, s.r, s.v);
    if (m > 0.0) rec(kBarycentreKey, {x / m, y / m, 0.0}, {vx / m, vy / m, 0.0});
    trails_.pruneUntouched(stamp_);
}

void OverlayBuilder::drawTrails(const DisplayState& st, const OverlayView& view, const FrameOrigin& o, RenderFrame& frame)
{
    const Trail* originTrail = nullptr;
    std::uint64_t originId = 0;
    if (o.mode == FrameMode::FollowBody) {
        originId = o.body;
        originTrail = trails_.find(o.body);
        if (!originTrail) return;
    } else if (o.mode == FrameMode::Barycentre) {
        originId = kBarycentreKey;
        originTrail = trails_.find(kBarycentreKey);
        if (!originTrail) return;
    }

    TrailDrawParams p;
    p.now = st.t;
    p.tailTime = settings.trailSeconds > 0.0 ? st.t - settings.trailSeconds : -1e300;
    p.offX = o.pos.x - view.centre.x;
    p.offY = o.pos.y - view.centre.y;
    p.metresPerPixel = view.metresPerPixel;

    auto draw = [&](std::uint64_t id, const Rgba& colour) {
        if (id == originId) return;    // the frame origin sits still in its own frame
        const Trail* t = trails_.find(id);
        if (!t) return;
        p.colour = colour;
        stats_.trailVertices += appendTrail(*t, originTrail, p, frame.underlay);
        ++stats_.trailObjects;
    };
    if (settings.onBodies)
        for (const DisplayBody& b : st.bodies) draw(b.id, objectColour(b, settings.trailAlpha));
    if (settings.onShuttles)
        for (const DisplayShuttle& s : st.shuttles) draw(s.id, objectColour(s, settings.trailAlpha));
}

// ---------------------------------------------------------------------------------------------------------------
// Osculating orbits

void OverlayBuilder::appendOrbit(const Vec3& parentPos, const Vec3&, double muTotal, const Vec3& r, const Vec3& v,
                                 const Rgba& colour, const OverlayView& view, RenderFrame& frame)
{
    const Elements el = stateToElements(r, v, muTotal);

    const double mpp = view.metresPerPixel;
    const double halfDiag = 0.5 * std::hypot(static_cast<double>(frame.viewportWidth), static_cast<double>(frame.viewportHeight)) * mpp;
    const double dx = parentPos.x - view.centre.x, dy = parentPos.y - view.centre.y;
    const double dist = std::hypot(dx, dy);
    const double rCap = std::max(20.0 * el.q, 2.0 * (2.0 * halfDiag + dist));

    if (!buildOrbitCurve(el, rCap, settings.orbitSegments, curve_)) return;
    if (el.e < 1.0 && curve_.radiusMax < 1.5 * mpp) return;               // smaller than a dot
    if (dist - curve_.radiusMax > halfDiag) return;                       // the loop's circle misses the view entirely

    LineBatch& b = frame.underlay;
    const std::size_t first = b.vertices.size();
    for (const Vec3& q : curve_.points)
        b.vertices.push_back({static_cast<float>(dx + q.x), static_cast<float>(dy + q.y), colour.r, colour.g, colour.b, colour.a});
    b.strips.push_back({static_cast<std::uint32_t>(first), static_cast<std::uint32_t>(curve_.points.size()), 1.0f});

    // Periapsis: a '+', apoapsis: an 'x', both kMarkPx wide on screen.
    const double k = kMarkPx * mpp;
    const Rgba mark{std::min(1.0f, colour.r + 0.35f), std::min(1.0f, colour.g + 0.35f), std::min(1.0f, colour.b + 0.35f),
                    std::min(1.0f, colour.a + 0.3f)};
    auto seg = [&](double cx, double cy, double ax, double ay, double bx, double by) {
        const LineVertex p0{static_cast<float>(cx + ax), static_cast<float>(cy + ay), mark.r, mark.g, mark.b, mark.a};
        const LineVertex p1{static_cast<float>(cx + bx), static_cast<float>(cy + by), mark.r, mark.g, mark.b, mark.a};
        b.addSegment(p0, p1, 1.0f);
    };
    seg(dx + curve_.periapsis.x, dy + curve_.periapsis.y, -k, 0.0, k, 0.0);
    seg(dx + curve_.periapsis.x, dy + curve_.periapsis.y, 0.0, -k, 0.0, k);
    if (curve_.hasApoapsis) {
        const double s = k * 0.7071067811865476;
        seg(dx + curve_.apoapsis.x, dy + curve_.apoapsis.y, -s, -s, s, s);
        seg(dx + curve_.apoapsis.x, dy + curve_.apoapsis.y, -s, s, s, -s);
    }
    ++stats_.orbits;
}

void OverlayBuilder::drawOrbits(const DisplayState& st, const OverlayView& view, RenderFrame& frame)
{
    ensureHierarchy(st);
    if (settings.onBodies) {
        for (std::size_t i = 0; i < st.bodies.size(); ++i) {
            if (st.bodies[i].absorbed) continue;
            const ParentRel p = bodyParentRel(hier_, st, i);
            if (!p.ok) continue;
            appendOrbit(p.parentPos, p.parentVel, p.mu, p.r, p.v, objectColour(st.bodies[i], settings.orbitAlpha), view, frame);
        }
    }
    if (settings.onShuttles) {
        for (const DisplayShuttle& s : st.shuttles) {
            if (s.dying) continue;
            const ParentRel p = shuttleParentRel(hier_, s);
            if (!p.ok) continue;
            appendOrbit(p.parentPos, p.parentVel, p.mu, p.r, p.v, objectColour(s, settings.orbitAlpha), view, frame);
        }
    }
}

// ---------------------------------------------------------------------------------------------------------------
// Predicted path

double OverlayBuilder::predictionHorizon(const DisplayState& st, const PredictRequest& target)
{
    if (settings.predictSeconds > 0.0) return std::min(settings.predictSeconds, settings.predictMaxSeconds);
    ensureHierarchy(st);
    ParentRel p;
    if (target.kind == TargetKind::Body) {
        for (std::size_t i = 0; i < st.bodies.size(); ++i)
            if (st.bodies[i].id == target.id) p = bodyParentRel(hier_, st, i);
    } else {
        for (const DisplayShuttle& s : st.shuttles)
            if (s.id == target.id) p = shuttleParentRel(hier_, s);
    }
    double h = kFallbackHorizon;
    if (p.ok) {
        const Elements el = stateToElements(p.r, p.v, p.mu);
        if (el.e < 1.0 && el.a > 0.0 && std::isfinite(el.a)) {
            h = kHorizonPeriods * 2.0 * kPi * std::sqrt(el.a * el.a * el.a / p.mu);
        } else {
            const double sp = std::hypot(p.v.x, p.v.y);
            h = sp > 0.0 ? 10.0 * std::hypot(p.r.x, p.r.y) / sp : kFallbackHorizon;   // ten current distances at the current speed
        }
    }
    if (!std::isfinite(h)) h = kFallbackHorizon;
    return std::clamp(h, kMinHorizon, std::max(kMinHorizon, settings.predictMaxSeconds));
}

void OverlayBuilder::updatePrediction(const DisplayState& st, const Simulation& sim, const OverlayView& view,
                                      const FrameOrigin& o, RenderFrame& frame)
{
    if (!settings.predictedPath) {
        if (reqLive_) {
            predictor_.cancel();
            reqLive_ = false;
        }
        return;
    }
    Target target = predTarget_;
    if (target.kind == TargetKind::None) {
        if (view.follow.kind == TargetKind::Body) target = {TargetKind::Body, view.follow.id};
        else if (view.follow.kind == TargetKind::Shuttle) target = {TargetKind::Shuttle, view.follow.id};
    }
    // The target must exist in the display state.
    const DisplayBody* tb = nullptr;
    const DisplayShuttle* ts = nullptr;
    if (target.kind == TargetKind::Body) {
        for (const DisplayBody& b : st.bodies)
            if (b.id == target.id) tb = &b;
    } else if (target.kind == TargetKind::Shuttle) {
        for (const DisplayShuttle& s : st.shuttles)
            if (s.id == target.id && !s.dying) ts = &s;
    }
    if (!tb && !ts) {
        if (reqLive_) {
            predictor_.cancel();
            reqLive_ = false;
        }
        return;
    }

    predictor_.poll();
    stats_.predictionPending = predictor_.busy();

    // New request? A different target / run, none yet, or the path is running out or getting old.
    const bool sameTarget = reqLive_ && reqTarget_.kind == target.kind && reqTarget_.id == target.id && reqGeneration_ == generation_;
    bool want = !sameTarget;
    if (sameTarget) {
        const double age = st.t - reqT0_;
        if (age >= reqHorizon_) want = true;                                           // the path is used up
        else if (age > settings.predictRefresh * reqHorizon_ && !predictor_.busy()) want = true;   // getting old
        if (want && frame.time - reqReal_ < settings.predictMinRealGap && age < reqHorizon_) want = false;
    }
    if (want) {
        PredictRequest req;
        req.kind = target.kind;
        req.id = target.id;
        req.samples = settings.predictSamples;
        req.sim = predictorOptions_;
        req.horizon_s = predictionHorizon(st, req);
        if (predictor_.request(sim, req) != 0) {
            reqLive_ = true;
            reqTarget_ = target;
            reqGeneration_ = generation_;
            reqT0_ = sim.time();
            reqHorizon_ = req.horizon_s;
            reqReal_ = frame.time;
            stats_.predictionPending = true;
        }
    }

    // Draw what the worker has handed over, if it belongs to this target.
    const PredictedPath& path = predictor_.result();
    if (!path.valid() || path.kind != target.kind || path.id != target.id) return;

    int col = -1;
    if (o.mode == FrameMode::FollowBody) {
        col = path.bodyColumn(o.body);
        if (col < 0) return;
    }
    const std::size_t nb = path.bodyIds.size();
    const double offX = o.pos.x - view.centre.x, offY = o.pos.y - view.centre.y;
    const Vec3 headWorld = tb ? tb->r : ts->r;
    const Rgba base = tb ? objectColour(*tb, 1.0f) : objectColour(*ts, 1.0f);
    // Lightened towards white, fading along the path.
    const float cr = 0.5f * (base.r + 1.0f), cg = 0.5f * (base.g + 1.0f), cb = 0.5f * (base.b + 1.0f);

    LineBatch& b = frame.underlay;
    const std::size_t first = b.vertices.size();
    b.vertices.push_back({static_cast<float>(headWorld.x - view.centre.x), static_cast<float>(headWorld.y - view.centre.y), cr, cg,
                          cb, 0.9f});
    for (std::size_t k = 0; k < path.count; ++k) {
        if (!(path.timeOf(k) > st.t)) continue;
        Vec3 origin;
        if (o.mode == FrameMode::Barycentre) origin = path.barycentre[k];
        else if (o.mode == FrameMode::FollowBody) {
            origin = path.bodies[k * nb + static_cast<std::size_t>(col)];
            if (!std::isfinite(origin.x)) break;   // the frame body is gone from here on
        }
        const double alpha = 0.9 * (1.0 - 0.8 * static_cast<double>(k) / static_cast<double>(path.count));
        b.vertices.push_back({static_cast<float>((path.target[k].x - origin.x) + offX),
                              static_cast<float>((path.target[k].y - origin.y) + offY), cr, cg, cb, static_cast<float>(alpha)});
    }
    const std::size_t n = b.vertices.size() - first;
    if (n < 2) {
        b.vertices.resize(first);
        return;
    }
    b.strips.push_back({static_cast<std::uint32_t>(first), static_cast<std::uint32_t>(n), 1.5f});
    stats_.predictedVertices = n;
    stats_.predictionDrawn = true;
}

// ---------------------------------------------------------------------------------------------------------------
// Arrows

void OverlayBuilder::drawArrows(const DisplayState& st, const Simulation& sim, const OverlayView& view, const FrameOrigin& o,
                                RenderFrame& frame)
{
    const double mpp = view.metresPerPixel;
    auto emit = [&](double reference) {
        double ref = reference;
        if (!(ref > 0.0))
            for (const ArrowItem& a : arrowItems_) ref = std::max(ref, a.mag);
        for (const ArrowItem& a : arrowItems_) {
            const double len = arrowLengthPx(settings.arrowMode, a.mag, ref, settings.arrowLengthPx);
            if (appendArrow(frame.overlay, a.x, a.y, a.dx, a.dy, len, mpp, a.colour)) ++stats_.arrows;
        }
    };

    // ---- velocity, in the selected frame ----
    if (settings.velocityArrows) {
        arrowItems_.clear();
        if (settings.onBodies)
            for (const DisplayBody& b : st.bodies) {
                if (b.absorbed) continue;
                const double vx = b.v.x - o.vel.x, vy = b.v.y - o.vel.y;
                arrowItems_.push_back({b.r.x - view.centre.x, b.r.y - view.centre.y, vx, vy, std::hypot(vx, vy), kVelocityColour});
            }
        if (settings.onShuttles)
            for (const DisplayShuttle& s : st.shuttles) {
                if (s.dying) continue;
                const double vx = s.v.x - o.vel.x, vy = s.v.y - o.vel.y;
                arrowItems_.push_back({s.r.x - view.centre.x, s.r.y - view.centre.y, vx, vy, std::hypot(vx, vy), kVelocityColour});
            }
        emit(settings.velocityReference);
    }

    // ---- acceleration (inertial: the gravity of all bodies, plus thrust and drag for a shuttle) ----
    if (settings.accelerationArrows) {
        arrowItems_.clear();
        const std::size_t n = st.bodies.size();
        if (settings.onBodies && n > 0) {
            accState_.x.resize(n); accState_.y.resize(n); accState_.z.resize(n);
            accState_.vx.assign(n, 0.0); accState_.vy.assign(n, 0.0); accState_.vz.assign(n, 0.0);
            accMu_.resize(n);
            accRadius_.resize(n);
            for (std::size_t i = 0; i < n; ++i) {
                accState_.x[i] = st.bodies[i].r.x;
                accState_.y[i] = st.bodies[i].r.y;
                accState_.z[i] = 0.0;
                accMu_[i] = st.bodies[i].mu;
                accRadius_[i] = st.bodies[i].radius;
            }
            computeAccelerations(accState_, accMu_, accRadius_, acc_);
            for (std::size_t i = 0; i < n; ++i) {
                const DisplayBody& b = st.bodies[i];
                if (b.absorbed) continue;
                const double ax = acc_.ax[i], ay = acc_.ay[i];
                arrowItems_.push_back({b.r.x - view.centre.x, b.r.y - view.centre.y, ax, ay, std::hypot(ax, ay), kAccelerationColour});
            }
        }
        if (settings.onShuttles) {
            const Shuttles& shuttles = sim.shuttles();
            for (const DisplayShuttle& s : st.shuttles) {
                if (s.dying || s.index >= shuttles.size() || shuttles.alive[s.index] == 0) continue;
                double ax = 0.0, ay = 0.0;
                const double mass = shuttles.mass(s.index);
                for (const DisplayBody& b : st.bodies) {
                    const double dx = b.r.x - s.r.x, dy = b.r.y - s.r.y;
                    const double d2 = std::max(dx * dx + dy * dy, 1e-6);
                    const double f = b.mu / (d2 * std::sqrt(d2));
                    ax += f * dx;
                    ay += f * dy;
                    if (b.atmosphere) {   // exact-drag model of the physics: a = -k |v - v_air| (v - v_air)
                        const double h = std::sqrt(d2) - b.radius;
                        const double rho = dragDensity(*b.atmosphere, h);
                        if (rho > 0.0 && mass > 0.0) {
                            const Vec3 air = airVelocity(b.v, b.spinRate, b.r, s.r);
                            const double rx = s.v.x - air.x, ry = s.v.y - air.y;
                            const double k = dragRate(rho, shuttles.cdA[s.index], mass) * std::hypot(rx, ry);
                            ax -= k * rx;
                            ay -= k * ry;
                        }
                    }
                }
                if (mass > 0.0) {
                    const double thrust = s.throttle * shuttles.maxThrust[s.index] / mass;
                    ax += thrust * std::cos(s.heading);
                    ay += thrust * std::sin(s.heading);
                }
                arrowItems_.push_back({s.r.x - view.centre.x, s.r.y - view.centre.y, ax, ay, std::hypot(ax, ay), kAccelerationColour});
            }
        }
        emit(settings.accelerationReference);
    }
}

} // namespace app
} // namespace cosmos
