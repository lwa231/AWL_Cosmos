#include "app/overlay_trails.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace cosmos {
namespace app {

void hermiteXY(const TrailSample& a, const TrailSample& b, double t, double& x, double& y)
{
    const double h = b.t - a.t;
    if (!(h > 0.0)) {
        x = a.x;
        y = a.y;
        return;
    }
    const double s = std::clamp((t - a.t) / h, 0.0, 1.0);
    const double s2 = s * s, s3 = s2 * s;
    const double h00 = 2.0 * s3 - 3.0 * s2 + 1.0;
    const double h10 = s3 - 2.0 * s2 + s;
    const double h01 = -2.0 * s3 + 3.0 * s2;
    const double h11 = s3 - s2;
    x = h00 * a.x + h10 * h * a.vx + h01 * b.x + h11 * h * b.vx;
    y = h00 * a.y + h10 * h * a.vy + h01 * b.y + h11 * h * b.vy;
}

void Trail::store(const TrailSample& s)
{
    if (buf_.size() < kTrailCapacity) {
        buf_.push_back(s);                   // growing: start_ stays 0
        count_ = buf_.size();
    } else if (count_ < buf_.size()) {       // after clear(): refill from the start
        buf_[(start_ + count_) % buf_.size()] = s;
        ++count_;
    } else {                                 // full: overwrite the oldest
        buf_[start_] = s;
        start_ = (start_ + 1) % buf_.size();
    }
}

bool Trail::record(const TrailSample& s, double minSpacing, double turnRad)
{
    haveHead_ = true;
    head_ = s;
    if (count_ == 0) {
        store(s);
        return true;
    }
    const TrailSample& l = stored(count_ - 1);
    if (!(s.t > l.t)) return false;
    const double dx = s.x - l.x, dy = s.y - l.y;
    bool keep = dx * dx + dy * dy >= minSpacing * minSpacing;
    if (!keep) {
        const double a = std::hypot(s.vx, s.vy), b = std::hypot(l.vx, l.vy);
        if (a > 0.0 && b > 0.0) keep = (s.vx * l.vx + s.vy * l.vy) / (a * b) < std::cos(turnRad);
    }
    if (keep) store(s);
    return keep;
}

void TrailCursor::eval(double t, double& x, double& y)
{
    const std::size_t n = trail_ ? trail_->sampleCount() : 0;
    if (n == 0) {
        x = y = 0.0;
        return;
    }
    if (n == 1) {
        x = trail_->sample(0).x;
        y = trail_->sample(0).y;
        return;
    }
    while (seg_ + 2 < n && trail_->sample(seg_ + 1).t <= t) ++seg_;
    hermiteXY(trail_->sample(seg_), trail_->sample(seg_ + 1), t, x, y);
}

std::size_t appendTrail(const Trail& object, const Trail* origin, const TrailDrawParams& p, LineBatch& batch)
{
    if (object.empty() || (origin && origin->empty())) return 0;
    const double inf = std::numeric_limits<double>::infinity();

    double tStart = std::max(object.firstTime(), p.tailTime);
    double tEnd = std::min(object.lastTime(), p.now);
    if (origin) {
        tStart = std::max(tStart, origin->firstTime());
        tEnd = std::min(tEnd, origin->lastTime());
    }
    if (!(tEnd > tStart)) return 0;
    const double span = tEnd - tStart;
    const double minStep = p.minStepPx * p.metresPerPixel;
    const double minStep2 = minStep * minStep;

    TrailCursor ca(&object), cb(origin);
    std::vector<LineVertex>& out = batch.vertices;
    const std::size_t first = out.size();
    double lastX = 0.0, lastY = 0.0;
    bool have = false;

    auto emit = [&](double t, bool force) {
        double ax, ay, bx = 0.0, by = 0.0;
        ca.eval(t, ax, ay);
        if (origin) cb.eval(t, bx, by);
        const double rx = (ax - bx) + p.offX, ry = (ay - by) + p.offY;
        const bool tooClose = have && ((rx - lastX) * (rx - lastX) + (ry - lastY) * (ry - lastY) < minStep2);
        if (tooClose && !force) return;
        const float alpha = static_cast<float>(p.colour.a * (1.0 - (tEnd - t) / span));
        const LineVertex v{static_cast<float>(rx), static_cast<float>(ry), p.colour.r, p.colour.g, p.colour.b,
                           std::max(alpha, 0.0f)};
        if (tooClose && out.size() - first >= 2) {   // the end replaces a vertex that is almost the same place
            out.back() = v;
            return;
        }
        out.push_back(v);
        lastX = rx;
        lastY = ry;
        have = true;
    };

    // Vertex times: the union of the object's and the origin's sample times inside [tStart, tEnd].
    std::size_t ia = 0, ib = 0;
    const std::size_t na = object.sampleCount(), nb = origin ? origin->sampleCount() : 0;
    double t = tStart;
    emit(t, true);
    for (;;) {
        const double ta = ia < na ? object.sample(ia).t : inf;
        const double tb = ib < nb ? origin->sample(ib).t : inf;
        const double tn = std::min(ta, tb);
        if (tn == inf || tn >= tEnd) break;
        if (ta == tn) ++ia;
        if (tb == tn) ++ib;
        if (tn <= t) continue;     // before the start, or a duplicate time
        t = tn;
        emit(t, false);
    }
    emit(tEnd, true);

    const std::size_t n = out.size() - first;
    if (n < 2) {
        out.resize(first);
        return 0;
    }
    batch.strips.push_back({static_cast<std::uint32_t>(first), static_cast<std::uint32_t>(n), p.widthPx});
    return n;
}

Trail& TrailStore::touch(std::uint64_t id, std::uint64_t frameStamp)
{
    Entry& e = trails_[id];
    e.stamp = frameStamp;
    return e.trail;
}

const Trail* TrailStore::find(std::uint64_t id) const
{
    const auto it = trails_.find(id);
    return it == trails_.end() ? nullptr : &it->second.trail;
}

void TrailStore::pruneUntouched(std::uint64_t frameStamp)
{
    for (auto it = trails_.begin(); it != trails_.end();) {
        if (it->second.stamp != frameStamp) it = trails_.erase(it);
        else ++it;
    }
}

} // namespace app
} // namespace cosmos
