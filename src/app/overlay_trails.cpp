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
    std::size_t phys;
    if (buf_.size() < kTrailCapacity) {
        phys = buf_.size();
        buf_.push_back(s);                   // growing: start_ stays 0
        count_ = buf_.size();
        if (boxes_.empty()) boxes_.resize(kTrailCapacity / kTrailBlock);
    } else if (count_ < buf_.size()) {       // after clear(): refill from the start
        phys = (start_ + count_) & (kTrailCapacity - 1);
        buf_[phys] = s;
        ++count_;
    } else {                                 // full: overwrite the oldest
        phys = start_;
        buf_[phys] = s;
        start_ = (start_ + 1) & (kTrailCapacity - 1);
    }
    // Block box: recomputed over everything the block holds when a block is started, extended by each sample after that.
    TrailBox& box = boxes_[phys / kTrailBlock];
    if ((phys & (kTrailBlock - 1)) == 0) {
        const std::size_t lo = phys, hi = std::min(lo + kTrailBlock, buf_.size());
        box = {buf_[lo].x, buf_[lo].x, buf_[lo].y, buf_[lo].y};
        for (std::size_t j = lo + 1; j < hi; ++j) {
            box.x0 = std::min(box.x0, buf_[j].x);
            box.x1 = std::max(box.x1, buf_[j].x);
            box.y0 = std::min(box.y0, buf_[j].y);
            box.y1 = std::max(box.y1, buf_[j].y);
        }
    } else {
        box.x0 = std::min(box.x0, s.x);
        box.x1 = std::max(box.x1, s.x);
        box.y0 = std::min(box.y0, s.y);
        box.y1 = std::max(box.y1, s.y);
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
    const std::size_t n = n_;
    if (n == 0) {
        x = y = 0.0;
        return;
    }
    if (n == 1) {
        x = trail_->sample(0).x;
        y = trail_->sample(0).y;
        return;
    }
    if (seg_ + 2 < n && trail_->sample(seg_ + 1).t <= t) {
        // Usually the next segment or two; a jump (the end of the trail after the start) is a binary search.
        if (seg_ + 2 >= n || trail_->sample(seg_ + 3 < n ? seg_ + 3 : n - 1).t > t) {
            while (seg_ + 2 < n && trail_->sample(seg_ + 1).t <= t) ++seg_;
        } else {
            std::size_t lo = seg_ + 1, hi = n - 2;   // the largest segment index in [lo, hi] whose start is <= t
            if (trail_->sample(hi).t <= t) lo = hi;
            else
                while (lo + 1 < hi) {
                    const std::size_t mid = lo + (hi - lo) / 2;
                    if (trail_->sample(mid).t <= t) lo = mid;
                    else hi = mid;
                }
            seg_ = lo;
        }
    }
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

    // View culling: the margin covers the line half width, the antialiasing fringe and a mitre.
    const bool cull = p.cullHalfW < 1e290 && p.cullHalfH < 1e290;
    const double margin = (p.cullMarginPx + p.widthPx) * p.metresPerPixel;
    const double limX = p.cullHalfW + margin, limY = p.cullHalfH + margin;
    auto chordVisible = [&](const LineVertex& a, const LineVertex& b) {
        if (!cull) return true;
        return !((a.x > limX && b.x > limX) || (a.x < -limX && b.x < -limX) || (a.y > limY && b.y > limY) ||
                 (a.y < -limY && b.y < -limY));
    };

    TrailCursor ca(&object), cb(origin);
    std::vector<LineVertex>& out = batch.vertices;
    double lastX = 0.0, lastY = 0.0;
    // The polyline an unculled draw would emit is the "accepted" vertices; a chord between two of them is emitted only when it
    // can reach the view. `acc` is the last accepted vertex, `prevAcc` the one before it. A strip is open while its last
    // vertex is `acc`.
    LineVertex acc, prevAcc;
    std::size_t accCount = 0, stripFirst = 0, total = 0;
    bool open = false;

    auto closeStrip = [&]() {
        const std::size_t n = out.size() - stripFirst;
        batch.strips.push_back({static_cast<std::uint32_t>(stripFirst), static_cast<std::uint32_t>(n), p.widthPx});
        total += n;
        open = false;
    };

    // `direct`: the object's own sample at exactly time t when there is no origin trail, where the cubic Hermite is the sample
    // itself (s = 0 gives h00 = 1 and the other weights exactly 0): no segment search, no evaluation.
    auto emit = [&](double t, bool force, const TrailSample* direct = nullptr) {
        double ax, ay, bx = 0.0, by = 0.0;
        if (direct != nullptr) {
            ax = direct->x;
            ay = direct->y;
        } else {
            ca.eval(t, ax, ay);
            if (origin) cb.eval(t, bx, by);
        }
        const double rx = (ax - bx) + p.offX, ry = (ay - by) + p.offY;
        const bool tooClose = accCount > 0 && ((rx - lastX) * (rx - lastX) + (ry - lastY) * (ry - lastY) < minStep2);
        if (tooClose && !force) return;
        const float alpha = static_cast<float>(p.colour.a * (1.0 - (tEnd - t) / span));
        const LineVertex v{static_cast<float>(rx), static_cast<float>(ry), p.colour.r, p.colour.g, p.colour.b,
                           std::max(alpha, 0.0f)};
        if (tooClose && accCount >= 2) {   // the end replaces a vertex that is almost the same place
            if (open) {
                out.back() = v;
            } else if (chordVisible(prevAcc, v)) {
                stripFirst = out.size();
                out.push_back(prevAcc);
                out.push_back(v);
                open = true;
            }
            acc = v;
            return;
        }
        if (accCount > 0) {
            if (chordVisible(acc, v)) {
                if (!open) {
                    stripFirst = out.size();
                    out.push_back(acc);
                    open = true;
                }
                out.push_back(v);
            } else if (open) {
                closeStrip();
            }
        }
        prevAcc = acc;
        acc = v;
        ++accCount;
        lastX = rx;
        lastY = ry;
    };

    // Vertex times: the union of the object's and the origin's sample times inside [tStart, tEnd].
    std::size_t ia = 0, ib = 0;
    const std::size_t na = object.sampleCount(), nb = origin ? origin->sampleCount() : 0;
    double t = tStart;
    emit(t, true);
    // Whole blocks of the ring that lie outside the view are skipped when there is no origin trail (the vertices are the samples
    // themselves): the first and the last sample of the run are processed, which keeps the chords that enter and leave it.
    const double eps = 1e-3 * p.metresPerPixel;
    for (;;) {
        if (cull && !origin && ia < na) {
            TrailBox box;
            const std::size_t run = object.blockRun(ia, box);
            if (run >= 3 && (box.x1 + p.offX < -limX - eps || box.x0 + p.offX > limX + eps || box.y1 + p.offY < -limY - eps ||
                             box.y0 + p.offY > limY + eps)) {
                const TrailSample& first = object.sample(ia);
                const TrailSample& last = object.sample(ia + run - 1);
                if (first.t > t && last.t < tEnd) {
                    emit(first.t, false, &first);
                    emit(last.t, false, &last);
                    t = last.t;
                    ia += run;
                    continue;
                }
            }
        }
        const double ta = ia < na ? object.sample(ia).t : inf;
        const double tb = ib < nb ? origin->sample(ib).t : inf;
        const double tn = std::min(ta, tb);
        if (tn == inf || tn >= tEnd) break;
        if (ta == tn) ++ia;
        if (tb == tn) ++ib;
        if (tn <= t) continue;     // before the start, or a duplicate time
        t = tn;
        emit(t, false, (!origin && ta == tn) ? &object.sample(ia - 1) : nullptr);
    }
    emit(tEnd, true);
    if (open) closeStrip();
    return total;
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
